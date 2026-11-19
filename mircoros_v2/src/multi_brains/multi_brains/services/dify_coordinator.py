# -*- coding: utf-8 -*-
"""Dify Agent interaction: SSE chat calls, image upload, and response parsing.

Owns the ``ChatClient`` lifecycle, the current conversation id, and the
streaming-response → action/response parsing. Keeping the conversation id here
lets the node reset it atomically (after a ``finishtask`` result is published)
without exposing the lock.
"""

import os
import threading
import time

from ..core.feedback_protocol import build_dify_inputs
from ..utils.common_tools import extract_json_content
from ..utils.dify_client import ChatClient


class DifyCoordinator:
    """Wrap Dify chat messaging with image upload and JSON parsing."""

    def __init__(self, node, config):
        self.node = node

        dify_base_url = config.get("DIFY_BASE_URL")
        dify_api_key = config.get("DIFY_API_KEY", "")
        self.conversation_id = None
        self.conversation_lock = threading.Lock()
        self.call_lock = threading.Lock()

        masked_key = (
            dify_api_key[:8] + "****" + dify_api_key[-4:]
            if dify_api_key and len(dify_api_key) > 12
            else "****"
        )
        self.node.get_logger().info(
            f"[DIFY] 初始化客户端: base_url={dify_base_url}, api_key={masked_key}"
        )

        self.client = None
        if not dify_api_key:
            self.node.get_logger().error("[DIFY] DIFY_API_KEY 未配置!")
            return

        self.client = ChatClient(dify_api_key, dify_base_url)
        self._start_connection_check()

    def _start_connection_check(self) -> None:
        """Check Dify connectivity without blocking ROS node construction."""
        thread = threading.Thread(
            target=self._check_connection,
            name="dify-connection-check",
            daemon=True,
        )
        thread.start()

    def _check_connection(self) -> None:
        """Log whether the configured Dify endpoint is reachable."""
        self.node.get_logger().info("[DIFY] 正在测试连接...")
        try:
            response = self.client.get_meta("llm_agent")
            if response.status_code == 200:
                self.node.get_logger().info("[DIFY] 连接成功 ✓")
            else:
                self.node.get_logger().error(
                    f"[DIFY] 连接失败: HTTP {response.status_code}, "
                    f"body={response.text[:200]}"
                )
        except Exception as error:
            self.node.get_logger().error(f"[DIFY] 连接异常: {error}")

    # ---- conversation id management ------------------------------------

    def get_conversation_id(self) -> str | None:
        """Thread-safe read of the current conversation id."""
        with self.conversation_lock:
            return self.conversation_id

    def set_conversation_id(self, new_id: str | None) -> None:
        """Thread-safe update of the current conversation id."""
        with self.conversation_lock:
            self.conversation_id = new_id

    def reset_conversation_id(self) -> None:
        """Clear the conversation id (after a finishtask result is published)."""
        self.set_conversation_id(None)

    # ---- chat call -----------------------------------------------------

    def call_agent(
        self,
        user_text: str,
        robot_feedback=None,
        image_paths: list | None = None,
        conversation_id: str | None = None,
        update_conversation_id: bool = True,
    ) -> dict | None:
        """Call the Dify Agent in streaming SSE mode.

        Dify Agent applications do not support blocking mode, so streaming is
        used uniformly; ``message`` / ``agent_message`` answer fragments are
        accumulated internally and a blocking-style result dict is returned.
        """
        if not self.client:
            self.node.get_logger().error("[DIFY] 客户端未初始化，无法调用")
            return None

        image_paths = image_paths or []
        image_file_objects = self.upload_images(image_paths)
        if image_paths and image_file_objects is None:
            return None

        inputs = build_dify_inputs(robot_feedback, image_file_objects)
        active_conversation_id = (
            self.get_conversation_id() if conversation_id is None else conversation_id
        )
        kwargs = {
            "inputs": inputs,
            "query": user_text,
            "user": "llm_agent",
            "conversation_id": active_conversation_id,
        }
        if image_file_objects:
            kwargs["files"] = image_file_objects

        if self.node.debug_mode:
            self.node.get_logger().info(
                f"[DIFY] >>> 发送请求: query='{user_text}', "
                f"robot_feedback={inputs['robot_feedback']}, "
                f"images={len(image_file_objects or [])}, "
                f"conversation_id={active_conversation_id}"
            )

        max_retries = 2
        last_error = None
        for attempt in range(max_retries + 1):
            try:
                with self.call_lock:
                    result = self.client.create_chat_message_stream(**kwargs)
                    if self.node.debug_mode:
                        self.node.get_logger().info("[DIFY] <<< streaming 完成")

                    response_conversation_id = result.get("conversation_id")
                    if (
                        update_conversation_id
                        and not self.node.should_interrupt()
                        and response_conversation_id
                    ):
                        self.set_conversation_id(response_conversation_id)

                    if self.node.debug_mode:
                        message_id = result.get("message_id", "")
                        answer = result.get("answer", "")
                        self.node.get_logger().info(
                            f"[DIFY] message_id={message_id}, "
                            f"conversation_id={response_conversation_id}"
                        )
                        self.node.get_logger().info(
                            f"[DIFY] 完整answer({len(answer)}字符): {answer[:300]}"
                        )

                return result
            except RuntimeError as error:
                last_error = error
                self.node.get_logger().warn(
                    f"[DIFY] streaming 错误 (第{attempt+1}次): {error}"
                )
            except Exception as error:
                last_error = error
                self.node.get_logger().warn(
                    f"[DIFY] 调用异常 (第{attempt+1}次): "
                    f"{type(error).__name__}: {error}"
                )
            if attempt < max_retries:
                wait = 2 * (attempt + 1)
                self.node.get_logger().info(f"[DIFY] {wait}秒后重试...")
                time.sleep(wait)
                if self.node.should_interrupt():
                    return None

        self.node.get_logger().error(f"[DIFY] 重试{max_retries}次后仍失败: {last_error}")
        return None

    def upload_images(self, image_paths: list) -> list | None:
        """Upload feedback images and return Dify chat-messages file objects."""
        image_file_objects = []
        for image_index, image_path in enumerate(image_paths, start=1):
            if not image_path or not os.path.isfile(image_path):
                self.node.get_logger().error(f"[DIFY] 反馈图片不存在: {image_path}")
                return None

            try:
                with open(image_path, "rb") as image_file:
                    file_response = self.client.file_upload(
                        "llm_agent",
                        files={
                            "file": (
                                f"robot_feedback_{image_index}.jpg",
                                image_file,
                                "image/jpeg",
                            )
                        },
                    )
            except Exception as error:
                self.node.get_logger().error(
                    f"[DIFY] 反馈图片上传异常: {type(error).__name__}: {error}"
                )
                return None

            if not self.is_success_http_status(file_response.status_code):
                self.node.get_logger().error(
                    f"[DIFY] 反馈图片上传失败: {self.format_dify_error(file_response)}"
                )
                return None

            upload_result = file_response.json()
            upload_file_id = upload_result.get("id")
            if not upload_file_id:
                self.node.get_logger().error(
                    f"[DIFY] 上传响应缺少文件ID: {upload_result}"
                )
                return None

            image_file_objects.append(
                self.build_local_image_file_object(upload_file_id)
            )
            self.node.get_logger().info(
                f"[DIFY] 反馈图片 {image_index}/{len(image_paths)} 上传完成: "
                f"file_id={upload_file_id}, HTTP {file_response.status_code}"
            )

        return image_file_objects

    def parse_response(self, dify_response: dict) -> tuple:
        """Parse the Dify response into (actions list, response text)."""
        answer = dify_response.get("answer", "")
        if not isinstance(answer, str) or not answer.strip():
            conversation_id = dify_response.get("conversation_id", "")
            message_id = dify_response.get("message_id", "")
            metadata = dify_response.get("metadata", {})
            self.node.get_logger().error(
                "[DIFY] streaming 完成但 answer 为空，"
                f"conversation_id={conversation_id}, "
                f"message_id={message_id}, metadata={metadata}"
            )
            return [], "Dify返回空响应，无法继续解析动作；请检查Dify工作流运行日志"

        try:
            # 格式: {JSON}&&&&{可选说明}
            if "&&&&" in answer:
                parts = answer.split("&&&&", maxsplit=1)
                json_part = parts[0]
                decision_part = parts[1] if len(parts) > 1 else ""
                if self.node.debug_mode:
                    self.node.get_logger().info(
                        f"[PARSE] 检测到'&&&&'分隔符, 决策说明: {decision_part[:100]}"
                    )
            else:
                json_part = answer

            extracted = extract_json_content(json_part)
            if self.node.debug_mode:
                self.node.get_logger().info(f"[PARSE] JSON提取结果: {extracted}")

            if extracted is None:
                self.node.get_logger().error(
                    f"[PARSE] 无法从Dify响应中提取JSON, 原始内容: {json_part[:200]}"
                )
                return [], answer

            actions = extracted.get("action", [])
            response = extracted.get("response", "")

            if not isinstance(actions, list):
                self.node.get_logger().warn(
                    f"[PARSE] action字段不是列表类型: {type(actions)}, 值: {actions}"
                )
                actions = []
            else:
                invalid_actions = [
                    action
                    for action in actions
                    if not isinstance(action, str) or not action.strip()
                ]
                if invalid_actions:
                    self.node.get_logger().warn(
                        f"[PARSE] 已忽略无效动作: {invalid_actions}"
                    )
                actions = [
                    action.strip()
                    for action in actions
                    if isinstance(action, str) and action.strip()
                ]

            if not isinstance(response, str):
                self.node.get_logger().warn(
                    f"[PARSE] response字段不是字符串: {type(response)}"
                )
                response = str(response or "")

            if self.node.debug_mode:
                self.node.get_logger().info(
                    f"[PARSE] 最终解析: {len(actions)}个动作, "
                    f"response='{response[:60]}'"
                )
            return actions, response
        except Exception as error:
            self.node.get_logger().error(
                f"[PARSE] 解析异常: {type(error).__name__}: {error}, "
                f"answer={answer[:200]}"
            )
            return [], answer

    # ---- helpers -------------------------------------------------------

    @staticmethod
    def build_local_image_file_object(upload_file_id: str) -> dict:
        """Build a Dify local image file object for the files field."""
        return {
            "type": "image",
            "transfer_method": "local_file",
            "upload_file_id": upload_file_id,
        }

    @staticmethod
    def is_success_http_status(http_status_code: int) -> bool:
        """Check whether an HTTP status code is a success (2xx)."""
        return 200 <= int(http_status_code) < 300

    @staticmethod
    def format_dify_error(response) -> str:
        """Extract a structured error message from a Dify HTTP response."""
        try:
            err_data = response.json()
            err_code = err_data.get("code", "unknown")
            err_msg = err_data.get("message", "")
            return f"code={err_code}, message={err_msg}"
        except Exception:
            return f"status={response.status_code}, body={response.text[:200]}"

