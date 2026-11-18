#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
terminal_client.py
终端交互客户端 - 用于直接输入文本控制机器人
Terminal Interactive Client - For direct text input robot control
"""

import os
import readline  # noqa: F401 - enables interactive line editing
import threading

import rclpy
from rclpy.node import Node

from interfaces.msg import LlmTextRequest, LlmTextResponse


class TerminalClient(Node):
    """终端交互客户端节点"""

    def __init__(self):
        super().__init__("terminal_client")

        # 发布者：发送用户输入
        self.text_pub = self.create_publisher(LlmTextRequest, "/llm_text_input", 10)

        # 订阅者：接收机器人响应
        self.text_sub = self.create_subscription(
            LlmTextResponse, "/llm_text_output", self.response_callback, 10
        )

        self.latest_response = None
        self.print_lock = threading.Lock()

        self.get_logger().info("Terminal client initialized")

    def response_callback(self, msg):
        """异步显示机器人响应，不阻塞用户继续输入"""
        self.latest_response = msg
        msg_type = getattr(msg, "msg_type", "") or ""

        with self.print_lock:
            if msg_type == "progress":
                # 进度消息：简洁格式
                print(f"\n>   [进度] {msg.response_text}")
            else:
                # 最终结果：完整格式
                print("\n> [机器人回复/Robot Response]")
                print("> " + "-" * 60)
                print(f"> {msg.response_text}")
                print("> " + "-" * 60)

                if msg.action_list_json:
                    print("> [执行动作/Executed Actions]")
                    print("> " + "-" * 60)
                    print(f"> {msg.action_list_json}")
                    print("> " + "-" * 60)

            # 重新显示提示符，让用户知道可以继续输入
            print("> ", end="", flush=True)

    def send_text(self, text: str):
        """发送文本到机器人"""
        msg = LlmTextRequest()
        msg.text = text
        msg.is_feedback = False
        self.text_pub.publish(msg)


def print_banner():
    """打印欢迎横幅"""
    print("\n> " + "=" * 60)
    print(">   MicroROS LLM Agent 终端客户端")
    print(">   Terminal Client for LLM Agent Control")
    print("> " + "=" * 60)
    print("> 输入自然语言指令与机器人交互")
    print("> Enter natural language commands to control the robot")
    print("> ")
    print("> 示例 / Examples:")
    print(">   > 去A点              (Go to point A)")
    print(">   > 沿着红线走          (Follow the red line)")
    print(">   > 看看前面有什么      (Look at what's ahead)")
    print(">   > 前进2米             (Move forward 2 meters)")
    print("> ")
    print("> 命令 / Commands:")
    print(">   reset  - 重置对话上下文 (Reset conversation)")
    print(">   clear  - 清屏 (Clear screen)")
    print(">   help   - 显示帮助 (Show help)")
    print(">   exit   - 退出程序 (Exit)")
    print("> " + "=" * 60 + "\n")


def print_help():
    """打印帮助信息"""
    print("\n> 可用命令 / Available Commands:")
    print("> " + "-" * 60)
    print(">   <任意文本>  发送指令给机器人")
    print(">               Send command to robot")
    print("> ")
    print(">   reset       重置对话上下文")
    print(">               Reset conversation context")
    print("> ")
    print(">   clear       清屏")
    print(">               Clear screen")
    print("> ")
    print(">   help        显示此帮助")
    print(">               Show this help")
    print("> ")
    print(">   exit/quit   退出程序")
    print(">               Exit program")
    print("> " + "-" * 60)
    print("> ")
    print("> 示例 / Examples:")
    print(">   > 去A点")
    print(">   > 沿着红线走")
    print(">   > 看看前面有什么")
    print(">   > 前进2米然后左转90度")
    print(">   > 获取当前位置")
    print("> ")


def main():
    """主函数"""
    rclpy.init()
    client = TerminalClient()

    # 启动spin线程（非daemon，确保清理时能正常join）
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=False)
    spin_thread.start()

    # 打印欢迎横幅
    print_banner()

    try:
        while rclpy.ok():
            try:
                # 读取用户输入
                user_input = input("\n> ").strip()

                if not user_input:
                    continue

                # 处理特殊命令
                cmd = user_input.lower()

                if cmd in ["exit", "quit", "q"]:
                    print("\n退出程序... / Exiting...")
                    break

                elif cmd == "reset":
                    client.send_text("__reset_conversation__")
                    print("> [INFO] 已请求重置对话 / Reset requested")
                    continue

                elif cmd == "help":
                    print_help()
                    continue

                elif cmd == "clear":
                    os.system("clear" if os.name != "nt" else "cls")
                    print_banner()
                    continue

                # 发送文本到机器人
                print(f">\n> [发送/Sending] {user_input}")
                client.send_text(user_input)
                print("> [已发送/Sent] 可继续输入新指令，最新指令会优先生效")

            except KeyboardInterrupt:
                print("\n> 退出程序... / Exiting...")
                break
            except EOFError:
                print("> 退出程序... / Exiting...")
                break
            except Exception as error:
                print(f"> [错误/Error] {error}")

    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        client.destroy_node()
        print("> 程序已退出 / Program exited")


if __name__ == "__main__":
    main()
