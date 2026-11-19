#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audio_input_node.py
电脑端语音识别节点：录音 → whisper.cpp 离线识别 → 发布到 /llm_text_input
配合 llm_agent_node 使用，可替代 terminal_client 手动输入。

两种触发模式：
  1) 按回车开始录音，再次回车停止（默认，适合测试）
  2) 发布 /speech_trigger (std_msgs/Empty) 触发，发布 /speech_stop 停止（适合后续接入按钮/VAD）

依赖:
  - 系统: arecord (ALSA), whisper.cpp 的 whisper-cli
  - Python: rclpy (ROS 2)
  - 消息: interfaces.msg.LlmTextRequest

用法:
  source /opt/ros/jazzy/setup.bash
  source <项目>/install/setup.bash
  export ROS_DOMAIN_ID=40
  python3 audio_input_node.py \
      --ros-args \
      -p whisper_bin:=/home/asang/tools/whisper.cpp/build/bin/whisper-cli \
      -p model_path:=/home/asang/tools/whisper.cpp/models/ggml-small.bin \
      -p record_seconds:=5.0
"""

import os
import re
import sys
import signal
import subprocess
import tempfile
import threading
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty
from interfaces.msg import LlmTextRequest


class AudioInputNode(Node):
    def __init__(self):
        super().__init__('audio_input_node')

        # ---- 可配置参数 ----
        self.declare_parameter('whisper_bin', '/home/asang/tools/whisper.cpp/build/bin/whisper-cli')
        self.declare_parameter('model_path', '/home/asang/tools/whisper.cpp/models/ggml-small.bin')
        self.declare_parameter('language', 'zh')              # 中文识别
        self.declare_parameter('record_seconds', 5.0)        # 单次录音时长
        self.declare_parameter('record_device', 'default')   # ALSA 设备 default / hw:1,0
        self.declare_parameter('llm_topic', '/llm_text_input')
        self.declare_parameter('threads', 4)
        self.declare_parameter('input_mode', 'manual')       # manual | topic

        self.whisper_bin = self.get_parameter('whisper_bin').value
        self.model_path = self.get_parameter('model_path').value
        self.language = self.get_parameter('language').value
        self.record_seconds = float(self.get_parameter('record_seconds').value)
        self.record_device = self.get_parameter('record_device').value
        self.llm_topic = self.get_parameter('llm_topic').value
        self.threads = int(self.get_parameter('threads').value)
        self.input_mode = self.get_parameter('input_mode').value

        # ---- 验证 whisper CLI ----
        if not Path(self.whisper_bin).is_file():
            self.get_logger().error(f'whisper-cli 不存在: {self.whisper_bin}')
        if not Path(self.model_path).is_file():
            self.get_logger().warn(f'whisper 模型文件不存在: {self.model_path}')
            self.get_logger().warn('首次使用需下载模型，例如: curl -L -o ggml-small.bin '
                                   'https://hf-mirror.com/ggerganov/whisper.cpp/resolve/main/ggml-small.bin')

        # ---- 发布者 ----
        self.text_pub = self.create_publisher(LlmTextRequest, self.llm_topic, 10)

        # ---- 话题触发（模式: topic） ----
        if self.input_mode == 'topic':
            self.start_sub = self.create_subscription(
                Empty, '/speech_trigger', self._on_trigger, 10)
            self.stop_sub = self.create_subscription(
                Empty, '/speech_stop', self._on_stop, 10)

        self._recording = False
        self._record_proc = None

        self.get_logger().info(
            f'audio_input_node 已启动 | 录音 {self.record_seconds}s | '
            f'whisper={self.whisper_bin} | model={self.model_path}'
        )

    # ------------------------------------------------------------------
    # 话题触发模式
    # ------------------------------------------------------------------
    def _on_trigger(self, _msg):
        if self._recording:
            self.get_logger().info('正在录音中，忽略重复触发')
            return
        self._record_and_publish_async()

    def _on_stop(self, _msg):
        if self._recording and self._record_proc and self._record_proc.poll() is None:
            self._record_proc.terminate()

    # ------------------------------------------------------------------
    # 核心: 录音 + 识别 + 发布
    # ------------------------------------------------------------------
    def _record_and_publish_async(self):
        """在独立线程中执行，避免阻塞 spin。"""
        threading.Thread(target=self._record_and_publish, daemon=True).start()

    def _record_and_publish(self):
        tmp_dir = Path(tempfile.mkdtemp(prefix='speech_'))
        wav_path = str(tmp_dir / 'input.wav')
        self._recording = True
        try:
            self.get_logger().info(f'开始录音 {self.record_seconds:.1f}s ...')
            ok = self._record_wav(wav_path)
            if not ok:
                self.get_logger().error('录音失败，跳过本次识别')
                return
            self.get_logger().info('录音结束，正在识别 ...')
            text = self._transcribe(wav_path)
            text = (text or '').strip()
            if not text:
                self.get_logger().warn('识别结果为空，请再说一次')
                return
            self.get_logger().info(f'识别结果: {text}')
            msg = LlmTextRequest()
            msg.text = text
            msg.is_feedback = False
            self.text_pub.publish(msg)
            self.get_logger().info(f'已发布到 {self.llm_topic}')
        except Exception as e:
            self.get_logger().error(f'语音处理异常: {e}')
        finally:
            self._recording = False
            try:
                for f in tmp_dir.iterdir():
                    f.unlink()
                tmp_dir.rmdir()
            except Exception:
                pass

    def _record_wav(self, wav_path: str) -> bool:
        """用 arecord 录 16kHz 单声道 16bit WAV。"""
        dev_args = ['-D', self.record_device] if self.record_device != 'default' else []
        cmd = ['arecord', *dev_args,
               '-d', str(self.record_seconds),
               '-r', '16000',
               '-c', '1',
               '-f', 'S16_LE',
               '-t', 'wav',
               wav_path]
        self._record_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        try:
            _, err = self._record_proc.communicate(timeout=self.record_seconds + 3)
        except subprocess.TimeoutExpired:
            self._record_proc.kill()
            _, err = self._record_proc.communicate()
        rc = self._record_proc.returncode
        self._record_proc = None
        if rc != 0 or not os.path.isfile(wav_path):
            self.get_logger().error(f'arecord 失败 (rc={rc}): {err.decode(errors="replace")[:200]}')
            return False
        return True

    def _transcribe(self, wav_path: str) -> str:
        """调用 whisper-cli 并从输出中抽取纯文本。"""
        if not Path(self.whisper_bin).is_file():
            self.get_logger().error(f'whisper-cli 不存在: {self.whisper_bin}')
            return ''
        if not Path(self.model_path).is_file():
            self.get_logger().error(f'model 不存在: {self.model_path}')
            return ''
        cmd = [self.whisper_bin,
               '-m', self.model_path,
               '-f', wav_path,
               '-l', self.language,
               '-t', str(self.threads),
               '-otxt', 'false',          # 不输出到 txt 文件
               '-np',                     # 不打印进度条
               '-ng',                     # 不输出产生的 token
               '-pc',                     # 打印性能统计（方便日志里有耗时）
               '-ps']                     # 打印 segments
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            self.get_logger().error('whisper 识别超时')
            return ''
        if proc.returncode != 0:
            self.get_logger().error(f'whisper 失败: {proc.stderr[:300]}')
            return ''
        return self._extract_text(proc.stdout)

    @staticmethod
    def _extract_text(raw: str) -> str:
        """
        whisper-cli -ps 输出形如:
          [00:00:00.000 --> 00:00:03.500]  你好，帮我找红色水杯
          [00:00:03.500 --> 00:00:05.000]  好的，现在开始执行
        每行方括号后是识别文本，逐行拼接即可。
        """
        lines = []
        for line in raw.splitlines():
            m = re.match(r'^\s*\[\d{2}:\d{2}:\d{2}\.\d{3}\s*-->\s*\d{2}:\d{2}:\d{2}\.\d{3}\]\s*(.*)$', line)
            if m:
                lines.append(m.group(1).strip())
        if not lines:
            # 退化: 去掉方括号、空行、性能统计
            cleaned = re.sub(r'\[.*?\]', '', raw)
            cleaned = re.sub(r'\(.*?\)', '', cleaned)
            cleaned = re.sub(r'\s+', ' ', cleaned).strip()
            return cleaned
        return ' '.join(lines)


# ---------------------------------------------------------------------------
# 手动交互入口（按回车录音）
# ---------------------------------------------------------------------------
def manual_loop(node: AudioInputNode):
    print('\n=== audio_input_node 手动模式 ===')
    print('按 <回车> 开始录音并识别（Ctrl+C 退出）\n')
    try:
        while rclpy.ok():
            try:
                input('> 按回车说话 ... ')
            except EOFError:
                break
            node._record_and_publish()
    except KeyboardInterrupt:
        print('\n退出 ...')


def main(args=None):
    rclpy.init(args=args)
    node = AudioInputNode()

    # SIGINT 时让 rclpy.shutdown 先执行，这样 manual_loop 里的 rclpy.ok() 能正确返回
    stop = threading.Event()

    def handle_signal(signum, frame):
        stop.set()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # 先启动 spin 线程（非 daemon，确保 shutdown 被调用）
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=False)
    spin_thread.start()

    try:
        if node.input_mode == 'manual':
            manual_loop(node)
        else:
            print('[audio_input_node] topic 模式已就绪，等待 /speech_trigger 触发 ...')
            # topic 模式下直接阻塞等待事件
            while not stop.is_set() and rclpy.ok():
                stop.wait(0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
