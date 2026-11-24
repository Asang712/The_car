"""
PPT 生成脚本：micro-ROS V2 小车项目分享
主题：我用 7 天把一台 micro-ROS 小车跑通——ROS 2 新人的 6 个认知转变
Palette: Ocean Gradient + Orange accent
"""

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
import os

# ===== Constants =====
SLIDE_W = Inches(13.333)
SLIDE_H = Inches(7.5)

# Ocean Gradient palette
BG_DARK = RGBColor(0x1F, 0x29, 0x37)    # dark gray cover bg
BG_DEEP = RGBColor(0x06, 0x5A, 0x82)    # deep blue primary
BG_TEAL = RGBColor(0x1C, 0x72, 0x93)    # teal secondary
ACCENT = RGBColor(0xFF, 0x5A, 0x1F)     # orange for pitfalls
ACCENT_SOFT = RGBColor(0xFF, 0x8C, 0x42)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
LIGHT_GRAY = RGBColor(0x9C, 0xA3, 0xAF)
CARD_BG = RGBColor(0xF3, 0xF4, 0xF6)
WAS_RED = RGBColor(0xFE, 0xF2, 0xE2)
NOW_BLUE = RGBColor(0xE0, 0xF2, 0xFE)

# Fonts
FONT_TITLE = 'Noto Serif CJK SC'
FONT_BODY = 'Noto Sans CJK SC'
FONT_MONO = 'JetBrains Mono'

OUT = 'ros2-car-postmortem/ROS2认知转变分享.pptx'
os.makedirs('ros2-car-postmortem', exist_ok=True)

prs = Presentation()
prs.slide_width = SLIDE_W
prs.slide_height = SLIDE_H

blank = prs.slide_layouts[6]  # blank

# ===== Helpers =====
def add_bg(slide, color):
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, SLIDE_W, SLIDE_H)
    shape.fill.solid()
    shape.fill.fore_color.rgb = color
    shape.line.fill.background()
    shape.shadow.inherit = False
    return shape

def add_text(slide, text, left, top, width, height, font_size=18, color=RGBColor(0x1A,0x1A,0x1C),
             bold=False, font=FONT_BODY, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(left, top, width, height)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = Emu(0)
    tf.margin_right = Emu(0)
    tf.margin_top = Emu(0)
    tf.margin_bottom = Emu(0)
    tf.vertical_anchor = anchor
    p = tf.paragraphs[0]
    p.alignment = align
    run = p.add_run()
    run.text = text
    run.font.size = Pt(font_size)
    run.font.color.rgb = color
    run.font.bold = bold
    run.font.name = font
    return tb

def add_para(text_frame, text, font_size=18, color=RGBColor(0x1A,0x1A,0x1C),
             bold=False, font=FONT_BODY, align=PP_ALIGN.LEFT, space_before=Pt(6),
             space_after=Pt(6)):
    p = text_frame.add_paragraph()
    p.alignment = align
    p.space_before = space_before
    p.space_after = space_after
    run = p.add_run()
    run.text = text
    run.font.size = Pt(font_size)
    run.font.color.rgb = color
    run.font.bold = bold
    run.font.name = font
    return p

def add_card(slide, left, top, width, height, bg_color=RGBColor(0xF3,0xF4,0xF6),
             border_color=None, border_width=Pt(0)):
    shape = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
    shape.fill.solid()
    shape.fill.fore_color.rgb = bg_color
    if border_color:
        shape.line.color.rgb = border_color
        shape.line.width = border_width
    else:
        shape.line.fill.background()
    shape.shadow.inherit = False
    return shape

def add_accent_bar(slide, left, top, height=Pt(36), color=ACCENT):
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, left, top, Inches(0.12), height)
    shape.fill.solid()
    shape.fill.fore_color.rgb = color
    shape.line.fill.background()
    return shape

# ===== SLIDE 1: 封面 =====
s = prs.slides.add_slide(blank)
add_bg(s, BG_DARK)
# Accent bars decoration
bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(0), SLIDE_W, Pt(6))
bar.fill.solid(); bar.fill.fore_color.rgb = ACCENT; bar.line.fill.background()

add_text(s, '给同学的 ROS 2 分享 · 2026', Inches(1.0), Inches(1.2), Inches(11), Inches(0.5),
         font_size=13, color=LIGHT_GRAY, font=FONT_BODY, bold=False)

add_text(s, '我用 7 天把一台 micro-ROS 小车跑通', Inches(1.0), Inches(2.0), Inches(11), Inches(1.2),
         font_size=40, color=WHITE, font=FONT_TITLE, bold=True)

add_text(s, 'ROS 2 新人的 6 个认知转变', Inches(1.0), Inches(3.2), Inches(11), Inches(0.8),
         font_size=28, color=ACCENT_SOFT, font=FONT_TITLE, bold=True)

add_text(s, '厂家说"开箱即用"，我用了 7 天。这不是一份项目报告，是我真正学会的事。',
         Inches(1.0), Inches(4.3), Inches(11), Inches(0.6),
         font_size=16, color=LIGHT_GRAY, font=FONT_BODY)

add_text(s, '📅 2026-11-17 → 2026-11-24   🛠️ Ubuntu 24.04 + ROS 2 Jazzy',
         Inches(1.0), Inches(6.3), Inches(11), Inches(0.5),
         font_size=13, color=LIGHT_GRAY, font=FONT_BODY)

# Stat cards row
stat_y = Inches(5.15)
for i, (num, label) in enumerate([('7', '天'), ('15', '功能包'), ('11', '坑'), ('6', '认知转变')]):
    cx = Inches(1.0 + i * 3.0)
    card = add_card(s, cx, stat_y, Inches(2.7), Inches(0.9), bg_color=RGBColor(0x2B,0x37,0x48))
    add_text(s, num, cx + Inches(0.3), stat_y + Inches(0.05), Inches(1.2), Inches(0.6),
             font_size=28, color=ACCENT_SOFT, font=FONT_TITLE, bold=True)
    add_text(s, label, cx + Inches(0.3), stat_y + Inches(0.55), Inches(2.0), Inches(0.3),
             font_size=12, color=LIGHT_GRAY, font=FONT_BODY)

# ===== SLIDE 2: 目录 =====
s = prs.slides.add_slide(blank)
add_bg(s, WHITE)
add_accent_bar(s, Inches(1.0), Inches(0.8), Pt(32), ACCENT)
add_text(s, '目录', Inches(1.3), Inches(0.75), Inches(10), Inches(0.5),
         font_size=32, color=RGBColor(0x1A,0x1A,0x1C), font=FONT_TITLE, bold=True)
add_text(s, '6 个认知转变 + 1 份 checklist', Inches(1.3), Inches(1.25), Inches(10), Inches(0.4),
         font_size=14, color=LIGHT_GRAY)

toc_items = [
    ('01', 'QoS 是看不见的墙',  'ROS 2 话题"通了"≠"数据能传"'),
    ('02', '厂家给的是源码不是产品', '那些写死的路径和隐式依赖'),
    ('03', '硬件约束先排查别先调代码', '我浪费 80% 调试时间的原因'),
    ('04', 'ROS 2 版本升级的隐形地雷', '-Werror 会把小警告炸成编译失败'),
    ('05', '不建图也能做机器人', '视觉伺服路线选择'),
    ('06', '自动化脚本写死脏活', '200 行 shell 省好几天返工'),
]
for i, (num, title, sub) in enumerate(toc_items):
    row_y = Inches(1.8 + i * 0.85)
    # num badge
    badge = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE,
                               Inches(1.3), row_y, Inches(0.7), Inches(0.65))
    badge.fill.solid(); badge.fill.fore_color.rgb = BG_DEEP
    badge.line.fill.background()
    add_text(s, num, Inches(1.3), row_y + Inches(0.07), Inches(0.7), Inches(0.5),
             font_size=18, color=WHITE, font=FONT_MONO, bold=True, align=PP_ALIGN.CENTER)
    add_text(s, title, Inches(2.2), row_y + Inches(0.05), Inches(8), Inches(0.4),
             font_size=18, bold=True, font=FONT_TITLE, color=RGBColor(0x1A,0x1A,0x1C))
    add_text(s, sub, Inches(2.2), row_y + Inches(0.35), Inches(8), Inches(0.3),
             font_size=13, color=LIGHT_GRAY)

# ===== SLIDE 3: 时间线快闪 =====
s = prs.slides.add_slide(blank)
add_bg(s, WHITE)
add_accent_bar(s, Inches(0.8), Inches(0.7), Pt(32), BG_DEEP)
add_text(s, '时间线快闪', Inches(1.1), Inches(0.65), Inches(11), Inches(0.5),
         font_size=30, color=RGBColor(0x1A,0x1A,0x1C), font=FONT_TITLE, bold=True)
add_text(s, 'Day 1 → Day 7 都干了啥', Inches(1.1), Inches(1.15), Inches(10), Inches(0.35),
         font_size=13, color=LIGHT_GRAY)

timeline = [
    ('Day 1', '解压源码\n编译 9/15 通过，roadnet 炸了', False),
    ('Day 2-3', 'Agent↔控制板链路打通\n相机 HTTP MJPEG 流接入', False),
    ('Day 4-5', '话题有发布者有订阅者但没数据\n卡了整整两天 → QoS 坑', True),
    ('Day 6', 'App 视频通了，编译全过\n全量 15/15 通过', False),
    ('Day 7', '决定放弃建图路线\n写自动化脚本、补文档', False),
]
# timeline connector
line = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(1.8), Inches(2.6), Inches(10.0), Pt(4))
line.fill.solid(); line.fill.fore_color.rgb = BG_TEAL; line.line.fill.background()

for i, (day, desc, highlight) in enumerate(timeline):
    dot_x = Inches(1.8 + i * 2.0)
    dot = s.shapes.add_shape(MSO_SHAPE.OVAL, dot_x - Inches(0.18), Inches(2.45),
                              Inches(0.36), Inches(0.36))
    dot.fill.solid()
    dot.fill.fore_color.rgb = ACCENT if highlight else BG_TEAL
    dot.line.color.rgb = WHITE
    dot.line.width = Pt(2)
    add_text(s, day, dot_x - Inches(0.5), Inches(2.95), Inches(1.0), Inches(0.4),
             font_size=12, bold=True, color=ACCENT if highlight else BG_DEEP, align=PP_ALIGN.CENTER)
    tb = add_text(s, desc, dot_x - Inches(0.9), Inches(3.35), Inches(1.8), Inches(2.0),
                  font_size=11, color=RGBColor(0x1A,0x1A,0x1C), align=PP_ALIGN.CENTER)
    for p in tb.text_frame.paragraphs:
        p.space_before = Pt(1)
        p.space_after = Pt(1)

# ===== SLIDE 4: 认知转变 1 — QoS =====
def insight_slide(prs, num, title, was, now, story, pitfall_code=None):
    s = prs.slides.add_slide(blank)
    add_bg(s, WHITE)
    add_accent_bar(s, Inches(0.8), Inches(0.6), Pt(36), BG_DEEP)
    # num badge
    badge = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(1.1), Inches(0.55),
                                Inches(1.1), Inches(0.5))
    badge.fill.solid(); badge.fill.fore_color.rgb = ACCENT; badge.line.fill.background()
    add_text(s, num, Inches(1.1), Inches(0.6), Inches(1.1), Inches(0.42),
             font_size=16, color=WHITE, bold=True, font=FONT_MONO, align=PP_ALIGN.CENTER)
    add_text(s, title, Inches(2.4), Inches(0.55), Inches(10), Inches(0.7),
             font_size=28, color=RGBColor(0x1A,0x1A,0x1C), bold=True, font=FONT_TITLE)

    # Was vs Now boxes
    was_box = add_card(s, Inches(0.8), Inches(1.6), Inches(5.6), Inches(1.6), bg_color=WAS_RED)
    add_text(s, '我原来以为', Inches(1.0), Inches(1.65), Inches(5.2), Inches(0.35),
             font_size=11, bold=True, color=ACCENT)
    add_text(s, was, Inches(1.0), Inches(2.0), Inches(5.2), Inches(1.1),
             font_size=14, color=RGBColor(0x1A,0x1A,0x1C))

    now_box = add_card(s, Inches(6.8), Inches(1.6), Inches(5.7), Inches(1.6), bg_color=NOW_BLUE,
                       border_color=BG_DEEP, border_width=Pt(1))
    add_text(s, '我后来明白', Inches(7.0), Inches(1.65), Inches(5.3), Inches(0.35),
             font_size=11, bold=True, color=BG_DEEP)
    add_text(s, now, Inches(7.0), Inches(2.0), Inches(5.3), Inches(1.1),
             font_size=14, color=RGBColor(0x1A,0x1A,0x1C))

    # Story
    add_text(s, '具体经历', Inches(0.8), Inches(3.4), Inches(12), Inches(0.4),
             font_size=13, bold=True, color=LIGHT_GRAY)
    add_text(s, story, Inches(0.8), Inches(3.8), Inches(11.7), Inches(1.6),
             font_size=14, color=RGBColor(0x1A,0x1A,0x1C))

    if pitfall_code:
        # Pitfall box
        add_accent_bar(s, Inches(0.8), Inches(5.6), Pt(32), ACCENT)
        add_text(s, '避坑指南', Inches(1.1), Inches(5.55), Inches(11), Inches(0.4),
                 font_size=13, bold=True, color=WHITE)
        add_text(s, pitfall_code, Inches(1.1), Inches(5.95), Inches(11), Inches(1.3),
                 font_size=12, color=WHITE)
    return s

insight_slide(prs, '#01', 'QoS 是看不见的墙',
    '发布者订阅者都在 = 话题就通了，ros2 topic echo 就能看到数据。',
    'QoS（Quality of Service）匹配规则是 ROS 2 里最隐形的兼容性杀手。发布端 RELIABLE 订阅端 BEST_EFFORT，或者 history/depth 不一样，话题就会出现"有发布者有订阅者但没数据"的幽灵状态。',
    '相机视频话题就是这么卡的。camera_driver.py 默认用传感器 QoS（RELIABLE），transmission.py 订阅端没显式声明，App 视频流黑屏了整整两天。敲了 ros2 topic info -v /camera/image_raw 才发现两端 QoS 不一致。统一改成 SensorDataQoS() 后第一帧画面就出来了。',
    '✅ 任何话题先出问题 → ros2 topic info -v /话题名 查 QoS\n✅ 发布端订阅端 QoS 必须完全一致（RELIABILITY + history + depth）\n✅ 相机/雷达这类高频传感器，默认用 BEST_EFFORT')

# ===== SLIDE 5: 认知转变 2 — 厂家源码 =====
insight_slide(prs, '#02', '厂家给的是源码，不是产品',
    'colcon build 过了 = 所有东西都就位了，直接 ros2 launch。',
    'launch 文件背后藏着厂家写死的路径、隐式的配置文件位置、默认参数。换台电脑或者换目录，这些"看不见的依赖"就会浮到水面上。',
    'bringup_app.launch.py 里调用 GetCameraUrl()，它在 robot_tool.py 里写死读 ~/microros_ws/.system_cfg/robot_info.json——不是 launch file 同目录，而是 home 目录下另一个位置。新电脑上直接 launch，第一秒就抛 FileNotFoundError。\n\ncamera_driver.py 的标定文件同理：ost.yaml 厂家明明在包里，但 ost_file 参数默认空。launch 里有传参没事，我手动启动就忘了。',
    '✅ 编译通过后先 grep -r "~/" src/launch/ | head -20\n✅ 扫 setup.py 的 entry_points，确认想用的节点都注册了\n✅ 把 launch 和 config 里的默认参数记下来（标定文件在哪、QoS 是什么）')

# ===== SLIDE 6: 认知转变 3 — 硬件约束（重点）=====
insight_slide(prs, '#03', '硬件约束先排查，别先调代码',
    '"车不动"大概率是我 /cmd_vel 指令写错了，先查代码。',
    '电源模式、USB 模式、Wi-Fi 连接、Agent 进程——这些比任何算法都更"底层"。软件调半天，可能只是电池没电。',
    '我在"代码没错但车不动"上浪费的时间里，80% 最后发现是硬件状态问题。下面这 5 件事，软件调试之前先过一遍。',
    '✅ USB 没插控制板 → 插了就切手柄模式完全忽略 /cmd_vel\n✅ 电池 ≥ 6.50V → 低于这个阈值 ESP32 直接挂起所有模块\n✅ Agent 终端在跑 → 关了 = 断链\n✅ ROS_DOMAIN_ID 两端一致 → 不一致节点互相发现失败\n◉ 热点刚重启过 → 重启了重新扫描所有 IP')

# ===== SLIDE 7: 硬件五件套详情 =====
s = prs.slides.add_slide(blank)
add_bg(s, WHITE)
add_accent_bar(s, Inches(0.8), Inches(0.6), Pt(36), ACCENT)
add_text(s, '硬件排查五件套', Inches(1.1), Inches(0.55), Inches(11), Inches(0.6),
         font_size=28, color=RGBColor(0x1A,0x1A,0x1C), bold=True, font=FONT_TITLE)
add_text(s, '软件调试之前必过，能省至少 2 小时', Inches(1.1), Inches(1.1), Inches(10), Inches(0.35),
         font_size=13, color=LIGHT_GRAY)

hw_items = [
    ('P05', 'USB 插控制板 → 手柄模式', '插 USB 时固件自动切 USB HID 手柄控制，完全忽略 /cmd_vel。拔掉 USB 才恢复话题控制。', ACCENT),
    ('P06', '电池 < 6.50V → 挂起所有模块', 'ESP32 上电先查电压，低于阈值直接挂 Wi-Fi、相机、串口。看似软件没连上，实则硬件自我保护。', ACCENT),
    ('P07', '手机热点 DHCP → IP 全变', '电脑/控制板/相机每次热点重启都要重新扫 IP，再更新 robot_info.json 的 camera_ip。', ACCENT),
    ('P08', 'Agent 是前台进程', '关终端 = Agent 终止 = topic list 全空。放 tmux 或 nohup 里跑。', ACCENT),
    ('P09', 'Domain ID 必须两端一致', 'Agent 和控制板用不同"频道"，ROS 2 正常但节点互相发现失败，话题列表为空。', ACCENT),
]
for i, (code, title, desc, color) in enumerate(hw_items):
    row_y = Inches(1.6 + i * 1.12)
    card = add_card(s, Inches(0.8), row_y, Inches(11.7), Inches(0.98), bg_color=CARD_BG)
    add_accent_bar(s, Inches(0.8), row_y, Inches(0.98), color)
    add_text(s, code, Inches(1.1), row_y + Inches(0.05), Inches(0.8), Inches(0.35),
             font_size=11, bold=True, color=ACCENT, font=FONT_MONO)
    add_text(s, title, Inches(2.0), row_y + Inches(0.02), Inches(10), Inches(0.4),
             font_size=15, bold=True, font=FONT_TITLE, color=RGBColor(0x1A,0x1A,0x1C))
    add_text(s, desc, Inches(2.0), row_y + Inches(0.42), Inches(10.2), Inches(0.5),
             font_size=12, color=LIGHT_GRAY)

# ===== SLIDE 8: 认知转变 4 — 版本升级 =====
insight_slide(prs, '#04', 'ROS 2 版本升级的隐形地雷',
    '从 Humble 升到 Jazzy 是"改个环境变量、编一下"的事。',
    'ROS 2 每个版本都有弃用迁移指南，Jazzy 尤其多。-Werror 会把任何小警告炸成编译失败。',
    'roadnet 的 rmw_qos_profile_t 弃用警告是最明显的例子。nav2 的 nav2_package() 宏默认开启 -Wall -Wextra -Werror，厂家源码可能是 Humble 时代写的，在 Jazzy 上编不过不是厂家菜，是版本升级链路里的隐式断裂。\n\n我修复的代码只有两行，但理解"为什么这两行会炸整个编译"花了 40 分钟读 CMakeLists.txt 和 nav2 的宏定义。',
    '✅ 升级 ROS 2 时，先 grep 一遍 .get_rmw_qos_profile 和 rmw_qos_profile_t\n✅ 错误写法：create_client(..., qos.get_rmw_qos_profile())\n✅ 正确写法：create_client(..., qos)')

# ===== SLIDE 9: 认知转变 5+6 + 饼图合并 =====
# 认知转变 5 单独一张
insight_slide(prs, '#05', '不建图也能做机器人',
    'SLAM → Nav2 全局规划是"正规机器人"的标准路线。不建图的都是玩具。',
    '视觉伺服 + AprilTag 视觉路标这条路成本低、稳定、无需标定地图，更适合室内找东西、跟人这种场景。',
    '厂家默认路线 SLAM → Nav2 → roadnet 全写好了。但用户明确说"不想手动驾驶小车走一圈建图"——室内光线条件下 SLAM 效果本来就一般，Nav2 全局规划要建图、标定、TF 树对齐，一步出错全断。\n\n视觉伺服原理很简单：相机每帧检测目标 → 框在画面左边就右转、框大了就前进 → 框足够大足够居中就停。本质是用画面特征直接闭环控制，不用地图、不用全局定位。',
    '✅ 先问"我到底要解决什么问题"，再选工具\n✅ 不要默认选最复杂的方案\n✅ 视觉伺服 vs SLAM：你真的需要全局地图吗？')

# 认知转变 6
insight_slide(prs, '#06', '自动化脚本写死那些容易忘的"脏活"',
    '手动跑 Agent、手动 launch、手动 git push 就是正常工作流。',
    '那些每次手动做都容易忘、但又必须做的事，值得写一个几行 shell 脚本。',
    '两个脚本加起来不到 200 行：\n\nstart_nomap.sh：一条命令拉起 Agent + launch + odom_pose_bridge，Ctrl+C 全退。最值钱的功能不是"一键启动"，而是自动检查 robot_info.json 存在不存在、退出时清理残留进程。\n\nauto_push.sh：300 秒轮询自动 commit + push，flock 防重复实例。最值钱的功能是不用每次 commit 再 push，也不用怕"改了东西忘了提交"。\n\n写这些脚本的成本远低于"某次忘了推代码、某次退出没清理干净导致下次启动冲突"的返工成本。',
    '✅ 容易忘的操作 > 5 次就值得写脚本\n✅ 脚本最值钱的不是"一键启动"，是自动检查和自动清理\n✅ 用 flock 防止多个实例同时运行')

# ===== SLIDE: 饼图（时间分布）=====
s = prs.slides.add_slide(blank)
add_bg(s, WHITE)
add_accent_bar(s, Inches(0.8), Inches(0.6), Pt(36), BG_TEAL)
add_text(s, '我花时间最多在哪类问题上', Inches(1.1), Inches(0.55), Inches(11), Inches(0.6),
         font_size=28, color=RGBColor(0x1A,0x1A,0x1C), bold=True, font=FONT_TITLE)
add_text(s, '反直觉：一半的调试时间花在了"等硬件好"上', Inches(1.1), Inches(1.1), Inches(10), Inches(0.35),
         font_size=13, color=LIGHT_GRAY)

# Python 原生饼图（简单版，用 pptx 自带 chart）
from pptx.chart.data import CategoryChartData
from pptx.enum.chart import XL_CHART_TYPE, XL_LABEL_POSITION

chart_data = CategoryChartData()
chart_data.categories = ['硬件约束 (5)', '启动前置 (2)', 'ROS 2 Jazzy (1)', 'QoS 不匹配 (1)', '代码 bug (1)', '工具链 (1)']
chart_data.add_series('', [5, 2, 1, 1, 1, 1])

chart = s.shapes.add_chart(XL_CHART_TYPE.DOUGHNUT,
    Inches(1.0), Inches(1.7), Inches(5.5), Inches(4.5),
    chart_data).chart
chart.has_legend = False
chart.plots[0].has_data_labels = True
dl = chart.plots[0].data_labels
dl.position = XL_LABEL_POSITION.OUTSIDE_END
dl.font.size = Pt(12)
dl.font.name = FONT_BODY

# 右侧 insight cards
insights = [
    ('硬件约束占了近一半', '我以为软件坑会多，结果 5/11 个坑是硬件状态问题。USB 模式、电池电压、热点 DHCP——这些在软件调试之前先确认。'),
    ('启动前置条件是隐藏杀手', '两次"为啥 launch 就炸"的根因都是厂家写死的路径和默认参数。编译过了≠所有东西就位了。'),
    ('版本升级是一次性投资', 'roadnet 两行修复花了 40 分钟理解，但换来了整个 nav2 生态在 Jazzy 上能编过。'),
]
for i, (title, body) in enumerate(insights):
    y = Inches(1.7 + i * 1.5)
    card = add_card(s, Inches(7.0), y, Inches(5.5), Inches(1.35), bg_color=CARD_BG)
    add_accent_bar(s, Inches(7.0), y, Inches(1.35), BG_TEAL)
    add_text(s, title, Inches(7.3), y + Inches(0.1), Inches(5), Inches(0.35),
             font_size=13, bold=True, font=FONT_TITLE, color=BG_DEEP)
    add_text(s, body, Inches(7.3), y + Inches(0.45), Inches(5), Inches(0.85),
             font_size=12, color=RGBColor(0x1A,0x1A,0x1C))

# ===== SLIDE: 完整 checklist =====
s = prs.slides.add_slide(blank)
add_bg(s, WHITE)
add_accent_bar(s, Inches(0.8), Inches(0.6), Pt(36), BG_DEEP)
add_text(s, '给同学的完整 checklist', Inches(1.1), Inches(0.55), Inches(11), Inches(0.6),
         font_size=28, color=RGBColor(0x1A,0x1A,0x1C), bold=True, font=FONT_TITLE)
add_text(s, '我希望自己 7 天前就能看到的东西', Inches(1.1), Inches(1.1), Inches(10), Inches(0.35),
         font_size=13, color=LIGHT_GRAY)

checklists = [
    ('【编译前】', [
        ('◉', '确认硬件型号和 ROS 2 版本兼容性（厂家源码 Humble → 你是 Jazzy？）'),
        ('◉', 'grep .get_rmw_qos_profile → 这些在 Jazzy 上会炸'),
        ('✓', '先 colcon build --packages-up-to interfaces 编依赖，再全量'),
    ]),
    ('【编译通过后】', [
        ('◉', 'grep launch 文件里的 ~/ 硬编码路径，逐个确认存在'),
        ('✓', '扫 setup.py entry_points，确认想用的节点都注册了'),
        ('✓', '记录 launch/config 的默认参数（QoS、标定文件路径）'),
    ]),
    ('【启动前·硬件五件套】', [
        ('✓', 'USB 没插控制板 → 插了就切手柄模式'),
        ('✓', '电池 ≥ 6.50V → 低于阈值挂起所有模块'),
        ('✓', 'Agent 终端在跑 → 关了 = 断链'),
        ('✓', 'ROS_DOMAIN_ID 两端一致 → 不一致节点失联'),
        ('◉', '热点刚重启过 → 重新扫描所有 IP'),
    ]),
    ('【跑起来后】', [
        ('✓', '话题有问题先 ros2 topic info -v 查 QoS'),
        ('◉', '问自己：我真的需要全局地图吗？'),
        ('✓', '容易忘的事写进脚本'),
    ]),
]
for i, (section, items) in enumerate(checklists):
    col = i % 2
    row = i // 2
    cx = Inches(0.9 + col * 6.1)
    cy = Inches(1.6 + row * 2.85)
    card = add_card(s, cx, cy, Inches(5.9), Inches(2.65), bg_color=CARD_BG)
    add_text(s, section, cx + Inches(0.2), cy + Inches(0.15), Inches(5.5), Inches(0.4),
             font_size=13, bold=True, font=FONT_TITLE, color=BG_DEEP)
    for j, (mark, text) in enumerate(items):
        iy = cy + Inches(0.6 + j * 0.42)
        color = ACCENT if mark == '◉' else RGBColor(0x16,0xA3,0x4A)
        add_text(s, mark, cx + Inches(0.2), iy, Inches(0.3), Inches(0.35),
                 font_size=11, bold=True, color=color, align=PP_ALIGN.LEFT)
        add_text(s, text, cx + Inches(0.5), iy, Inches(5.3), Inches(0.35),
                 font_size=11.5, color=RGBColor(0x1A,0x1A,0x1C))

# ===== SLIDE: 仓库 + Q&A =====
s = prs.slides.add_slide(blank)
add_bg(s, BG_DARK)
# Top accent bar
bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(0), SLIDE_W, Pt(6))
bar.fill.solid(); bar.fill.fore_color.rgb = ACCENT; bar.line.fill.background()

add_text(s, '仓库 & Q&A', Inches(1.0), Inches(0.9), Inches(11), Inches(0.7),
         font_size=40, color=WHITE, bold=True, font=FONT_TITLE)
add_text(s, '所有代码、修复、脚本、文档都在', Inches(1.0), Inches(1.65), Inches(11), Inches(0.4),
         font_size=14, color=LIGHT_GRAY)

# Repo card
card = add_card(s, Inches(1.0), Inches(2.2), Inches(11.3), Inches(1.1),
                bg_color=RGBColor(0x2B,0x37,0x48))
add_text(s, 'github.com/Asang712/The_car', Inches(1.3), Inches(2.35), Inches(10.7), Inches(0.45),
         font_size=20, color=ACCENT_SOFT, bold=True, font=FONT_MONO)
add_text(s, 'auto_push.sh 会自动 commit + push，所有改动都在里面', Inches(1.3), Inches(2.85), Inches(10.7), Inches(0.35),
         font_size=12, color=LIGHT_GRAY)

# Scripts row
scripts = [
    ('start_nomap.sh', '一键启动不建图模式'),
    ('auto_push.sh', '自动 commit + push'),
    ('robot_info.json', '相机 IP 配置（热点重启后要更）'),
]
for i, (name, desc) in enumerate(scripts):
    sx = Inches(1.0 + i * 3.85)
    scard = add_card(s, sx, Inches(3.6), Inches(3.65), Inches(1.3),
                     bg_color=RGBColor(0x2B,0x37,0x48))
    add_text(s, name, sx + Inches(0.25), Inches(3.7), Inches(3.15), Inches(0.45),
             font_size=14, color=ACCENT_SOFT, bold=True, font=FONT_MONO)
    add_text(s, desc, sx + Inches(0.25), Inches(4.2), Inches(3.15), Inches(0.5),
             font_size=12, color=LIGHT_GRAY)

# Q&A
add_text(s, '有问题在 GitHub 提 issue 或者直接找我 🙂', Inches(1.0), Inches(5.4), Inches(11.3), Inches(0.6),
         font_size=18, color=WHITE)
add_text(s, '本分享里所有数字和事件都来自我自己的调试记录，没有编造。如果不准确请告诉我。',
         Inches(1.0), Inches(6.1), Inches(11.3), Inches(0.6),
         font_size=12, color=LIGHT_GRAY)

# Save
prs.save(OUT)
print(f'Saved: {OUT}')
print(f'Slides: {len(prs.slides)}')
