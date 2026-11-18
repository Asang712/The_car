import re
import json
import psutil
import os
import yaml
import math
import cv2
import numpy as np

# ---------------------------------------------------------------------------
# tool function
# ---------------------------------------------------------------------------

def extract_json_content(
    raw_content,
):  # 解析变量提取json / Extract JSON by parsing variables
    try:
        # 方法一：分割代码块 / Method 1: Split code blocks
        if "```json" in raw_content:
            # 分割代码块并取中间部分 / Split code blocks and take the middle part
            json_str = raw_content.split("```json")[1].split("```")[0].strip()
        elif "```" in raw_content:
            # 处理没有指定类型的代码块 / Handle code blocks without specified types
            json_str = raw_content.split("```")[1].strip()
        else:
            # 直接尝试解析 / Try parsing directly
            json_str = raw_content

        # 方法二：正则表达式提取（备用方案） / Method 2: Regular expression extraction (backup plan)
        if not json_str:

            match = re.search(r"\{.*\}", raw_content, re.DOTALL)
            if match:
                json_str = match.group()

        return json.loads(json_str)

    except Exception as e:
        return None
    

def kill_process_tree(pid):
    try:
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            child.kill()
        parent.kill()
    except psutil.NoSuchProcess:
        pass

def get_yaw_from_quaternion(quaternion):
    ''' Get yaw angle from quaternion'''
    siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def normalize_angle(angle):
    ''' Normalize angle to [-π, π] range'''
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def calculate_distance(start_pose, current_pose):
    '''Calculate distance between two points'''
    dx = current_pose.position.x - start_pose.position.x
    dy = current_pose.position.y - start_pose.position.y
    return math.sqrt(dx*dx + dy*dy)

def calculate_angle_diff(target_yaw, current_yaw):
    '''Calculate angle difference'''
    diff = target_yaw - current_yaw
    return normalize_angle(diff)


def draw_grid_overlay(
    image,
    grid_size=40,
    line_color=(200, 200, 200),
    line_thickness=1,
    alpha=0.6,
    font_scale=0.3,
    font_thickness=1,
    text_color=(0, 0, 0) 
):

    h, w = image.shape[:2]
    overlay = image.copy()

 
    for x in range(0, w, grid_size):
        cv2.line(overlay, (x, 0), (x, h), line_color, line_thickness)

    for y in range(0, h, grid_size):
        cv2.line(overlay, (0, y), (w, y), line_color, line_thickness)


    grid_image = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)

    return grid_image





def resample_audio(samples: np.ndarray, src_rate: int, dst_rate: int) -> np.ndarray:
    """线性插值重采样（将麦克风音频从硬件采样率降至 16 kHz）。"""
    if src_rate == dst_rate:
        return samples
    n_out = int(len(samples) * dst_rate / src_rate)
    indices = np.linspace(0, len(samples) - 1, n_out)
    return np.interp(indices, np.arange(len(samples)), samples).astype(np.float32)


# ---------------------------------------------------------------------------
# tool class
# ---------------------------------------------------------------------------



class LogTranslator:
    """
    日志翻译类 / Log Translation Class
    """
    def __init__(self, language='zh'):
        
        self.language = language
        self.translations = {}
    def load_translations_file(self,language_dir:str)->bool:
        '''加载翻译文件 / Load translation file'''
        lang_file = os.path.join(language_dir, f"{self.language}.yaml")
        try:
            with open(lang_file, 'r', encoding='utf-8') as f:
                self.translations = yaml.safe_load(f)
            return True
        except :
            return False
    
    def get_text(self, key:str, **kwargs)->str:
        '''获取翻译 / Get translation'''
        text = self.translations.get(key)
        try:
            return text.format(**kwargs)
        except KeyError:
            print("The translation key cannot be found. Please check if the language configuration file has been accidentally modified")
            return text
        
