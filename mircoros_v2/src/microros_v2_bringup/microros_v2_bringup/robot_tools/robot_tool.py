import os
import json
import socket

_ROBOT_INFO_PATH =os.path.join(os.path.expanduser('~'), 'microros_ws', '.system_cfg', 'robot_info.json')

def GetRobotType(default=None):
    car_type = os.getenv("CAR_TYPE", default)
    if car_type is None:
        raise Exception("CAR_TYPE environment variable is not set. please set it first.")
    return car_type

def GetDomainId(default=None):
    domain_id = os.getenv("ROS_DOMAIN_ID", default)
    if domain_id is None:
        raise Exception("ROS_DOMAIN_ID environment variable is not set. please set it first.")
    return domain_id

def GetRobotInfo(key: str = None):
    with open(_ROBOT_INFO_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    if key is None:
        return data
    return data.get(key)


def SetRobotInfo(key: str, value):
    with open(_ROBOT_INFO_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    data[key] = value
    with open(_ROBOT_INFO_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)

_DESKTOP_DIR = os.path.join(os.path.expanduser('~'), 'Desktop')

def GetCameraUrl()->str:
    camera_ip = GetRobotInfo("camera_ip")
    return f"http://{camera_ip}:81/stream"


def GetLocalIP()->str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "N/A"

def PrintRobotInfo():
    robot_type = GetRobotType(default="Unknown")
    info = GetRobotInfo()
    domain_id = GetDomainId(default="Unknown")
    print(f"==================MicroRosV2 Robot Info=======================")
    print(f"Robot Type:         {robot_type}")
    print(f"Domain ID:          {domain_id}")
    print(f"Current IP:         {GetLocalIP()}")
    print(f"Camera  IP:         {info.get('camera_ip', 'N/A')}")
    print(f"Vision API Key:     {info.get('Vision_API_KEY', 'N/A')}")
    print(f"Vision Base URL:    {info.get('Vision_BASE_URL', 'N/A')}")
    board_config = info.get('board_config', {})
    print(f"ROS Namespace:      {board_config.get('ros_namespace', 'N/A')}")
    print(f"Dify API Key:       {info.get('DIFY_API_KEY', 'N/A')}")
    print(f"Dify Base URL:      {info.get('DIFY_BASE_URL', 'N/A')}")
    print(f"Vision Model:       {info.get('Vision_MODEL', 'N/A')}")
    print(f"System Image Ver:   {info.get('System_Image_Version', 'N/A')}\n")
