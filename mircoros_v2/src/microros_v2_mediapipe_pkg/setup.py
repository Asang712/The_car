from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'microros_v2_mediapipe_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='2474250941@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "hand_detector_node = microros_v2_mediapipe_pkg.HandDetector:main",
            "pose_detector_node = microros_v2_mediapipe_pkg.PoseDetector:main",
            "holistic_node = microros_v2_mediapipe_pkg.Holistic:main",
            "face_mesh_node = microros_v2_mediapipe_pkg.FaceMesh:main",
            "face_detection_node = microros_v2_mediapipe_pkg.FaceDetection:main",
            "face_landmarks_node = microros_v2_mediapipe_pkg.FaceLandmarks:main",
            "objectron_node = microros_v2_mediapipe_pkg.Objectron:main",
            "virtual_paint_node = microros_v2_mediapipe_pkg.VirtualPaint:main",
            "finger_ctrl_node = microros_v2_mediapipe_pkg.FingerCtrl:main",
            "find_hand_node = microros_v2_mediapipe_pkg.FindHand:main",
            "finger_trajectory_node = microros_v2_mediapipe_pkg.FingerTrajectory:main",
            "gesture_follow_node = microros_v2_mediapipe_pkg.gestureFollow:main",
            "pose_follow_node = microros_v2_mediapipe_pkg.poseFollow:main",
            "hand_ctrl_node = microros_v2_mediapipe_pkg.HandCtrl:main",
            "mediapipe_gesture_node = microros_v2_mediapipe_pkg.mediapipe_gesture:main",
            "robot_ctrl_node = microros_v2_mediapipe_pkg.RobotCtrl:main",
        ],
    },
)
