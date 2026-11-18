from cv_bridge import CvBridge


_BRIDGE = CvBridge()


def image_to_bgr(image_msg):
    return _BRIDGE.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
