import cv2
import numpy as np

if __name__ == '__main__':
    dam_img = cv2.imread('/home/yahboom/microros_ws/src/microros_v2_opencv_base_pkg/scripts/images/damaged.png', cv2.IMREAD_UNCHANGED)
    if dam_img is None:
        print("图片读取失败，请检查 images/damaged.png 路径是否正确")
        exit()

    imgInfo = dam_img.shape
    height = imgInfo[0]
    width = imgInfo[1]

    # 创建单通道掩码图
    paint = np.zeros((height, width, 1), np.uint8)

    for i in range(50, 100):
        paint[i, 50] = 255
        paint[i, 50 + 1] = 255
        paint[i, 50 - 1] = 255

    for i in range(100, 150):
        paint[150, i] = 255
        paint[150 + 1, i] = 255
        paint[150 - 1, i] = 255

    # 图像修复
    dst_img = cv2.inpaint(dam_img, paint, 3, cv2.INPAINT_TELEA)

    while True:
        cv2.imshow("dam_img", dam_img)
        cv2.imshow("paint", paint)
        cv2.imshow("dst", dst_img)

        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break

    '''
        关闭窗口,读取图片时，不需要调用 img.release()，因为 cv2.imread() 返回的是一个 NumPy 数组，
        而不是一个需要手动释放的资源对象。调用 img.release() 会导致错误，因为 NumPy 数组没有 release() 方法。
    '''
    cv2.destroyAllWindows()