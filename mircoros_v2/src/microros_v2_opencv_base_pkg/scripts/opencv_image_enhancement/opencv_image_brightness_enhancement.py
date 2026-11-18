import cv2
import numpy as np

if __name__ == '__main__':
    img = cv2.imread('/home/yahboom/microros_ws/src/microros_v2_opencv_base_pkg/scripts/images/yahboom.png', cv2.IMREAD_UNCHANGED)
    if img is None:
        print("图片读取失败，请检查 images/yahboom.png 路径是否正确")
        exit()
        
        
    imgInfo = img.shape
    height = imgInfo[0]
    width = imgInfo[1]

    dst = np.zeros((height, width, 3), np.uint8)

    for i in range(0, height):
        for j in range(0, width):
            b, g, r = img[i, j]

            bb = int(b) + 100
            gg = int(g) + 100
            rr = int(r) + 100

            if bb > 255:
                bb = 255
            if gg > 255:
                gg = 255
            if rr > 255:
                rr = 255

            dst[i, j] = (bb, gg, rr)

    while True:
        cv2.imshow("yahboom_brightness_enhancement", dst)

        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break

    '''
        关闭窗口,读取图片时，不需要调用 img.release()，因为 cv2.imread() 返回的是一个 NumPy 数组，
        而不是一个需要手动释放的资源对象。调用 img.release() 会导致错误，因为 NumPy 数组没有 release() 方法。
    '''
    cv2.destroyAllWindows()