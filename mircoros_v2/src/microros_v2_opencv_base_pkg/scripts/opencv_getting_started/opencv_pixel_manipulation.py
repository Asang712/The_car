import cv2

if __name__ == '__main__':
    img = cv2.imread("/home/yahboom/microros_ws/src/microros_v2_opencv_base_pkg/scripts/images/yahboom.png", cv2.IMREAD_UNCHANGED)
    if img is None:
        print("图片读取失败，请检查 images/yahboom.png 路径是否正确")
        exit()
    
    (b, g, r) = img[100, 100]
    print(b, g, r)


    # 将第10行的像素值修改为黑色
    i = 10
    j = 0
    # 注意：OpenCV中图像的像素值是以BGR（蓝绿红）的顺序存储的，而不是RGB（红绿蓝）。因此，(0, 0, 0)表示黑色，(255, 255, 255)表示白色。
    for j in range(1, 255):
        img[i, j] = (0, 0, 0)

    while True:
        cv2.imshow("yahboom", img)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113: # ord('q') 返回 'q' 的 ASCII 码，113 是 'q' 的十进制 ASCII 码
            break
        
    '''
        关闭窗口,读取图片时，不需要调用 img.release()，因为 cv2.imread() 返回的是一个 NumPy 数组，
        而不是一个需要手动释放的资源对象。调用 img.release() 会导致错误，因为 NumPy 数组没有 release() 方法。
    '''
    cv2.destroyAllWindows()
