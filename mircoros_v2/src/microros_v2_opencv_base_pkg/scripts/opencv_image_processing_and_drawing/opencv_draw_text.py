import cv2


if __name__ == '__main__':
    img = cv2.imread('/home/yahboom/microros_ws/src/microros_v2_opencv_base_pkg/scripts/images/yahboom.png', cv2.IMREAD_UNCHANGED)
    if img is None:
        print("图片读取失败，请检查 images/yahboom.png 路径是否正确")
        exit()
        
        
    cv2.putText(
        img,
        'This is Yahboom!',
        (50, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 200, 0),
        2
    )

    while True:
        cv2.imshow("draw_text", img)
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113:
            break

    '''
        关闭窗口,读取图片时，不需要调用 img.release()，因为 cv2.imread() 返回的是一个 NumPy 数组，
        而不是一个需要手动释放的资源对象。调用 img.release() 会导致错误，因为 NumPy 数组没有 release() 方法。
    '''
    cv2.destroyAllWindows()
