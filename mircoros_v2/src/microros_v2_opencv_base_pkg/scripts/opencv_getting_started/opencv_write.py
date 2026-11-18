import cv2


if __name__ == "__main__":
    
    # 读取图片
    img = cv2.imread("/home/yahboom/microros_ws/src/microros_v2_opencv_base_pkg/scripts/images/yahboom.png", cv2.IMREAD_UNCHANGED)

    # 判断图片是否读取成功
    if img is None:
        print("图片读取失败，请检查 images/yahboom.png 路径是否正确")
        exit()

    print("图片尺寸信息:", img.shape)

    # 保存图片
    save_path = "/home/yahboom/microros_ws/src/microros_v2_opencv_base_pkg/scripts/images/yahboom_copy.png"
    if cv2.imwrite(save_path, img):
        print(f"图片成功保存到 {save_path}")
    else:
        print(f"图片保存失败，请检查路径 {save_path} 是否正确")
        
        
    copy_img = cv2.imread(save_path, cv2.IMREAD_UNCHANGED)
    if copy_img is None:
        print("复制图片读取失败，请检查保存路径是否正确")
        exit()
    
    print("复制图片尺寸信息:", copy_img.shape)
    
    while True:
        # 显示原图和复制图
        cv2.imshow("Original Image", img)
        cv2.imshow("Copied Image", copy_img)

        # 等待按键事件，按下 'q' 键退出
        action = cv2.waitKey(10) & 0xFF
        if action == ord('q') or action == 113: # ord('q') 返回 'q' 的 ASCII 码，113 是 'q' 的十进制 ASCII 码
            break
    
    '''
        关闭窗口,读取图片时，不需要调用 img.release()，因为 cv2.imread() 返回的是一个 NumPy 数组，
        而不是一个需要手动释放的资源对象。调用 img.release() 会导致错误，因为 NumPy 数组没有 release() 方法。
    '''
    cv2.destroyAllWindows()
