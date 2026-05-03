import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import base64
import sys
import time
from ollama import chat


# 从命令行获取提示词（如果提供）
def get_prompt():
    if len(sys.argv) > 1:
        return sys.argv[1]
    else:
        return "图像中有什么"  # 默认提示词


# 创建并配置pipeline
pipeline = rs.pipeline()
config = rs.config()
# 启用彩色流（通用标准配置：640x480，BGR格式，30fps）
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

# 启动流
pipeline.start(config)


# 定义图像转Base64的函数（复用性更强）
def image_to_base64(image, ext=".jpg", quality=95):
    """
    将OpenCV图像转换为Base64编码字符串
    :param image: OpenCV格式的图像（numpy数组）
    :param ext: 图像格式后缀（.jpg/.png等）
    :param quality: JPG质量（0-100），PNG无效
    :return: Base64编码字符串（纯字符串，不含b'...'）
    """
    try:
        # 编码图像为二进制流
        encode_param = [int(cv.IMWRITE_JPEG_QUALITY), quality] if ext == ".jpg" else []
        retval, img_buffer = cv.imencode(ext, image, encode_param)
        if not retval:
            raise ValueError("图像编码失败")

        # 转换为Base64并解码为字符串（去除b'...'）
        base64_str = base64.b64encode(img_buffer).decode("utf-8")
        return base64_str
    except Exception as e:
        print(f"转换Base64失败: {e}")
        return None


try:
    prompt = get_prompt()
    # print(f"使用提示词: {prompt}")
    # print("相机启动中，等待自动曝光稳定...")

    # 等待相机自动曝光稳定（丢弃前30帧）
    stable_frames = 60  # 丢弃的帧数
    for i in range(stable_frames):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # if i % 10 == 0:
        # print(f"正在稳定曝光... 已丢弃 {i+1}/{stable_frames} 帧")

    # print("曝光稳定完成，捕获当前图像")

    # 获取稳定后的帧
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

    if color_frame:
        # 转换为OpenCV格式图像
        color_img = np.asanyarray(color_frame.get_data())

        # # 显示图像
        # cv.imshow('RealSense Captured Frame', color_img)
        # cv.waitKey(1000)  # 显示1秒

        # 转换为Base64并进行推理
        base64_result = image_to_base64(color_img)
        if base64_result:
            # print("正在进行推理...")
            try:
                response = chat(
                    model="qwen3.5:9b",
                    messages=[
                        {
                            "role": "user",
                            "content": prompt,
                            "images": [base64_result],
                        }
                    ],
                )
                print("\n推理结果:")
                print(response.message.content)
            except Exception as e:
                print(f"推理错误: {e}")
    else:
        print("无法获取图像帧")

    # 等待按键退出
    # print("\n按任意键退出...")
    # cv.waitKey(0)

finally:
    # 停止流并释放窗口资源
    pipeline.stop()
    cv.destroyAllWindows()

