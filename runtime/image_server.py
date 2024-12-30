# Import the required modules
import socket
import pickle
import numpy as np
import struct  ## new

import pyrealsense2 as rs
import numpy as np
import cv2
import os

from extrinsic_calibration import Cam
import ai
from vector_math import project_vector_to_plane, angle_between_vectors_2d, unit_vector
from orientation_model import detect
from PIL import Image


HOST = ""
PORT = 5006
WIDTH = 1280
HEIGHT = 720
FPS = 30
CAM_DATA_FILE = "cam_data.npz"


def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == "RGB Camera":
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    # Start streaming
    pipeline.start(config)

    depth_sensor = pipeline_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    align_to = rs.stream.color  # Align depth to color
    align = rs.align(align_to)

    #### Initialize Cam
    cam = Cam()
    if os.path.exists(CAM_DATA_FILE):
        cam.load(CAM_DATA_FILE)
        print(cam.rvecs)
        print(cam.tvecs)
    else:
        # Calibrate
        for i in range(60):  # throw away frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        cam.calibrate(np.asanyarray(color_frame.get_data()), CAM_DATA_FILE)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # experiment to reduce lag
        print("Socket created")

        s.bind((HOST, PORT))
        print("Socket bind complete")
        s.listen(10)
        print("Socket now listening")
        old_theta = 0
        while True:
            conn, addr = s.accept()
            print(f"New Connection: {addr}")
            with conn:
                while True:
                    key = cv2.pollKey()
                    if key == ord("q"):  # exit
                        return

                    frames = pipeline.wait_for_frames()
                    frames = align.process(frames)

                    color_frame = frames.get_color_frame()
                    color_image = np.asanyarray(color_frame.get_data())

                    # send the image through the ai pipeline
                    bounding_boxes = ai.detect(color_image)

                    x = 0
                    y = 0
                    biggest_conf = 0

                    copy = color_image.copy()  # dont write on original

                    for box in bounding_boxes:
                        # Coordinates of the bounding box
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        x1 = int(x1)
                        x2 = int(x2)
                        y1 = int(y1)
                        y2 = int(y2)

                        # Calculate center point
                        cx = x1 + (x2 - x1) / 2
                        cy = y1 + (y2 - y1) / 2

                        # Confidence score
                        confidence = box.conf[0].item()

                        # Draw the rectangle
                        cv2.rectangle(
                            copy, (x1, y1), (x2, y2), (0, 255, 0), 2
                        )  # Green box with thickness 2

                        # Prepare label with class name and confidence
                        label = f"Robot: {confidence:.2f}"

                        # Choose font and get label size
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        label_size, _ = cv2.getTextSize(label, font, 0.5, 1)

                        # Draw background rectangle for label
                        cv2.rectangle(
                            copy,
                            (x1, y1 - label_size[1] - 10),
                            (x1 + label_size[0], y1),
                            (0, 255, 0),
                            cv2.FILLED,
                        )

                        # Put label text above the bounding box
                        cv2.putText(copy, label, (x1, y1 - 5), font, 0.5, (0, 0, 0), 1)

                        if confidence > biggest_conf:
                            x = cx
                            y = cy
                            biggest_conf = confidence

                    if biggest_conf > 0:
                        # send pose x, y, theta
                        x_world, y_world = cam.get_homography(x, y)  # project to plane
                        size = 100
                        cropped = color_image[
                            int(y) - size // 2 : int(y) + size // 2,
                            int(x) - size // 2 : int(x) + size // 2,
                        ]
                        pil_img = Image.fromarray(
                            cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
                        )
                        theta = detect(pil_img)

                        new_y = unit_vector(
                            project_vector_to_plane([0, 1, 0], cam.rvecs3[:, 0])
                        )
                        # adjust to world frame
                        theta = (
                            theta - angle_between_vectors_2d([0, 1], new_y[:2])
                        ) % 360

                        x = int(x)
                        y = int(y)
                        font = cv2.FONT_HERSHEY_SIMPLEX

                        cv2.putText(
                            copy,
                            f"x = {x_world}",
                            (x, y - 140),
                            font,
                            0.5,
                            (255, 255, 255),
                            1,
                        )
                        cv2.putText(
                            copy,
                            f"y = {y_world}",
                            (x, y - 110),
                            font,
                            0.5,
                            (255, 255, 255),
                            1,
                        )
                        color = (0, 255, 0)
                        if abs(theta - old_theta) < 300 and abs(theta - old_theta) > 60:
                            color = (0, 0, 255)

                        cv2.putText(
                            copy,
                            f"theta = {theta}",
                            (x, y - 80),
                            font,
                            0.5,
                            color,
                            1,
                        )
                        old_theta = theta
                        # write_to_image(copy, f"x = {x_world}", (x, y - 300))
                        # write_to_image(copy, f"y = {y_world}", (x, y - 250))
                        # write_to_image(copy, f"theta = {theta}", (x, y - 200))

                        # packing as big-endian 3x 16-bit signed
                        conn.sendall(
                            struct.pack("!hhh", int(x_world), int(y_world), int(theta))
                        )

                    cv2.imshow("Image with Bounding Boxes", copy)
    cv2.destroyAllWindows()
    pipeline.stop()


def write_to_image(image, text: str, position: tuple):
    # Choose font and get label size
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(image, text, position, font, 0.5, (0, 0, 0), 1)


if __name__ == "__main__":
    main()
