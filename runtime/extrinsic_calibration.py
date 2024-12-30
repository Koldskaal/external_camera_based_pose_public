import cv2
import numpy as np
import draw_point_helper
import os
from typing import Tuple
import numpy.typing as npt
import math

POINTS_FILE = "points_data2.npz"

Array2DFloat = npt.NDArray[np.float32]


class Cam:
    calibrated = False

    def __init__(self) -> None:
        self.mtx: Array2DFloat = np.loadtxt("cam_values")
        self.dist: Array2DFloat = np.loadtxt("dist_values")
        self.spacepoints: Array2DFloat = np.array(
            [(0, 0, 0), (189, 0, 0), (0, 150, 0), (189, 150, 0)], dtype="f"
        ).astype(np.float32)
        self.spacepoints_cam: Array2DFloat = np.array(
            [
                (0, 0, 0),
                (189, 0, 0),
                (0, 150, 0),
                (189, 150, 0),
                (189 // 2, 0, 0),
                (189 // 2, 150, 0),
            ],
            dtype="f",
        ).astype(np.float32)

    def load(self, file_path):
        data = np.load(file_path)
        self.image_points: Array2DFloat = data["points"]
        self.tvecs: Array2DFloat = data["tvecs"]
        self.rvecs: Array2DFloat = data["rvecs"]
        self.rvecs3, _ = cv2.Rodrigues(self.rvecs)  # convert to 3x3
        self.H, status = cv2.findHomography(self.image_points, self.spacepoints)

        self.calibrated = True

    def calibrate(self, image: Array2DFloat, file_path: str = "") -> None:
        self.image_points: Array2DFloat = np.array(
            draw_point_helper.create_image_points(image), dtype="float32"
        )

        self.get_frame(image)
        self.H, status = cv2.findHomography(self.image_points, self.spacepoints)

        if file_path:
            np.savez(
                file_path, points=self.image_points, tvecs=self.tvecs, rvecs=self.rvecs
            )
        self.calibrated = True

    def get_frame(self, image: Array2DFloat) -> None:
        axis: Array2DFloat = np.float32([[10, 0, 0], [0, 10, 0], [0, 0, 10]]).reshape(
            -1, 3
        )

        # Find the rotation and translation vectors.
        ret, self.rvecs, self.tvecs = cv2.solvePnP(
            self.spacepoints,
            self.image_points,
            self.mtx,
            self.dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        self.rvecs3, _ = cv2.Rodrigues(self.rvecs)  # convert to 3x3

        imgpts, jac = cv2.projectPoints(
            axis, self.rvecs, self.tvecs, self.mtx, self.dist
        )

        image = Cam.draw(image, self.image_points, imgpts)
        cv2.imshow("img", image)
        k = cv2.waitKey(0) & 0xFF

    @staticmethod
    def draw(
        img: npt.NDArray[np.uint8], corners: Array2DFloat, imgpts: Array2DFloat
    ) -> npt.NDArray[np.uint8]:
        corner = tuple(corners[0].ravel().astype(np.uint16))
        img = cv2.line(
            img, corner, tuple(imgpts[0].ravel().astype(np.uint16)), (255, 0, 0), 5
        )
        img = cv2.line(
            img, corner, tuple(imgpts[1].ravel().astype(np.uint16)), (0, 255, 0), 5
        )
        img = cv2.line(
            img, corner, tuple(imgpts[2].ravel().astype(np.uint16)), (0, 0, 255), 5
        )
        return img

    def get_homography(self, x: float, y: float) -> Tuple[float, float]:
        assert (
            self.calibrated
        ), "Cannot run without calibration. Run calibrate or load from file"
        point_src: Array2DFloat = np.array([[x, y]], dtype="float32")
        point_src = np.array([point_src])

        # temp solution
        H_4x4 = np.eye(4)
        H_4x4[:3, :3] = self.H

        # Translation matrix to move by 15 in z
        T_z = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 15, 1]])

        # Apply translation
        H_new = (T_z @ H_4x4)[:3, :3]

        # Transform the point
        point_dst: Array2DFloat = cv2.perspectiveTransform(point_src, H_new)
        inverted_rot = self.rvecs3.T
        inverted_tran = -inverted_rot @ self.tvecs
        # print(inverted_tran, inverted_rot, self.tvecs)
        x = point_dst[0][0][0]
        y = point_dst[0][0][1]
        z = inverted_tran[2]
        # Fixing offset from box being approximately 15 cm above ground plane
        scaling = 15 / z
        yd = y - (y - inverted_tran[1]) * scaling
        xd = x - (x - inverted_tran[0]) * scaling

        # returns transformed_x, transformed_y

        return int(xd), int(yd)


if __name__ == "__main__":

    cam = Cam()
    if os.path.exists(POINTS_FILE):
        cam.load(POINTS_FILE)
    else:
        # Calibrate
        image_path = "baseline_Color.png"  # Replace with your image path
        original_image = cv2.imread(image_path)
        cam.calibrate(original_image, POINTS_FILE)

    print(cam.get_homography(320, 405))
    print(cam.tvecs)
    print(cam.rvecs)
