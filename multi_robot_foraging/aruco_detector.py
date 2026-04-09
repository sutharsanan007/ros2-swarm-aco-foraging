import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import cv2.aruco as aruco
import numpy as np
import math

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # --- PARAMETERS ---
        # marker_size: physical side length in metres.
        # Override per-robot in launch if the home marker (ID 0) is 0.60 m.
        self.declare_parameter('marker_size', 0.20)
        self.marker_size = self.get_parameter('marker_size').value

        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        self.pose_pub  = self.create_publisher(Float32MultiArray, 'aruco_poses',       10)
        self.image_pub = self.create_publisher(Image,             'camera/image_annotated', 10)

        # ArUco dictionary — fallback for old OpenCV API
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters()
        except AttributeError:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters_create()

        # 3-D corners of a flat square marker (Z=0 plane)
        self._update_obj_points()

        # ── EXACT camera matrix from model.sdf ──────────────────────────
        # horizontal_fov = 1.089 rad, width = 640, height = 480
        # fx = fy = (width/2) / tan(fov/2) = 320 / tan(0.5445) = 532.5
        W, H = 640, 480
        fov  = 1.089                              # radians — from SDF
        fx   = (W / 2.0) / math.tan(fov / 2.0)   # ≈ 532.5
        self.camera_matrix = np.array([
            [fx,   0.0, W / 2.0],
            [0.0,  fx,  H / 2.0],
            [0.0,  0.0, 1.0    ]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.get_logger().info(
            f'ArucoDetector ready — marker_size={self.marker_size:.2f}m, fx={fx:.1f}'
        )

    def _update_obj_points(self):
        """Recompute 3-D object corners whenever marker_size changes."""
        h = self.marker_size / 2.0
        self.obj_points = np.array([
            [-h,  h, 0],
            [ h,  h, 0],
            [ h, -h, 0],
            [-h, -h, 0],
        ], dtype=np.float32)

    # ------------------------------------------------------------------ #

    def image_callback(self, msg: Image):
        # ── Decode raw bytes → BGR ──────────────────────────────────────
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1    # -1 handles 1, 3, or 4 channels
            )
            if img.shape[2] == 4:
                code = (cv2.COLOR_RGBA2BGR
                        if 'rgba' in msg.encoding.lower()
                        else cv2.COLOR_BGRA2BGR)
                cv_image = cv2.cvtColor(img, code)
            elif 'rgb' in msg.encoding.lower():
                cv_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                cv_image = img.copy()
        except Exception as e:
            self.get_logger().error(f'Image decode failed: {e}')
            return

        # ── ArUco detection ─────────────────────────────────────────────
        try:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )

            # FIX: pose_data prefixed with marker count so the subscriber
            # can parse it safely even if the marker count changes mid-frame.
            pose_data = []
            if ids is not None:
                pose_data.append(float(len(ids)))     # [0] = count
                for i in range(len(ids)):
                    ok, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[i][0],
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if ok:
                        mid_x = int(corners[i][0][:, 0].mean())
                        mid_y = int(corners[i][0][:, 1].mean())
                        pose_data.extend([
                            float(ids[i][0]),
                            float(tvec[0][0]),  # lateral offset (camera X)
                            float(tvec[1][0]),  # vertical  offset (camera Y)
                            float(tvec[2][0]),  # depth from camera (Z)
                        ])
                        cv2.putText(
                            cv_image,
                            f'ID:{int(ids[i][0])} Z:{tvec[2][0]:.2f}m',
                            (mid_x - 40, mid_y - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                        )
                aruco.drawDetectedMarkers(cv_image, corners, ids)
            else:
                pose_data.append(0.0)   # count = 0

            self.pose_pub.publish(Float32MultiArray(data=pose_data))

        except Exception as e:
            self.get_logger().error(f'Detection pipeline error: {e}')

        # ── Republish annotated frame ────────────────────────────────────
        out             = Image()
        out.header      = msg.header
        out.height      = cv_image.shape[0]
        out.width       = cv_image.shape[1]
        out.encoding    = 'bgr8'
        out.step        = cv_image.shape[1] * 3
        out.data        = cv_image.tobytes()
        self.image_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()