"""
职责:
  - 订阅底盘摄像头图像 (aruco_raw)
  - 检测 ArUco 二维码
  - 当识别到特定二维码时，发布 dock_detected 信号
  - 用于触发机械臂复位流程
"""

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from send_interfaces.action import NavAndExecute
from ultralytics import YOLO
from send_interfaces.msg import SendData


# STM32 USB command IDs. 这些流程命令允许携带空载荷或 4 个 float，
# send2stm32 统一使用 4 个 0.0，避免通信桥需要额外支持空载荷。
CMD_CHASSIS_SET_VEL = 0x13
CMD_FLOW_WEAPON_DOCK_TEST = 0x69
CMD_FLOW_CHASSIS_MOVE_DONE = 0x6A
CMD_FLOW_DOCK_DONE = 0x6B

ARM_STAGE_IDLE = 0
ARM_STAGE_GRAB_STARTED = 1
ARM_STAGE_CHASSIS_ACKING = 2
ARM_STAGE_DOCK_ACKING = 3
ARM_STAGE_FINISHED = 4


def abs_clip(x, x_max):
    return max(min(x, x_max), -x_max)
class PIDXY:
    def __init__(self, kp , ki, kd, max_integral):
        self._kp_x = kp[0]
        self._ki_x = ki[0]
        self._kd_x = kd[0]
        self._kp_y = kp[1]
        self._ki_y = ki[1]
        self._kd_y = kd[1]
        self._now = None
        self._target = None
        self._error_x = None
        self._error_y = None
        self._last_error_x = 0.0
        self._last_error_y = 0.0
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._max_integral = max_integral

    def set_target(self, target):
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._target = target

    def output(self, now, c=1):
        self._error_x = self._target[0] - now[0]
        self._error_y = self._target[1] - now[1]
        self._integral_x += self._error_x
        self._integral_y += self._error_y
        self._integral_x = abs_clip(self._integral_x, self._max_integral)
        self._integral_y = abs_clip(self._integral_y, self._max_integral)
        d_x = self._error_x - self._last_error_x
        d_y = self._error_y - self._last_error_y
        output_x = self._kp_x * self._error_x + self._ki_x * self._integral_x + self._kd_x * d_x
        output_y = self._kp_y * self._error_y + self._ki_y * self._integral_y + self._kd_y * d_y
        self._last_error_x = self._error_x
        self._last_error_y = self._error_y
        return output_x * c, output_y * c


class SolveR2Pose(Node):
    def __init__(self):
        super().__init__('solve_r2_pose')
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter("model_path", "/home/vinciorin/Desktop/r2_ws/src/r2_zone1_hall/models/best(16).engine")
        self.declare_parameter("use_tensorrt", True)
        self.declare_parameter("target_aruco_id", 1)
        self.declare_parameter("chassis_control_period_sec", 0.05)
        self.declare_parameter("alignment_timeout_sec", 30.0)
        self.declare_parameter("dock_detect_timeout_sec", 60.0)
        self.declare_parameter("arm_command_interval_sec", 0.1)
        self.declare_parameter("dock_ack_retry_sec", 45.0)
        self.pub = self.create_publisher(SendData, "send2stm32", 10)

        self.target_aruco_id = int(self.get_parameter("target_aruco_id").value)
        self.chassis_control_period_sec = max(
            0.01, float(self.get_parameter("chassis_control_period_sec").value)
        )
        self.alignment_timeout_sec = float(
            self.get_parameter("alignment_timeout_sec").value
        )
        self.dock_detect_timeout_sec = float(
            self.get_parameter("dock_detect_timeout_sec").value
        )
        self.arm_command_interval_sec = max(
            0.02, float(self.get_parameter("arm_command_interval_sec").value)
        )
        self.dock_ack_retry_sec = max(
            0.1, float(self.get_parameter("dock_ack_retry_sec").value)
        )
        self.arm_stage = ARM_STAGE_IDLE
        self.arm_stage_lock = threading.Lock()
        self.action_lock = threading.Lock()
        self.dock_ack_started_at = None
        self.arm_command_timer = self.create_timer(
            self.arm_command_interval_sec,
            self.arm_command_timer_cb,
            callback_group=self.cb_group,
        )

        model_path = self.get_parameter("model_path").value
        use_tensorrt = self.get_parameter("use_tensorrt").value
        if use_tensorrt and not model_path.endswith(".engine"):
            self.get_logger().warn("use_tensorrt=true 但模型路径不是 .engine，已自动改用 .pt")
            model_path = model_path.replace(".engine", ".pt")

        self.model = None
        self.get_logger().info(f"正在加载 YOLO 模型: {model_path}")
        try:
            self.model = YOLO(model_path, task="detect")
            self.get_logger().info("YOLO 模型加载完成")
        except Exception as e:
            self.get_logger().error(f"YOLO 模型加载失败: {e}")
        self.action_ser = ActionServer(self, NavAndExecute, 'nav_and_execute',self.ser_cb,callback_group=self.cb_group)
        self.sub_img = self.create_subscription(
            Image, 'aruco_raw', self.img_cb, 10,callback_group=self.cb_group
        )
        self.sub_img_head = self.create_subscription(
            Image, 'image_jiazhua', self.img_cb_head, 10,callback_group=self.cb_group
        )
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        # OpenCV 4.7+ 使用 DetectorParameters()，旧版使用 DetectorParameters_create()
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        # OpenCV 4.7+ 才支持 ArucoDetector；4.6 及以下用旧 detectMarkers API
        self._use_new_aruco = tuple(map(int, cv2.__version__.split('.')[:2])) >= (4, 7)
        if self._use_new_aruco:
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.cornerRefinementWinSize = 7
            self.aruco_params.cornerRefinementMaxIterations = 100
            self.aruco_params.cornerRefinementMinAccuracy = 0.001
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_detector = None
        self.aruco_params.minMarkerPerimeterRate = 0.05
        self.aruco_params.errorCorrectionRate = 0.4
        self.ready = False
        self.cx,self.cy = None,None
        self.w,self.h = None,None
        self.detections_timestamp = self.get_clock().now()
        self.timeout_sec = 0.8

        self.get_logger().info(
            f'ArUco 识别节点已启动，OpenCV={cv2.__version__}, '
            f'使用新 API={self._use_new_aruco}'
        )
        self.pid_xy = PIDXY((0.0001,0.0001),(0,0),(0,0),10)
        # PID 输入顺序是 (-距离, 图像横坐标)，目标必须保持同一顺序。
        self.pid_xy.set_target((-150, 640))

    @staticmethod
    def estimate_distance(pixel_width, pixel_height):
        Z1 = (27.6 * 978.67967778) / pixel_width
        Z2 = (37.7 * 979.27602966) / pixel_height
        return (Z2 + Z1) / 2.0
    
    
    def ser_cb(self, goal_handle):
        if not self.action_lock.acquire(blocking=False):
            self.get_logger().error("已有 nav_and_execute 动作正在执行")
            goal_handle.abort()
            return NavAndExecute.Result(success=False)

        try:
            action_type = goal_handle.request.action_type
            if action_type == 1:
                return self.align_and_start_weapon_grab(goal_handle)
            elif action_type == 2:
                return self.confirm_chassis_and_wait_for_dock(goal_handle)

            self.get_logger().error(f"不支持的 action_type: {action_type}")
            goal_handle.abort()
            return NavAndExecute.Result(success=False)
        finally:
            self.action_lock.release()

    def align_and_start_weapon_grab(self, goal_handle):
        """视觉对准武器头；对准完成后启动 MCU 的夹取/对接流程。"""
        started_at = time.monotonic()
        self.pid_xy.set_target((-150, 640))

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_chassis()
                goal_handle.canceled()
                return NavAndExecute.Result(success=False)

            if time.monotonic() - started_at > self.alignment_timeout_sec:
                self.stop_chassis()
                self.get_logger().error("武器头视觉对准超时")
                goal_handle.abort()
                return NavAndExecute.Result(success=False)

            now = self.get_clock().now()
            det_valid = (
                (now - self.detections_timestamp).nanoseconds / 1e9
                < self.timeout_sec
            )
            if det_valid and self.cx is not None and self.w and self.h:
                cx = float(self.cx)
                z = float(self.estimate_distance(float(self.w), float(self.h)))
                vx, vy = self.pid_xy.output((-z, cx))
                vx, vy = float(vx), float(vy)

                if abs(cx - 640.0) < 50.0 and abs(z - 150.0) < 50.0:
                    self.stop_chassis()
                    self.send_flow_command(
                        CMD_FLOW_WEAPON_DOCK_TEST,
                        "启动武器头夹取/对接流程",
                    )
                    with self.arm_stage_lock:
                        self.arm_stage = ARM_STAGE_GRAB_STARTED
                        self.dock_ack_started_at = None
                    goal_handle.succeed()
                    return NavAndExecute.Result(success=True)

                # CHS_SET_VEL 有 100 ms 看门狗；这里默认每 50 ms 刷新一次。
                self.send_single(CMD_CHASSIS_SET_VEL, [vx, vy, 0.0, 0.0])

            time.sleep(self.chassis_control_period_sec)

        self.stop_chassis()
        goal_handle.abort()
        return NavAndExecute.Result(success=False)

    def confirm_chassis_and_wait_for_dock(self, goal_handle):
        """底盘移动完成后确认 checkpoint 1，识别二维码后确认 checkpoint 2。"""
        with self.arm_stage_lock:
            if self.arm_stage not in (
                ARM_STAGE_GRAB_STARTED,
                ARM_STAGE_CHASSIS_ACKING,
            ):
                self.get_logger().error("机械臂夹取流程尚未启动，不能确认底盘完成")
                goal_handle.abort()
                return NavAndExecute.Result(success=False)
            self.ready = False  # 必须使用本次 action 开始后的 ArUco 检测结果
            self.arm_stage = ARM_STAGE_CHASSIS_ACKING

        # 立即发送一次，后续由 100 ms timer 重试，直到检测到对接二维码。
        self.send_flow_command(
            CMD_FLOW_CHASSIS_MOVE_DONE,
            "底盘移动完成，确认机械臂 checkpoint 1",
        )
        started_at = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.arm_stage_lock:
                    self.arm_stage = ARM_STAGE_GRAB_STARTED
                goal_handle.canceled()
                return NavAndExecute.Result(success=False)

            if self.ready:
                with self.arm_stage_lock:
                    self.arm_stage = ARM_STAGE_DOCK_ACKING
                    self.dock_ack_started_at = time.monotonic()
                self.send_flow_command(
                    CMD_FLOW_DOCK_DONE,
                    "检测到对接二维码，确认机械臂 checkpoint 2",
                )
                goal_handle.succeed()
                return NavAndExecute.Result(success=True)

            if time.monotonic() - started_at > self.dock_detect_timeout_sec:
                with self.arm_stage_lock:
                    self.arm_stage = ARM_STAGE_GRAB_STARTED
                self.get_logger().error("等待对接 ArUco 二维码超时")
                goal_handle.abort()
                return NavAndExecute.Result(success=False)

            time.sleep(0.05)

        goal_handle.abort()
        return NavAndExecute.Result(success=False)

    def arm_command_timer_cb(self):
        """无 STM32 状态订阅时，重发检查点确认；下位机只接受当前检查点。"""
        with self.arm_stage_lock:
            stage = self.arm_stage
            dock_ack_started_at = self.dock_ack_started_at

            if (
                stage == ARM_STAGE_DOCK_ACKING
                and dock_ack_started_at is not None
                and time.monotonic() - dock_ack_started_at >= self.dock_ack_retry_sec
            ):
                self.arm_stage = ARM_STAGE_FINISHED
                self.dock_ack_started_at = None
                return

        if stage == ARM_STAGE_CHASSIS_ACKING:
            self.send_flow_command(CMD_FLOW_CHASSIS_MOVE_DONE)
        elif stage == ARM_STAGE_DOCK_ACKING:
            self.send_flow_command(CMD_FLOW_DOCK_DONE)

    def stop_chassis(self):
        self.send_single(CMD_CHASSIS_SET_VEL, [0.0, 0.0, 0.0, 0.0])

    def send_flow_command(self, cmd, description=None):
        if description:
            self.get_logger().info(f"{description}: CMD=0x{cmd:02X}")
        self.send_single(cmd, [0.0, 0.0, 0.0, 0.0])





    def img_cb_head(self, msg):
        if self.model is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        ret = self.model.predict(source=img, verbose=False)[0].cpu()
        if ret.boxes is None or len(ret.boxes.xyxy) == 0:
            return
        for x1,y1,x2,y2 in ret.boxes.xyxy:
            self.detections_timestamp = self.get_clock().now()
            self.cx, self.cy =(x1 + x2) / 2,(y1 + y2 ) / 2
            self.w,self.h = x2 - x1,y2 - y1
            self.get_logger().info(f"{self.cx=} {self.cy=}")

    @staticmethod
    def get_dist_2(x1,y1,x2,y2):
        return (x2 - x1) ** 2 + (y2 - y1) ** 2
    def img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self._use_new_aruco:
            corners, ids, rejected = self.aruco_detector.detectMarkers(img)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                img, self.aruco_dict, parameters=self.aruco_params
            )
        if ids is not None:
            id_list = ids.reshape(-1).tolist()
            self.get_logger().info(f"{id_list}")
            for i in id_list:
                if i == self.target_aruco_id:
                    self.get_logger().info(f"detect target ArUco {i}")
                    self.ready = True



    def send_single(self,cmd, data_list):
        msg = SendData()
        msg.cmd = cmd
        msg.data_type = 2
        msg.float_arr = [float(x) for x in data_list]
        msg.int_arr = []
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SolveR2Pose()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
