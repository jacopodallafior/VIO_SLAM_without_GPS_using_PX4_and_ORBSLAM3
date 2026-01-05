#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool
from px4_msgs.msg import VehicleAttitude, VehicleOdometry


# ORB-SLAM3 tracking_state typical values (as you stated)
STATE_OK = 2
STATE_RECENTLY_LOST = 3
STATE_LOST = 4


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def rot2d(x: float, y: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def time_to_us(stamp) -> int:
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1_000


def yaw_from_ros_quat_enu(qx: float, qy: float, qz: float, qw: float) -> float:
    # Yaw around +Z, ROS ENU convention
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_from_px4_attitude(msg: VehicleAttitude) -> float:
    # msg.q = [w,x,y,z], rotation body FRD -> world NED
    w, x, y, z = map(float, msg.q)
    # yaw = atan2(R21, R11) from quaternion->rot matrix
    ww, xx, yy, zz = w*w, x*x, y*y, z*z
    r11 = ww + xx - yy - zz
    r21 = 2.0 * (x*y + w*z)
    return math.atan2(r21, r11)


class OrbVioToPx4EV(Node):
    """
    Subscribes ORB pose + tracking state, and PX4 attitude.
    Publishes external-vision odometry to PX4 in NED.

    Key idea:
      - ORB world yaw is gauge (arbitrary), so XY can be rotated.
      - We compute ONE constant yaw_offset when ORB tracking is stable:
            yaw_offset = yaw_px4 - yaw_vio_ned
      - Then we rotate all incoming VIO positions by yaw_offset before publishing as NED.
      - If ORB becomes RECENTLY_LOST/LOST, we stop publishing and increment reset_counter.
    """

    def __init__(self):
        super().__init__("orb_vio_to_px4_ev")

        # ---- parameters ----
        self.declare_parameter("pose_topic", "/orb_slam3/pose")
        self.declare_parameter("tracking_state_topic", "/orb_slam3/tracking_state")
        self.declare_parameter("tracking_ok_topic", "/orb_slam3/tracking_ok")
        self.declare_parameter("att_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("out_topic", "/fmu/in/vehicle_visual_odometry")

        self.declare_parameter("ok_count_to_lock", 30)     # consecutive OK poses before lock
        self.declare_parameter("publish_z", False)         # you want XY only initially
        self.declare_parameter("use_origin_on_lock", True) # origin set when lock happens

        # variances (m^2) for EKF2 if EKF2_EV_NOISE_MD = 0
        self.declare_parameter("pos_std_xy_m", 0.10)
        self.declare_parameter("pos_std_z_m", 1.00)

        self.declare_parameter("quality_ok", 100)

        # ---- read params ----
        self.pose_topic = self.get_parameter("pose_topic").value
        self.tracking_state_topic = self.get_parameter("tracking_state_topic").value
        self.tracking_ok_topic = self.get_parameter("tracking_ok_topic").value
        self.att_topic = self.get_parameter("att_topic").value
        self.out_topic = self.get_parameter("out_topic").value

        self.ok_count_to_lock = int(self.get_parameter("ok_count_to_lock").value)
        self.publish_z = bool(self.get_parameter("publish_z").value)
        self.use_origin_on_lock = bool(self.get_parameter("use_origin_on_lock").value)

        self.pos_std_xy = float(self.get_parameter("pos_std_xy_m").value)
        self.pos_std_z = float(self.get_parameter("pos_std_z_m").value)
        self.quality_ok = int(self.get_parameter("quality_ok").value)

        # ---- QoS ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        


        # ---- subscriptions / pub ----
        self.sub_pose = self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, qos)
        self.sub_state = self.create_subscription(Int32, self.tracking_state_topic, self.on_state, qos)
        self.sub_ok = self.create_subscription(Bool, self.tracking_ok_topic, self.on_ok, qos)
        self.sub_att = self.create_subscription(VehicleAttitude, self.att_topic, self.on_att, qos)
        self.pub = self.create_publisher(VehicleOdometry, self.out_topic, qos_pub)

        # ---- internal state ----
        self.tracking_state: int = -1
        self.tracking_ok: bool = False

        self.ok_pose_count: int = 0
        self.locked: bool = False
        self.yaw_offset: float = 0.0
        self.px4_yaw: Optional[float] = None

        self.origin_enu: Optional[Tuple[float, float, float]] = None
        self.reset_counter: int = 0

        self.get_logger().info("ORB->PX4 EV bridge started")
        self.get_logger().info(f"pose: {self.pose_topic} (assumed ENU)")
        self.get_logger().info(f"state: {self.tracking_state_topic} (Int32, OK=2)")
        self.get_logger().info(f"ok: {self.tracking_ok_topic} (Bool)")
        self.get_logger().info(f"att: {self.att_topic} (PX4 yaw)")
        self.get_logger().info(f"out: {self.out_topic} (VehicleOdometry NED)")

    # ---------- callbacks ----------
    def on_state(self, msg: Int32):
        self.tracking_state = int(msg.data)

    def on_ok(self, msg: Bool):
        ok = bool(msg.data)

        # Transition OK -> not OK => treat as reset event
        if self.tracking_ok and (not ok):
            self.locked = False
            self.ok_pose_count = 0
            if self.use_origin_on_lock:
                self.origin_enu = None
            self.reset_counter = (self.reset_counter + 1) % 256
            self.get_logger().warn(
                f"Tracking lost (state={self.tracking_state}) -> unlock, reset_counter={self.reset_counter}"
            )

        self.tracking_ok = ok

    def on_att(self, msg: VehicleAttitude):
        self.px4_yaw = yaw_from_px4_attitude(msg)

    def try_lock(self, pose_msg: PoseStamped) -> bool:
        """
        Lock yaw_offset only when:
          - tracking_ok is true (state==2)
          - we have PX4 yaw
          - we have seen ok_count_to_lock consecutive OK poses
        """
        if self.locked:
            return True

        if not self.tracking_ok:
            self.ok_pose_count = 0
            return False

        if self.px4_yaw is None:
            return False

        self.ok_pose_count += 1
        if self.ok_pose_count < self.ok_count_to_lock:
            return False

        # VIO yaw from pose quaternion (ENU), convert to NED yaw
        q = pose_msg.pose.orientation
        yaw_vio_enu = yaw_from_ros_quat_enu(q.x, q.y, q.z, q.w)
        yaw_vio_ned = wrap_pi((math.pi / 2.0) - yaw_vio_enu)

        self.yaw_offset = wrap_pi(self.px4_yaw - yaw_vio_ned)
        self.yaw_offset = wrap_pi(self.yaw_offset + math.pi/2)

        self.locked = True

        if self.use_origin_on_lock:
            p = pose_msg.pose.position
            self.origin_enu = (float(p.x), float(p.y), float(p.z))

        self.get_logger().info(
            f"LOCKED: state={self.tracking_state}, "
            f"yaw_offset={math.degrees(self.yaw_offset):.1f} deg, "
            f"px4_yaw={math.degrees(self.px4_yaw):.1f} deg"
        )
        return True

    def on_pose(self, msg: PoseStamped):
        # only publish when locked and tracking OK
        if not self.try_lock(msg):
            return
        if not self.tracking_ok:
            return

        # ENU position, optionally origin-shifted
        p = msg.pose.position
        x_enu, y_enu, z_enu = float(p.x), float(p.y), float(p.z)
        if self.origin_enu is not None:
            x_enu -= self.origin_enu[0]
            y_enu -= self.origin_enu[1]
            z_enu -= self.origin_enu[2]

        # ENU -> NED: [E,N,U] -> [N,E,D]
        x_n = y_enu # added
        y_e = x_enu
        z_d = -z_enu

        # Rotate XY by yaw_offset so that published NED axes match PX4 NED heading
        x_n, y_e = rot2d(x_n, y_e, self.yaw_offset)
        # after you compute x_n, y_e and t_sample_us



        # Build PX4 VehicleOdometry
        odom = VehicleOdometry()
        now_ns = self.get_clock().now().nanoseconds
        odom.timestamp = int(now_ns // 1000)
        odom.timestamp_sample = int(time_to_us(msg.header.stamp))

        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        nan = float("nan")
        odom.position = [x_n, y_e, z_d if self.publish_z else nan]
        odom.q = [nan, nan, nan, nan]
        odom.velocity = [nan, nan, nan]
        odom.angular_velocity = [nan, nan, nan]
        

        vxy = self.pos_std_xy * self.pos_std_xy
        vz = self.pos_std_z * self.pos_std_z
        odom.position_variance = [vxy, vxy, vz if self.publish_z else 1.0e6]
        odom.velocity_variance = [1.0e6, 1.0e6, 1.0e6]
        odom.orientation_variance = [1.0e6, 1.0e6, 1.0e6]

        odom.reset_counter = int(self.reset_counter)
        odom.quality = int(self.quality_ok)

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = OrbVioToPx4EV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
