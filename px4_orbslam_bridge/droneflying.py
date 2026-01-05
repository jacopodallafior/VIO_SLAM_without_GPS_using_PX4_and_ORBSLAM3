#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
import math
import time


class droneflying(Node):
    def __init__(self):
        super().__init__('arm_and_takeoff')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # publishers
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos
        )

        # waypoints in local NED (z < 0 is up)
        self.waypoints = [
            # --- PHASE 1: TAKEOFF & STABILIZE ---
            {"x": 0.0,  "y": 0.0,  "z": -5.0, "yaw": math.radians(20)},              # Takeoff to 2m
            {"x": 0.7,  "y": 0.7,  "z": -3.0, "yaw": math.radians(10)}, 
            {"x": -0.7,  "y": -0.7,  "z": -2.0, "yaw": math.radians(-20)}, 
            {"x": 1.4,  "y": -0.7,  "z": -7.0, "yaw": math.radians(10)}, 
            # --- PHASE 2: VERTICAL EXCITATION (The "Bounce") ---
            # Quick rise and fall allows the filter to observe Z-accel bias
            {"x": 15.0,  "y": 0.0,  "z": -5, "yaw": math.radians(0)},              # Go Up
            {"x": 20.0,  "y": 20.0,  "z": -5, "yaw": math.radians(0)},              # Go Down

            # --- PHASE 3: LATERAL & ROTATIONAL EXCITATION (The "Figure 8") ---
            # Moving sideways while rotating allows the camera to triangulate features for scale
            {"x": 30.0,  "y": 15.0, "z": -5.0, "yaw": math.radians(0)}, # Slide Right + Yaw Right
            {"x": 20.0,  "y": 10.4,  "z": -5.0, "yaw": math.radians(0)},  # Slide Left + Yaw Left
            {"x": 10.0,  "y": 0.0,  "z": -5.0, "yaw": math.radians(0)},               # Re-center

            # --- PHASE 4: MAIN TRAJECTORY (The Square) ---
            {"x": 10.0, "y": 0.0,  "z": -5.0, "yaw": math.radians(0)},              # Long straight leg
            {"x": 10.0, "y": 10.0, "z": -3.0, "yaw": math.radians(0)}, # First Corner
            {"x": 0.0,  "y": 10.0, "z": -2.0, "yaw": math.radians(0)},# Second Corner
            {"x": 0.0,  "y": 0.0,  "z": -2.0, "yaw": math.radians(0)},# Return Home
            
            # Final land or hover
            #{"x": 0.0,  "y": 0.0,  "z": -1.0, "yaw": 0.0},
        ]
        self.current_wp = 0

        self.t0 = time.monotonic()

        # timers
        self.create_timer(0.05, self.publish_offboard_mode)
        self.create_timer(0.05, self.publish_setpoint)
        self.arm_timer = self.create_timer(2.0, self.arm)
        self.offboard_timer = self.create_timer(3.0, self.set_offboard_mode)

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.callback_pos,
            qos
        )

    def callback_pos(self, msg: VehicleLocalPosition):
        goal = self.waypoints[self.current_wp]

        d = math.sqrt(
            (msg.x - goal["x"]) ** 2 +
            (msg.y - goal["y"]) ** 2 +
            (msg.z - goal["z"]) ** 2
        )

        self.get_logger().info(
            f"Distance to WP {self.current_wp}: {d:.2f} m",
            throttle_duration_sec=1.0
        )

        if d < 0.20 and self.current_wp < len(self.waypoints) - 1:
            self.current_wp += 1

    def publish_setpoint(self):
        coordinate = self.waypoints[self.current_wp]

        sp = TrajectorySetpoint()
        sp.position[0] = coordinate["x"]
        sp.position[1] = coordinate["y"]
        sp.position[2] = coordinate["z"]
        sp.yaw = coordinate["yaw"]
       
        self.setpoint_pub.publish(sp)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)
        self.get_logger().info("Arm command sent")
        self.arm_timer.cancel()

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.cmd_pub.publish(msg)
        self.get_logger().info("Set OFFBOARD mode command sent")
        self.offboard_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = droneflying()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
