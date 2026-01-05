import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,  QoSDurabilityPolicy

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, Imu


class GzOrbslamDriver(Node):
    def __init__(self):
        super().__init__('gz_orbslam_driver')
        qos_imu_out = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2000,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        qos_img = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        qos_imu_in = qos_profile_sensor_data

        # ---------- Params ----------
        self.declare_parameter(
            'image_in_topic',
            '/world/baylands/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
        )
        self.declare_parameter(
            'imu_in_topic',
            '/world/baylands/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu'
        )
        self.declare_parameter('config_name', 'GZ_x500_mono_inertial')

        self.image_in_topic = self.get_parameter('image_in_topic').value
        self.imu_in_topic = self.get_parameter('imu_in_topic').value
        self.config_name = self.get_parameter('config_name').value

        # ---------- Pub verso C++ (topic attesi dal common.cpp) ----------
        self.pub_settings = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        self.pub_img = self.create_publisher(Image, '/mono_py_driver/img_msg', qos_img)
        self.pub_ts = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)
        self.pub_imu = self.create_publisher(Imu, '/mono_py_driver/imu', qos_imu_out)

        # ---------- Handshake ----------
        self.got_ack = False
        self.sub_ack = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self._ack_cb, 10
        )

        # Pubblica config finché non arriva ACK
        self.timer = self.create_timer(0.5, self._publish_settings)

        # ---------- Sub da Gazebo (bridged) ----------
        self.sub_img_in = self.create_subscription(
            Image, self.image_in_topic, self._img_cb, qos_img
        )
        self.sub_imu_in = self.create_subscription(
            Imu, self.imu_in_topic, self._imu_cb, qos_imu_in
        )

        self.get_logger().info(f"Input image topic: {self.image_in_topic}")
        self.get_logger().info(f"Input imu topic:   {self.imu_in_topic}")
        self.get_logger().info(f"Config name (YAML without extension): {self.config_name}")

    def _publish_settings(self):
        if self.got_ack:
            return
        msg = String()
        msg.data = self.config_name
        self.pub_settings.publish(msg)

    def _ack_cb(self, msg: String):
        if msg.data.strip().upper() == "ACK":
            self.got_ack = True
            self.get_logger().info("Handshake ACK received from C++ node. Starting forwarding data.")
    '''
    def _img_cb(self, msg: Image):
        if not self.got_ack:
            return

        self.pub_img.publish(msg)

        t = Float64()
        t.data = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.pub_ts.publish(t)

    def _imu_cb(self, msg: Imu):
        if not self.got_ack:
            return
        self.pub_imu.publish(msg)
    '''

    def _imu_cb(self, msg: Imu):
        self.pub_imu.publish(msg)       # no ACK gating

    def _img_cb(self, msg: Image):
        if not self.got_ack:
            return
        self.pub_img.publish(msg)

        # Forward IMU as-is (stamp già coerente con /clock se bridge correttamente)
        


def main():
    rclpy.init()
    node = GzOrbslamDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

'''
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image


class GzOrbslamDriver(Node):
    def __init__(self):
        super().__init__('gz_orbslam_driver')

        self.declare_parameter(
            'image_in_topic',
            '/world/baylands/model/x500_mono_cam_0/link/camera_link/sensor/imager/image'
        )
        self.declare_parameter('config_name', 'GZ_x500')

        self.image_in_topic = self.get_parameter('image_in_topic').value
        self.config_name = self.get_parameter('config_name').value

        # Topics expected by your C++ node
        self.pub_settings = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        self.pub_img = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.pub_ts = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 10)

        self.got_ack = False
        self.sub_ack = self.create_subscription(
            String, '/mono_py_driver/exp_settings_ack', self._ack_cb, 10
        )

        # Keep publishing settings until ACK arrives
        self.timer = self.create_timer(0.5, self._publish_settings)

        # Subscribe to Gazebo image and forward it
        self.sub_img_in = self.create_subscription(
            Image, self.image_in_topic, self._img_cb, qos_profile_sensor_data
        )

        self.get_logger().info(f"Input image topic: {self.image_in_topic}")
        self.get_logger().info(f"Config name (YAML without extension): {self.config_name}")

    def _publish_settings(self):
        if self.got_ack:
            return
        msg = String()
        msg.data = self.config_name  # C++ will load <config_name>.yaml
        self.pub_settings.publish(msg)

    def _ack_cb(self, msg: String):
        if msg.data.strip().upper() == "ACK":
            self.got_ack = True
            self.get_logger().info("Handshake ACK received from C++ node.")

    def _img_cb(self, msg: Image):
        # Don't send images/timestamps before C++ initializes ORB-SLAM3
        if not self.got_ack:
            return

        self.pub_img.publish(msg)

        t = Float64()
        t.data = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.pub_ts.publish(t)



def main():
    rclpy.init()
    node = GzOrbslamDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

'''