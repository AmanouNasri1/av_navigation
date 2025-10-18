import rclpy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3
from rclpy.qos import QoSProfile

class AVDriver:
    """Webots ROS 2 controller for TeslaModel3."""

    def init(self, webots_node, properties):
        """Called once when Webots starts this controller."""
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        print("[AVDriver] ✅ Initializing Webots ROS2 driver...")
        print(f"[AVDriver] Basic timestep: {self.__timestep} ms", flush=True)

        # Get devices (names must match your .wbt sensors)
        self.__imu   = self.__robot.getDevice('imu')
        self.__gyro  = self.__robot.getDevice('gyro')
        self.__accel = self.__robot.getDevice('accelerometer')
        self.__gps   = self.__robot.getDevice('gps')

        # Enable sensors
        if self.__imu:
            self.__imu.enable(self.__timestep); print("[AVDriver] IMU enabled", flush=True)
        if self.__gyro:
            self.__gyro.enable(self.__timestep); print("[AVDriver] Gyro enabled", flush=True)
        if self.__accel:
            self.__accel.enable(self.__timestep); print("[AVDriver] Accelerometer enabled", flush=True)
        if self.__gps:
            self.__gps.enable(self.__timestep); print("[AVDriver] GPS enabled", flush=True)

        # ROS 2 node
        rclpy.init(args=None)
        self.__node = rclpy.create_node('av_node')
        print("[AVDriver] ROS 2 node 'av_node' created", flush=True)

        qos = QoSProfile(depth=10)
        self.__node.create_subscription(
            AckermannDrive, 'cmd_ackermann', self.__cmd_ackermann_callback, qos)
        print("[AVDriver] Subscribed to /cmd_ackermann", flush=True)

        # Publishers
        self.__imu_pub = self.__node.create_publisher(Imu, '/vehicle/imu/data', qos)
        self.__gps_pub = self.__node.create_publisher(NavSatFix, '/vehicle/gps', qos)
        print("[AVDriver] Publishers created for IMU and GPS", flush=True)

        print("[AVDriver] ✅ Initialization complete. Waiting for step() calls...", flush=True)

    def __cmd_ackermann_callback(self, message):
        """Handle Ackermann drive commands."""
        try:
            self.__robot.setCruisingSpeed(message.speed)
            self.__robot.setSteeringAngle(message.steering_angle)
            print(f"[AVDriver] 🚗 Speed={message.speed:.2f}  Steering={message.steering_angle:.2f}", flush=True)
        except Exception as e:
            print(f"[AVDriver] ⚠️ Error applying command: {e}", flush=True)

    def step(self):
        """Called automatically by Webots each simulation step."""
        print("[AVDriver] 🔁 step() called", flush=True)

        # Process ROS 2 callbacks
        rclpy.spin_once(self.__node, timeout_sec=0)

        # IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.__node.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        if self.__imu:
            q = self.__imu.getQuaternion()
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = q

        if self.__gyro:
            gx, gy, gz = self.__gyro.getValues()
            imu_msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)

        if self.__accel:
            ax, ay, az = self.__accel.getValues()
            imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)

        self.__imu_pub.publish(imu_msg)

        # GPS
        if self.__gps:
            lat, lon, alt = self.__gps.getValues()
            gps_msg = NavSatFix()
            gps_msg.header.stamp = imu_msg.header.stamp
            gps_msg.header.frame_id = 'gps_link'
            gps_msg.latitude  = lat
            gps_msg.longitude = lon
            gps_msg.altitude  = alt
            gps_msg.status.status = 0
            gps_msg.status.service = 1
            self.__gps_pub.publish(gps_msg)