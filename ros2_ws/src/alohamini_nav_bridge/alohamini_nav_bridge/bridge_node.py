import json
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import zmq


@dataclass
class BodyVelocity:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return value
    return max(-limit, min(limit, value))


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class AlohaMiniNavBridge(Node):
    def __init__(self):
        super().__init__("alohamini_nav_bridge")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("cmd_port", 5555)
        self.declare_parameter("obs_port", 5556)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("obs_timeout_sec", 0.5)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_lateral_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("linear_x_scale", 1.0)
        self.declare_parameter("linear_y_scale", 1.0)
        self.declare_parameter("angular_z_scale", 1.0)
        self.declare_parameter("swap_xy", False)
        self.declare_parameter("require_observation_for_motion", True)
        self.declare_parameter("pose_covariance_xy", 0.10)
        self.declare_parameter("pose_covariance_yaw", 0.25)
        self.declare_parameter("twist_covariance_xy", 0.20)
        self.declare_parameter("twist_covariance_yaw", 0.35)

        self.host = self.get_parameter("host").value
        self.cmd_port = int(self.get_parameter("cmd_port").value)
        self.obs_port = int(self.get_parameter("obs_port").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = as_bool(self.get_parameter("publish_tf").value)
        self.cmd_timeout_sec = float(self.get_parameter("cmd_timeout_sec").value)
        self.obs_timeout_sec = float(self.get_parameter("obs_timeout_sec").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_lateral_speed = float(self.get_parameter("max_lateral_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_x_scale = float(self.get_parameter("linear_x_scale").value)
        self.linear_y_scale = float(self.get_parameter("linear_y_scale").value)
        self.angular_z_scale = float(self.get_parameter("angular_z_scale").value)
        self.swap_xy = as_bool(self.get_parameter("swap_xy").value)
        self.require_observation_for_motion = as_bool(self.get_parameter("require_observation_for_motion").value)
        self.pose_covariance_xy = float(self.get_parameter("pose_covariance_xy").value)
        self.pose_covariance_yaw = float(self.get_parameter("pose_covariance_yaw").value)
        self.twist_covariance_xy = float(self.get_parameter("twist_covariance_xy").value)
        self.twist_covariance_yaw = float(self.get_parameter("twist_covariance_yaw").value)

        self.zmq_context = zmq.Context()
        self.cmd_socket = self.zmq_context.socket(zmq.PUSH)
        self.cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.cmd_socket.connect(f"tcp://{self.host}:{self.cmd_port}")

        self.obs_socket = self.zmq_context.socket(zmq.PULL)
        self.obs_socket.setsockopt(zmq.CONFLATE, 1)
        self.obs_socket.connect(f"tcp://{self.host}:{self.obs_port}")

        self.cmd = BodyVelocity()
        self.observed = BodyVelocity()
        self.last_cmd_time = self.get_clock().now()
        self.last_obs_time = None
        self.last_integrate_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)

        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"Connected ZMQ host tcp://{self.host}:{self.cmd_port}/{self.obs_port}; "
            f"subscribing {cmd_vel_topic}, publishing {self.odom_topic}"
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        ros_x = clamp(float(msg.linear.x), self.max_linear_speed) * self.linear_x_scale
        ros_y = clamp(float(msg.linear.y), self.max_lateral_speed) * self.linear_y_scale
        vx, vy = (ros_y, ros_x) if self.swap_xy else (ros_x, ros_y)
        wz = clamp(float(msg.angular.z), self.max_angular_speed) * self.angular_z_scale
        self.cmd = BodyVelocity(x=vx, y=vy, yaw=wz)
        self.last_cmd_time = self.get_clock().now()

    def on_timer(self) -> None:
        now = self.get_clock().now()
        self.poll_observation(now)
        self.integrate(now)
        self.publish_odom(now)
        self.send_command(now)

    def poll_observation(self, now) -> None:
        latest = None
        while True:
            try:
                latest = self.obs_socket.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except zmq.ZMQError as exc:
                self.get_logger().warning(f"Observation socket error: {exc}")
                break

        if latest is None:
            return

        try:
            data = json.loads(latest)
            host_x = float(data.get("x.vel", 0.0))
            host_y = float(data.get("y.vel", 0.0))
            ros_x, ros_y = (host_y, host_x) if self.swap_xy else (host_x, host_y)
            self.observed = BodyVelocity(
                x=ros_x * self.linear_x_scale,
                y=ros_y * self.linear_y_scale,
                yaw=math.radians(float(data.get("theta.vel", 0.0))) * self.angular_z_scale,
            )
            self.last_obs_time = now
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Bad observation JSON: {exc}")

    def integrate(self, now) -> None:
        dt = (now - self.last_integrate_time).nanoseconds * 1e-9
        self.last_integrate_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        velocity = self.current_observed_velocity(now)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        self.x += (velocity.x * cos_yaw - velocity.y * sin_yaw) * dt
        self.y += (velocity.x * sin_yaw + velocity.y * cos_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + velocity.yaw * dt), math.cos(self.yaw + velocity.yaw * dt))

    def has_fresh_observation(self, now) -> bool:
        if self.last_obs_time is None:
            return False
        age = (now - self.last_obs_time).nanoseconds * 1e-9
        return age <= self.obs_timeout_sec

    def current_observed_velocity(self, now) -> BodyVelocity:
        if not self.has_fresh_observation(now):
            return BodyVelocity()
        return self.observed

    def publish_odom(self, now) -> None:
        velocity = self.current_observed_velocity(now)
        stamp = now.to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = velocity.x
        odom.twist.twist.linear.y = velocity.y
        odom.twist.twist.angular.z = velocity.yaw
        odom.pose.covariance[0] = self.pose_covariance_xy
        odom.pose.covariance[7] = self.pose_covariance_xy
        odom.pose.covariance[35] = self.pose_covariance_yaw
        odom.twist.covariance[0] = self.twist_covariance_xy
        odom.twist.covariance[7] = self.twist_covariance_xy
        odom.twist.covariance[35] = self.twist_covariance_yaw
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(transform)

    def send_command(self, now) -> None:
        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        obs_ready = self.has_fresh_observation(now) or not self.require_observation_for_motion
        command = self.cmd if cmd_age <= self.cmd_timeout_sec and obs_ready else BodyVelocity()
        payload = {
            "x.vel": command.x,
            "y.vel": command.y,
            "theta.vel": math.degrees(command.yaw),
        }
        try:
            self.cmd_socket.send_string(json.dumps(payload), flags=zmq.NOBLOCK)
        except zmq.Again:
            pass
        except zmq.ZMQError as exc:
            self.get_logger().warning(f"Command socket error: {exc}")

    def destroy_node(self):
        try:
            self.cmd_socket.send_string(
                json.dumps({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}),
                flags=zmq.NOBLOCK,
            )
            self.cmd_socket.close(linger=0)
            self.obs_socket.close(linger=0)
            self.zmq_context.term()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlohaMiniNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

