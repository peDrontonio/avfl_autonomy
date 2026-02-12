#!/usr/bin/env python3
"""
NED to ENU Coordinate Frame Converter Node for ArduPilot + ROS 2 Humble

ArduPilot publishes pose data in the NED (North-East-Down) reference frame,
but ROS 2 conventions use the ENU (East-North-Up) reference frame.

This node subscribes to the raw ArduPilot topics and republishes them
with the proper ENU frame conversion.

NED -> ENU conversion:
    Position:
        x_enu =  y_ned   (East  = NED-Y)
        y_enu =  x_ned   (North = NED-X)
        z_enu = z_ned   (Up    = -Down)

    Quaternion:
        q_x_enu =  q_y_ned
        q_y_enu =  q_x_ned
        q_z_enu = q_z_ned
        q_w_enu =  q_w_ned

Subscribes to:
    - /ap/pose/filtered          (PoseStamped, NED)
    - /ap/geopose/filtered       (GeoPoseStamped, NED orientation)
    - /ap/twist/filtered         (TwistStamped, NED)

Publishes to:
    - /ap/pose/filtered/enu      (PoseStamped, ENU)
    - /ap/geopose/filtered/enu   (GeoPoseStamped, ENU orientation)
    - /ap/twist/filtered/enu     (TwistStamped, ENU)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from geographic_msgs.msg import GeoPoseStamped


class NedToEnuNode(Node):
    """
    ROS 2 Node that converts ArduPilot NED data to ROS ENU convention.

    Performs a frame rotation on every incoming message and republishes
    on a parallel ``/enu`` topic so that downstream nodes can subscribe
    to data that already follows the ROS ENU convention.
    """

    def __init__(self):
        super().__init__('ned_to_enu_node')

        # QoS matching ArduPilot DDS bridge (BEST_EFFORT, VOLATILE)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS for republished topics (RELIABLE so downstream nodes
        # don't miss messages if they start slightly later)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Subscribers (raw NED topics) ----
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self._pose_callback,
            qos_sub,
        )

        self.geopose_sub = self.create_subscription(
            GeoPoseStamped,
            '/ap/geopose/filtered',
            self._geopose_callback,
            qos_sub,
        )

        self.twist_sub = self.create_subscription(
            TwistStamped,
            '/ap/twist/filtered',
            self._twist_callback,
            qos_sub,
        )

        # ---- Publishers (converted ENU topics) ----
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/ap/pose/filtered/enu',
            qos_pub,
        )

        self.geopose_pub = self.create_publisher(
            GeoPoseStamped,
            '/ap/geopose/filtered/enu',
            qos_pub,
        )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/ap/twist/filtered/enu',
            qos_pub,
        )

        self.get_logger().info(
            'NED → ENU converter node initialised. '
            'Republishing on /ap/*/filtered/enu topics.'
        )

    # ------------------------------------------------------------------
    # Conversion helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _convert_position_ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
        """Convert a position vector from NED to ENU."""
        x_enu = y_ned       # East  = NED-Y
        y_enu = x_ned       # North = NED-X
        z_enu = z_ned      # Up    = -Down
        return x_enu, y_enu, z_enu

    @staticmethod
    def _convert_quaternion_ned_to_enu(qx_ned: float, qy_ned: float,
                                       qz_ned: float, qw_ned: float):
        """Convert a quaternion from NED to ENU frame.

        The rotation that takes NED body axes to ENU body axes is a
        +90 ° rotation about Z followed by a 180 ° rotation about the
        new X axis. In quaternion form the mapping is:
            q_x_enu =  q_y_ned
            q_y_enu =  q_x_ned
            q_z_enu = -q_z_ned
            q_w_enu =  q_w_ned
        """
        qx_enu = qy_ned
        qy_enu = qx_ned
        qz_enu = qz_ned
        qw_enu = qw_ned
        return qx_enu, qy_enu, qz_enu, qw_enu

    @staticmethod
    def _convert_linear_vel_ned_to_enu(vx_ned: float, vy_ned: float, vz_ned: float):
        """Convert linear velocity from NED to ENU."""
        return vy_ned, vx_ned, vz_ned

    @staticmethod
    def _convert_angular_vel_ned_to_enu(wx_ned: float, wy_ned: float, wz_ned: float):
        """Convert angular velocity from NED to ENU."""
        return wy_ned, wx_ned, wz_ned

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _pose_callback(self, msg: PoseStamped):
        """Receive NED PoseStamped, convert and republish in ENU."""
        enu_msg = PoseStamped()
        enu_msg.header = msg.header

        # Position conversion
        enu_msg.pose.position.x, \
            enu_msg.pose.position.y, \
            enu_msg.pose.position.z = self._convert_position_ned_to_enu(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )

        # Orientation conversion
        enu_msg.pose.orientation.x, \
            enu_msg.pose.orientation.y, \
            enu_msg.pose.orientation.z, \
            enu_msg.pose.orientation.w = self._convert_quaternion_ned_to_enu(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )

        self.pose_pub.publish(enu_msg)

    def _geopose_callback(self, msg: GeoPoseStamped):
        """Receive NED GeoPoseStamped, convert orientation and republish in ENU.

        The lat/lon/alt position is frame-agnostic (geodetic), so only the
        orientation quaternion needs conversion.
        """
        enu_msg = GeoPoseStamped()
        enu_msg.header = msg.header

        # Geographic position is unchanged
        enu_msg.pose.position.latitude = msg.pose.position.latitude
        enu_msg.pose.position.longitude = msg.pose.position.longitude
        enu_msg.pose.position.altitude = msg.pose.position.altitude

        # Orientation conversion
        enu_msg.pose.orientation.x, \
            enu_msg.pose.orientation.y, \
            enu_msg.pose.orientation.z, \
            enu_msg.pose.orientation.w = self._convert_quaternion_ned_to_enu(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )

        self.geopose_pub.publish(enu_msg)

    def _twist_callback(self, msg: TwistStamped):
        """Receive NED TwistStamped, convert and republish in ENU."""
        enu_msg = TwistStamped()
        enu_msg.header = msg.header

        # Linear velocity conversion
        enu_msg.twist.linear.x, \
            enu_msg.twist.linear.y, \
            enu_msg.twist.linear.z = self._convert_linear_vel_ned_to_enu(
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            )

        # Angular velocity conversion
        enu_msg.twist.angular.x, \
            enu_msg.twist.angular.y, \
            enu_msg.twist.angular.z = self._convert_angular_vel_ned_to_enu(
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            )

        self.twist_pub.publish(enu_msg)


def main(args=None):
    """Entry point for the NED-to-ENU converter node."""
    rclpy.init(args=args)
    node = NedToEnuNode()

    try:
        node.get_logger().info('NED → ENU converter node spinning...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NED → ENU converter node...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
