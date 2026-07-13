"""Publish the Go2 vehicle pose from Fast-LIO's MID360 IMU pose."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .transform_math import compose, invert
from .transform_math import normalize_quaternion, validate_translation


class VehicleOdometryAdapter(Node):
    """Apply the fixed vehicle-to-MID360-IMU transform to Fast-LIO odometry."""

    def __init__(self):
        super().__init__('fast_lio_vehicle_adapter')

        self.declare_parameter('calibration.configured', False)
        self.declare_parameter(
            'vehicle_to_body.translation', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'vehicle_to_body.rotation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('input_odometry_topic', '/fast_lio/odometry_raw')
        self.declare_parameter('output_odometry_topic', '/state_estimation')
        self.declare_parameter('global_frame', 'camera_init')
        self.declare_parameter('fast_lio_body_frame', 'body')
        self.declare_parameter('vehicle_frame', 'vehicle')
        self.declare_parameter('publish_sensor_alias', True)
        self.declare_parameter('sensor_alias_frame', 'sensor')

        if not self.get_parameter('calibration.configured').value:
            raise RuntimeError(
                'MID360 vehicle extrinsic is not configured. Measure '
                'T_vehicle_body, update mid360_vehicle_extrinsic.yaml, and '
                'set calibration.configured=true before running the robot.'
            )

        vehicle_to_body_translation = validate_translation(
            self.get_parameter('vehicle_to_body.translation').value)
        vehicle_to_body_rotation = normalize_quaternion(
            self.get_parameter('vehicle_to_body.rotation_xyzw').value)
        self._body_to_vehicle_translation, self._body_to_vehicle_rotation = (
            invert(vehicle_to_body_translation, vehicle_to_body_rotation)
        )

        self._global_frame = self.get_parameter('global_frame').value
        self._body_frame = self.get_parameter('fast_lio_body_frame').value
        self._vehicle_frame = self.get_parameter('vehicle_frame').value
        input_topic = self.get_parameter('input_odometry_topic').value
        output_topic = self.get_parameter('output_odometry_topic').value

        self._publisher = self.create_publisher(Odometry, output_topic, 20)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._odometry_callback, 20)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self._warned_global_frame = False
        self._warned_body_frame = False

        self.get_logger().info(
            'Converting Fast-LIO odometry %s (%s -> %s) to %s (%s -> %s)'
            % (
                input_topic,
                self._global_frame,
                self._body_frame,
                output_topic,
                self._global_frame,
                self._vehicle_frame,
            )
        )

    def _publish_static_transforms(self):
        transforms = [self._make_transform(
            self._body_frame,
            self._vehicle_frame,
            self._body_to_vehicle_translation,
            self._body_to_vehicle_rotation,
        )]

        if self.get_parameter('publish_sensor_alias').value:
            sensor_frame = self.get_parameter('sensor_alias_frame').value
            if sensor_frame and sensor_frame != self._vehicle_frame:
                transforms.append(self._make_transform(
                    self._vehicle_frame,
                    sensor_frame,
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1.0),
                ))

        self._static_broadcaster.sendTransform(transforms)

    def _make_transform(self, parent, child, translation, rotation):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform

    def _odometry_callback(self, message):
        if message.header.frame_id != self._global_frame:
            if not self._warned_global_frame:
                self.get_logger().warning(
                    'Dropping odometry: expected frame_id %s, received %s'
                    % (self._global_frame, message.header.frame_id)
                )
                self._warned_global_frame = True
            return
        if message.child_frame_id != self._body_frame:
            if not self._warned_body_frame:
                self.get_logger().warning(
                    'Dropping odometry: expected child_frame_id %s, received %s'
                    % (self._body_frame, message.child_frame_id)
                )
                self._warned_body_frame = True
            return

        body_translation = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        body_rotation = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        vehicle_translation, vehicle_rotation = compose(
            body_translation,
            body_rotation,
            self._body_to_vehicle_translation,
            self._body_to_vehicle_rotation,
        )

        output = Odometry()
        output.header = message.header
        output.header.frame_id = self._global_frame
        output.child_frame_id = self._vehicle_frame
        output.pose.pose.position.x = vehicle_translation[0]
        output.pose.pose.position.y = vehicle_translation[1]
        output.pose.pose.position.z = vehicle_translation[2]
        output.pose.pose.orientation.x = vehicle_rotation[0]
        output.pose.pose.orientation.y = vehicle_rotation[1]
        output.pose.pose.orientation.z = vehicle_rotation[2]
        output.pose.pose.orientation.w = vehicle_rotation[3]
        # Fast-LIO's covariance does not model the additional lever-arm
        # transform. Preserve it as the best available estimate; current
        # downstream consumers use only the pose itself.
        output.pose.covariance = message.pose.covariance
        # Fast-LIO currently publishes an empty twist. Do not blindly copy a
        # future body-frame twist under a vehicle child_frame_id: that would
        # require a full adjoint/lever-arm conversion.
        self._publisher.publish(output)


def main(args=None):
    """Run the odometry adapter."""
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleOdometryAdapter()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
