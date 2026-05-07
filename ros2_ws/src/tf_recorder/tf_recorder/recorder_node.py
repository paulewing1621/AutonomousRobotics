import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('tf_recorder')

        # Declare and acquire `reference_frame` parameter
        self.reference_frame = self.declare_parameter(
          'reference_frame', 'map').get_parameter_value().string_value
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value
        self.output_file = self.declare_parameter(
          'output_file', '/tmp/traj.csv').get_parameter_value().string_value
        self.recording_period = self.declare_parameter(
          'recording_period', 1.0).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Path, '~/path', 1)
        self.path = Path()
        self.path.header.frame_id = self.reference_frame

        # Call on_timer function every second
        self.timer = self.create_timer(self.recording_period,self.on_timer)

        self.srv = self.create_service(Empty, '~/clear', self.on_clear_cb)
        self.srv = self.create_service(Empty, '~/write', self.on_write_cb)


    def on_clear_cb(self, request, response):
        self.path = Path()
        self.get_logger().info('Cleared recorded path')
        return response

    def on_write_cb(self, request, response):
        with open(self.output_file,"w") as f:
            f.write("#X,Y,Z, QW,QX,QY,QZ\n")
            for ps in self.path.poses:
                p=ps.pose
                f.write("%f,%f,%f, %f,%f,%f,%f\n"%(p.position.x,p.position.y,p.position.z,
                    p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z))
        self.get_logger().info('Written recorded path to \'%s\''%self.output_file)
        return response

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.reference_frame
        to_frame_rel = self.target_frame

        try:
            t = self.tf_buffer.lookup_transform(
                    from_frame_rel,
                    to_frame_rel,
                    rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                    f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        p = PoseStamped()
        p.header = t.header
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = t.transform.translation.z
        p.pose.orientation = t.transform.rotation

        self.path.poses.append(p)
        self.path.header.stamp = t.header.stamp

        self.publisher.publish(self.path)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
