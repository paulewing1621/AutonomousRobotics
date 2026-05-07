import rclpy
from rclpy.node import Node
import subprocess

from wpa_cli_msgs.msg import Scan,Network
from std_msgs.msg import Float32


class WpaCliPublisher(Node):

    def __init__(self):
        super().__init__('wpa_cli_publisher')
        self.declare_parameter('~/scan_period',5.0)
        self.scan_period = self.get_parameter('~/scan_period').get_parameter_value().double_value
        self.subscriber = self.create_subscription(Float32, '/vrep/signal', 
                self.signal_cb, 1)
        self.publisher = self.create_publisher(Scan, '~/scan', 1)
        self.last_signal = 0
        timer_period = self.scan_period  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def signal_cb(self,msg):
        self.last_signal = msg.data

    def timer_callback(self):
        scan = Scan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.interface = "any" # for now...
        n = Network()
        n.bssid = "00:11:22:33:44:55"
        n.ssid = "treasure"
        n.channel = 42
        n.level = int(100 * self.last_signal)
        n.info = "Simulated"
        scan.networks.append(n)
        self.publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = WpaCliPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
