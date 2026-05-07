import rclpy
from rclpy.node import Node
import subprocess

from wpa_cli_msgs.msg import Scan,Network


class WpaCliPublisher(Node):

    def __init__(self):
        super().__init__('wpa_cli_publisher')
        self.declare_parameter('~/scan_period',5.0)
        self.declare_parameter('~/ignore_ssid',"")
        self.declare_parameter('~/quiet',False)
        self.declare_parameter('~/command',"/opt/turtlebot_ws/nmcli_scan.sh")
        self.scan_period = self.get_parameter('~/scan_period').get_parameter_value().double_value
        self.quiet = self.get_parameter('~/quiet').get_parameter_value().bool_value
        self.ignore_ssid = self.get_parameter('~/ignore_ssid').get_parameter_value().string_value
        self.ignore_ssid = self.ignore_ssid.split(",")
        self.command = self.get_parameter('~/command').get_parameter_value().string_value
        self.command = [s.encode('utf-8') for s in self.command.split(" ")]
        self.publisher = self.create_publisher(Scan, '~/scan', 1)
        timer_period = self.scan_period  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def scan_equals(self,s1,s2):
        if s1.interface != s2.interface:
            return False
        if len(s1.networks) != len(s2.networks):
            return False
        for n1,n2 in zip(s1.networks,s2.networks):
            if n1.ssid != n2.ssid:
                return False
            if n1.bssid != n2.bssid:
                return False
            if n1.level != n2.level:
                return False
            if n1.frequency != n2.frequency:
                return False
        return True


    def word_index(self,s):
        blank=[" ","\t","\n","\r"]
        l=[]
        in_word=False
        for i,c in enumerate(s):
            if in_word and (c in blank):
                in_word = False
            if not in_word and (c not in blank):
                in_word = True
                l.append(i)
        return l


    def timer_callback(self):
        scan = Scan()
        if not self.quiet:
            self.get_logger().info("Scanning")
        # scan_results = subprocess.check_output([b"nmcli", b"dev",b"wifi",b"list",b"--rescan",b"auto"]).split("\n")
        scan_results = subprocess.check_output(self.command).split(b"\n")
        
        h=scan_results[0].decode("utf-8")
        l = self.word_index(h) + [len(h)]
        scanl = [[s.decode("utf-8")[l[i]:l[i+1]-1].strip() for i in range(len(l[:-1]))] for s in scan_results[1:]]
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.interface = "any" # for now...
        for l in scanl:
            try:
                n = Network()
                if len(l[1])==0:
                    continue
                if l[2] in self.ignore_ssid:
                    continue
                n.bssid = l[1]
                n.ssid = l[2]
                n.channel = int(l[4])
                n.level = int(l[6])
                n.info = l[5] + " " + l[8]
            except:
                continue
            if not self.quiet:
                self.get_logger().info(str(l))
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
