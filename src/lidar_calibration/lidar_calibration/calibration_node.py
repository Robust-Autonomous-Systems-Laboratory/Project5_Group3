import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class LidaCalibration(Node):

    def __init__(self):
        super().__init__("calibration_node")

        #When i didnt use this it was giving me weird errors and it was failing to subscribe to the /scan topic
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # according to the doc we need to accepts these parameters 
        self.declare_parameter("target_distance", 1)
        self.declare_parameter("target_angle", 0)
        self.declare_parameter("angle_window", 0.1)

        self.target_distance = float(self.get_parameter("target_distance").value)
        self.target_angle = float(self.get_parameter("target_angle").value)
        self.angle_window = float(self.get_parameter("angle_window").value)

        self.sigma_hit = 0.0
        self.bias = 0.0
        self.measured_values = np.array([])
        self.meean = 0.0
        self.error = 0.0
        
        ##required subscriptions and publisher accord to the doc
        self.create_subscription(LaserScan, "/scan", self.handle_scans, sub_qos)
        self.range_error_pub = self.create_publisher(Float64, "/calibration/range_error", pub_qos)
        self.statistics_pub =  self.create_publisher(Float64, "/calibration/statistics", pub_qos)

    def handle_scans(self, msg:LaserScan):

        
        index = int((self.target_angle - msg.angle_min) / msg.angle_increment)
        half = int((self.angle_window * 0.5) / msg.angle_increment)
        i0 = max(index - half, 0)
        i1 = min(index + half, len(msg.ranges) - 1)
        window_ranges = np.array(msg.ranges[i0:i1+1], dtype=float)
        valid = np.isfinite(window_ranges)
        valid &= (window_ranges >= msg.range_min) & (window_ranges <= msg.range_max)
        self.measured_values = np.array(window_ranges[valid].tolist(), dtype=float)
        

        self.mean = np.mean(self.measured_values)
        self.sigma_hit = np.std(self.measured_values)
        self.error = self.mean - self.target_distance

        print(self.mean)


    def p_hit(self):
        pass
    def p_short(self):
        pass
    def p_max(self):
        pass
    def p_rand(self):
        pass
    


def main():
    rclpy.init()
    node = LidaCalibration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
