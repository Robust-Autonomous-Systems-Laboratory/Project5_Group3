import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
from math import sqrt, pi, exp
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

        # TODO: initialize Welford's online algorithm state variables
        self.n = 0
        self.running_mean = 0.0
        self.M2 = 0.0

        # TODO: initialize outlier counter
        self.outlier_count = 0

        self.max_range = 0.0
        self.sigma_hit = 0.0
        self.short_reading = 0.0
        self.measured_values = np.array([])
        self.running_error = 0.0

        self.p_hit = 0.0
        self.p_short = 0.0
        self.p_rand = 0.0
        self.p_max = 0.0
        
        # required subscriptions and publisher accord to the doc
        self.create_subscription(LaserScan, "/scan", self.handle_scans, sub_qos)

        # publishes current measurement error: z - z*  (std_msgs/Float64)
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
        self.measured_values = window_ranges[valid]

        for z in self.measured_values:
            # compute error
            meas_error = float(z - self.target_distance)
            
            # publish error on /calibration/meas_error
            self.range_error_pub.publish(Float64(data=meas_error))

            # update Welford running statistics
            self.n += 1
            delta = z - self.running_mean
            self.running_mean += delta / self.n
            delta2 = z - self.running_mean
            self.M2 += delta * delta2

            # update sigma_hit = sqrt(M2 / n)
            if self.n > 1:
                self.sigma_hit = sqrt(self.M2 / self.n)

            # increment outlier_count if outlier is  > 3 * sigma_hit
            if self.n > 20 and abs(z - self.running_mean) > 3 * self.sigma_hit:
                self.outlier_count += 1

        # publish sigma_hit on /calibration/statistics
        self.statistics_pub.publish(Float64(data=self.sigma_hit))

        self.mean = self.running_mean
        self.error = self.mean - self.target_distance
        self.max_range = msg.range_max

        

        # log curent estimated mean, sigma_hit, and outlier count every 100 scans
        if self.n % 100 == 0:
            self.get_logger().info(f"Scans: {self.n}  Mean: {self.mean:.4f} m  Sigma_hit: {self.sigma_hit:.4f} m  Outliers: {self.outlier_count}")


    def p_hit(self):
        a = 1 / (sqrt( 2 * pi * self.sigma_hit ** 2 ))
        b = exp ( -1 * ( (self.error ** 2) / 2 * self.sigma_hit ** 2 ))
        self.p_hit = a * b

    def p_short(self):
        pass

    def p_max(self):
        
        if(self.mean >= self.max_range):
            self.p_max = 1
        else:
            self.p_max = 0

    def p_rand(self):
        
        self.p_rand = 1 / self.max_range
    
    # TODO: implement on_shutdown to save results to YAML


def main():
    rclpy.init()
    node = LidaCalibration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
