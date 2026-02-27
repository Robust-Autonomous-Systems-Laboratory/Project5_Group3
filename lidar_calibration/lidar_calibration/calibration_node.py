import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
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
        # n     : int   — count of measurements seen so far
        # mean  : float — running mean
        # M2    : float — running sum of squared deviations (for variance)
        # For each new measurement x:
        #   n += 1
        #   delta = x - mean
        #   mean += delta / n
        #   M2 += delta * (x - mean)
        #   variance = M2 / n          (or M2 / (n-1) for sample variance)
        #   sigma_hit = sqrt(variance)

        # TODO: initialize outlier counter
        

        self.sigma_hit = 0.0
        self.bias = 0.0

        # required subscriptions and publishers
        self.create_subscription(LaserScan, "/scan", self.handle_scans, sub_qos)

        # publishes current measurement error: z - z*  (std_msgs/Float64)
        self.range_error_pub = self.create_publisher(Float64, "/calibration/range_error", pub_qos)

        # publishes running statistics (std_msgs/Float64 reused for sigma_hit)
        # TODO: custom message? Array?
        self.statistics_pub = self.create_publisher(Float64, "/calibration/statistics", pub_qos)

    def handle_scans(self, msg: LaserScan):
        # TODO: find the beam index for target_angle
       

        # TODO: compute window indices from angle_window, clamp to valid range
        # half = int((self.angle_window * 0.5) / msg.angle_increment)
        # i0 = max(index - half, 0)
        # i1 = min(index + half, len(msg.ranges) - 1)

        # TODO: extract and filter the window
        # - reject values that are not finite
        # - reject values outside [msg.range_min, msg.range_max]

        # TODO: for each valid measurement z:
        #   1. compute range_error = z - self.target_distance
        #   2. publish range_error on /calibration/range_error
        #   3. update Welford running statistics (n, mean, M2)
        #   4. update sigma_hit = sqrt(M2 / n)
        #   5. increment outlier_count if |z - mean| > 3 * sigma_hit
        #   6. publish sigma_hit on /calibration/statistics

        # TODO: periodically log current estimates, e.g. every 100 scans:
        pass

    # TODO: implement on_shutdown to save results to YAML

    


def main():
    rclpy.init()
    node = LidaCalibration()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
