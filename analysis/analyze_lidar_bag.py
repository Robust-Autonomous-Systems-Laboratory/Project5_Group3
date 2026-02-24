import os
import rclpy
from rclpy.node import Node
import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class SimpleBagReader(Node):
    def __init__(self, bag_path: str):
        super().__init__("simple_bag_reader")

        if not os.path.exists(bag_path):
            raise FileNotFoundError(f"Bag path does not exist: {bag_path}")

        storage_id = "mcap"
    
        self.reader = rosbag2_py.SequentialReader()

        storage_options = StorageOptions(
            uri=bag_path,
            storage_id=storage_id,
        )

        converter_options = ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        self.reader.open(storage_options, converter_options)

        self.topic_msg_classes = {}
        for tm in self.reader.get_all_topics_and_types():
            try:
                self.topic_msg_classes[tm.name] = get_message(tm.type)
            except Exception as e:
                self.get_logger().warn(
                    f"Cannot import '{tm.type}' for topic '{tm.name}'. Skipping. Error: {e}"
                )

    def read_bag(self,target_angle , angle_window ,topic_filter: str | None = None):
       
        all_vals = []
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()

            if topic_filter and topic != topic_filter:
                continue

            msg_cls = self.topic_msg_classes.get(topic)
            if msg_cls is None:
                continue

            msg = deserialize_message(data, msg_cls)
            index = int((target_angle - msg.angle_min) / msg.angle_increment)

            half = int((angle_window * 0.5) / msg.angle_increment)
            i0 = max(index - half, 0)
            i1 = min(index + half, len(msg.ranges) - 1)

            window_ranges = np.array(msg.ranges[i0:i1+1], dtype=float)

            valid = np.isfinite(window_ranges)
            valid &= (window_ranges >= msg.range_min) & (window_ranges <= msg.range_max)

            all_vals.extend(window_ranges[valid].tolist())

        return np.array(all_vals, dtype=float)


def get_sigma(data, actual):

    mean = round(np.mean(data), 4)
    sigma = round(np.std(data), 4)
    bias = round(mean - actual, 4)


    return mean, sigma, bias


def main(args=None):
    rclpy.init(args=args)
    bag_files = [["../bags/rosbag_0_5m", 0.5],["../bags/rosbag_1m", 1],["../bags/rosbag_2m",2.0]]
    
    results = []

    for bag_path, actual in bag_files:

        node = SimpleBagReader(bag_path)
        target_angle = 0.0 
        angle_window = 0.1

        data = node.read_bag(target_angle,angle_window,topic_filter="/scan")
        mean, sigma, bias = get_sigma(data, actual)

        results.append([actual, mean,bias, sigma, data.size])

        # Histogram
        plt.figure()
        plt.hist(data, bins=50, density=True)
        plt.axvline(actual)
        plt.title(f"Histogram For : {actual} m (N={data.size})")
        plt.xlabel("range (m)")
        plt.ylabel("density")
        outpath = f"figures/hist_{str(actual).replace('.','p')}m.png"
        plt.savefig(outpath, dpi=200)
        plt.close()

    
        node.destroy_node()

    print("\nSummary:")
    print("Distance(m)   Mean(m)     Bias(m)     Sigma_hit(m)   N")
    for d, mean, bias, sigma, n in results:
        print(f"{d:10.2f}   {mean:8.4f}   {bias:8.4f}   {sigma:11.4f}   {n}")

    ds = [r[0] for r in results]
    sigs = [r[3] for r in results]
    plt.figure()
    plt.plot(ds, sigs, marker="o")
    plt.title("Sigma_hit vs Distance")
    plt.xlabel("True distance (m)")
    plt.ylabel("Sigma_hit (m)")
    plt.savefig("figures/sigma_vs_distance.png", dpi=200)
    plt.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
