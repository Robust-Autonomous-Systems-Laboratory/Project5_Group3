import os
import rclpy
from rclpy.node import Node

import rosbag2_py
from rosbag2_py import StorageOptions, ConverterOptions

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

    def read_bag(self, topic_filter: str | None = None):
        count = 0
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()

            if topic_filter and topic != topic_filter:
                continue

            msg_cls = self.topic_msg_classes.get(topic)
            if msg_cls is None:
                continue

            msg = deserialize_message(data, msg_cls)
            self.get_logger().info(str(msg.ranges))

            count += 1
            

def main(args=None):
    rclpy.init(args=args)
    bag_path = "../bags/rosbag_1m"
    node = SimpleBagReader(bag_path)

    node.read_bag(topic_filter="/scan")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
