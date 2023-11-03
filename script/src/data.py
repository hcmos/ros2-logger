import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
import time


class Subscriber(Node):

    def __init__(self, topic_name: str, update_cb):
        super().__init__('subscriber' + topic_name.replace('/', '_'))
        # print(topic_name)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            topic_name,
            self.callback,
            100)

        self.subscription  # 未定義の警告を防ぐ

        self.update_cb = update_cb
        self.topic_name = topic_name[5:]    #最初の'/log_'は使わない

    def callback(self, msg):
        data_dict = {}
        for i , data in enumerate(msg.data):
            data_dict[(self.topic_name, i)] = data

        if self.update_cb is not None:
            self.update_cb(data_dict)
            # self.get_logger().info(f'heard: {msg.data}')
