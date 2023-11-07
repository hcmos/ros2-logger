import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import time


class FloatSubscriber(Node):

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
        self.topic_name = topic_name[6:]    #最初の'/logf_'は使わない

    def callback(self, msg):
        data_dict = {}
        for i , data in enumerate(msg.data):
            data_dict[(self.topic_name, i)] = data

        if self.update_cb is not None:
            self.update_cb(data_dict)
            # self.get_logger().info(f'heard: {msg.data}')


class StringSubscriber(Node):

    def __init__(self, topic_name: str, update_cb):
        super().__init__('subscriber' + topic_name.replace('/', '_'))
        # print(topic_name)
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.callback,
            100)

        self.subscription  # 未定義の警告を防ぐ

        self.update_cb = update_cb
        self.topic_name = topic_name[6:]    #最初の'/logs_'は使わない

    def callback(self, msg):
        data_dict = {}
        data_dict[(self.topic_name)] = msg.data

        if self.update_cb is not None:
            self.update_cb(data_dict)
            # self.get_logger().info(f'heard: {msg.data}')

def main(args):
    rclpy.init(args=None)

    exec = MultiThreadedExecutor()

    # log data
    node_list: list[Node] = []

    float_topic_list = [s for s in args.topic_list if s.startswith('/logf')]
    str_topic_list = [s for s in args.topic_list if s.startswith('/logs')]


    for topic in float_topic_list:
        subscriber = FloatSubscriber(topic, args.update_float_cb)
        node_list.append(subscriber)
        exec.add_node(subscriber)

    for topic in str_topic_list:
        subscriber = StringSubscriber(topic, args.update_str_cb)
        node_list.append(subscriber)
        exec.add_node(subscriber)

    exec.spin()

    exec.shutdown()
    for node in node_list:
        node.destroy_node()

    rclpy.shutdown()
