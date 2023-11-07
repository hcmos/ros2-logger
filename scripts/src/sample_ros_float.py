import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
import random
import time


class MinimalPublisher(Node):

    def __init__(self, toipc_name: str, interval_sec: float = 1.0):
        super().__init__('minimal_publisher_' + toipc_name)
        self.publisher_ = self.create_publisher(Float64MultiArray, toipc_name, 10)
        timer_period = interval_sec
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        if random.randint(0, 50) == 0:
            self.get_logger().info('Delay occur!!')
            time.sleep(self.timer.timer_period_ns / 1e9 * random.random())

        # msg.data = 'Hello World: %d' % self.i
        for i in range(3):
            msg.data.append(random.uniform(0, 100))
        self.publisher_.publish(msg)


class MinimalSubscriber(Node):

    def __init__(self, toipc_name: str):
        super().__init__('minimal_subscriber_' + toipc_name)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            toipc_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    exec = MultiThreadedExecutor()

    node_list: list[Node] = []
    for i in [100,500]:
        interval_sec = i / 1000
        minimal_publisher = MinimalPublisher(f'logf_{i}_ms', interval_sec)
        minimal_subscriber = MinimalSubscriber(f'logf_{i}_ms')
        exec.add_node(minimal_publisher)
        exec.add_node(minimal_subscriber)
        node_list.append(minimal_publisher)
        node_list.append(minimal_subscriber)

    exec.spin()

    exec.shutdown()
    for node in node_list:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
