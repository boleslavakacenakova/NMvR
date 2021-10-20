import rclpy
import pandas as pd
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String


class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        time_period = 1
        self.timer = self.create_timer(time_period, self.map_load_callback)

    def map_load_callback(self):
        msg = String()
        msg.data = str(pd.read_csv('/home/bolka/dev_ws/src/data.csv', header=None).values).replace(' ', ',')
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()