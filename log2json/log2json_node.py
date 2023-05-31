import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log
from rclpy.time import Time


import json

#  https://docs.ros2.org/foxy/api/rcl_interfaces/msg/Log.html

# octet DEBUG=10
# octet INFO=20
# octet WARN=30
# octet ERROR=40
# octet FATAL=50
# builtin_interfaces/msg/Time stamp
# uint8 level
# string name
# string msg
# string file
# string function
# uint32 line

log_level = {10: 'DEBUG', 
             20: 'INFO', 
             30: 'WARN',
             40: 'ERROR',
             50: 'FATAL'}


class Log2JSONSubscriber(Node):

    def __init__(self):
        super().__init__('log2json')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #print(msg.__slots__)
        json_data = {
            'RealTime'      : self.get_clock().now().nanoseconds,
            'Debug_level'   : log_level[msg.level],
            'Timestamp'     : msg.stamp.sec,
            'Package'       : msg.file,
            'Function'      : msg.function,
            'Line'          : msg.line,
            'Name'          : msg.name,
            'Message'       : msg.msg
        }
        print(json.dumps(json_data, indent=4))
        

def main(args=None):
    rclpy.init(args=args)

    simple_Log2JSONSubscriber = Log2JSONSubscriber()
    # print (minimal_subscriber.get_clock().now())

    rclpy.spin(simple_Log2JSONSubscriber)

    # Destroy the node explicitly
    simple_Log2JSONSubscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
