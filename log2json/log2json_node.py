import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Log
from rclpy.time import Time
from std_msgs.msg import String


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


class Log2JsonNode(Node):

    def __init__(self):
        super().__init__('log2json')
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, 'topic', 10)

    def publisher(self, json_message):
        msg = String()
        msg.data = json_message
        self.publisher_.publish(msg)
        print (json_message)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        

    def listener_callback(self, msg):
        #print(msg.__slots__)
        json_data = {
            'realtime'      : self.get_clock().now().nanoseconds,
            'debuglevel'    : log_level[msg.level],
            'timestamp'     : msg.stamp.sec,
            'package'       : msg.file,
            'function'      : msg.function,
            'line'          : msg.line,
            'name'          : msg.name,
            'message'       : msg.msg
        }
        print(json.dumps(json_data, indent=4))
        self.publisher(json.dumps(json_data, indent=4))


    

def main(args=None):
    rclpy.init(args=args)

    simple_Log2JsonNode = Log2JsonNode()
    # print (minimal_subscriber.get_clock().now())

    rclpy.spin(simple_Log2JsonNode)

    # Destroy the node explicitly
    simple_Log2JsonNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
