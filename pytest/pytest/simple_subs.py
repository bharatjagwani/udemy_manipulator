import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("SimpleSubscriber")
        self.sub = self.create_subscription(String,"Chatter",self.callback,10)
        self.get_logger().info("Subscriber ready to receive messages")

    def callback(self,msg):
        self.get_logger().info("I heard '%s'" %msg.data)

def main():
    rclpy.init()
    simple_sub = SimpleSubscriber()
    rclpy.spin(simple_sub)
    simple_sub.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()