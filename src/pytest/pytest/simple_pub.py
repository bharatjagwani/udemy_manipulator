import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("SimplePublisher")
        self.pub = self.create_publisher(String,"Chatter",10)
        self.counter = 0
        self.frequency = 1.0
        self.get_logger().info("Publishing at %d Hz" %self.frequency)
        self.timer = self.create_timer(self.frequency,self.timerCallback)
    
    def timerCallback(self):
        msg = String()
        msg.data = "Hello ROS2 - counter: %d" %self.counter
        self.pub.publish(msg)
        self.counter +=1

def main():
    rclpy.init()
    simple_pub = SimplePublisher()
    rclpy.spin(simple_pub)
    simple_pub.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
