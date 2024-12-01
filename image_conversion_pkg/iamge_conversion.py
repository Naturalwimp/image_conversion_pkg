import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
import cv2

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')

        # Parameters
        self.declare_parameter('input_topic', 'camera1/image_raw')
        self.declare_parameter('output_topic', '/image_converted')
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Variables
        self.mode = 2  # Default mode: Color
        self.bridge = CvBridge()

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, self.output_topic, 10)

        # Service
        self.srv = self.create_service(SetBool, 'set_mode', self.set_mode_callback)

        self.get_logger().info('Image Conversion Node Initialized')

    def set_mode_callback(self, request, response):
        if request.data:
            self.mode = 1  # Greyscale
            response.message = "Mode set to Greyscale"
        else:
            self.mode = 2  # Color
            response.message = "Mode set to Color"
        response.success = True
        self.get_logger().info(response.message)
        return response

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.mode == 1:  # Greyscale
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                converted_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
            else:  # Color
                converted_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            converted_msg.header = msg.header
            self.publisher.publish(converted_msg)

            cv2.namedWindow("Output Image", cv2.WINDOW_NORMAL)
            cv2.imshow("Output Image", cv_image)

            # Display the image
            if self.mode == 1:
                # self.get_logger().info("Displaying image")
                cv2.imshow("Output Image (Greyscale)", cv_image)
                # self.get_logger().info("Image displayed")
            else:
                # self.get_logger().info("Displaying image")
                cv2.imshow("Output Image (Color)", cv_image)
                # self.get_logger().info("Image displayed")
            
            # Wait for a key event for 1ms to process GUI events
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def destroy_node(self):
        # Close any OpenCV windows when shutting down
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
