
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import Image

class CamRecorder(Node):
    def __init__(self):
        super().__init__('cam_recorder')
        self.to_frame = 'cam_frame'
        self.from_frame = 'base_link'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cam_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.cam_sub  # prevent unused variable warning


    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        try:
            t = self.tf_buffer.lookup_transform(
                self.to_frame,
                self.from_frame,
                self.get_clock().now(),
                #msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
            print(t)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not fetch transform: {ex}')
            return

def main():
    rclpy.init()
    print('Initialized node')
    node = CamRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()