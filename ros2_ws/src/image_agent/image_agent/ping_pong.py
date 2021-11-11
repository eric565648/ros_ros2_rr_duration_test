
import rclpy
from rclpy.node import Node
import argparse
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
from scipy.io import savemat
import time

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        print("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    
    return image_opencv

def cv2_to_imgmsg(cv_image):
    
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

class PingPong(Node):
    def __init__(self, machine_type, iter, system, trial) -> None:
        super().__init__(machine_type)

        # parameters
        self.mach_type = machine_type
        self.iter_amount = iter
        self.system = system
        self.trial = trial

        # variable
        self.it = -1
        self.durations = np.array([])

        if self.mach_type == 'ping':
            ## publisher
            self.ping_pub = self.create_publisher(Image,"ping",1)
            ## subscriber
            self.pong_sub = self.create_subscription(Image,"pong",self.pong_cb,1)

            self.init_ping()
        
        elif self.mach_type == 'pong':
            print("pong")
            ## publisher
            self.pong_pub = self.create_publisher(Image,"pong",1)
            ## subscriber
            self.ping_sub = self.create_subscription(Image,"ping",self.ping_cb,1)
    
    def ping_cb(self, msg):
        
        self.pong_pub.publish(msg)
        # print("pong cb")

    def pong_cb(self, msg):
        
        rec_stamp = float(self.get_clock().now().nanoseconds)*1e-9

        if self.it < 0:
            self.it += 1
            msg.header.stamp = self.get_clock().now().to_msg()
            self.ping_pub.publish(msg)
        else:
            dur = rec_stamp - float(rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds)*1e-9
            self.durations = np.append(self.durations,dur)

            self.it += 1

            if self.it >= self.iter_amount:
                print("End ping-pong")
                print("Msg duration Ave:",np.mean(self.durations))
                print("Msg duration Std:",np.std(self.durations))
                mdic = {str(self.system)+'_'+str(self.iter_amount)+'_'+str(self.trial):self.durations}
                savemat("duration_"+str(self.system)+'_ros2_'+str(self.iter_amount)+'_'+str(self.trial)+'.mat', mdic)
                return
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
                self.ping_pub.publish(msg)
    
    def init_ping(self):

        print("Wait for 3 secs")
        time.sleep(3)
        img = cv2.imread("logo.png")
        img = cv2.resize(img, (640,480))
        img_msg = cv2_to_imgmsg(img)
        self.ping_pub.publish(img_msg)
        print("init ping")

def main(args=None):
    parser = argparse.ArgumentParser(description="ros ping-pong time test")
    parser.add_argument("--ping-pong", type=str, help="ping (client) or pong (server)")
    parser.add_argument("--system", type=str, help="ubuntu or windows")
    parser.add_argument("--iter",type=int ,default=1000, help="iterations")
    parser.add_argument("--trial", type=str, help="trials")

    args,_ = parser.parse_known_args()
    # initial a ros2
    rclpy.init()
    
    ping_pong_obj = PingPong(args.ping_pong, args.iter, args.system, args.trial)

    rclpy.spin(ping_pong_obj)

    ping_pong_obj.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()