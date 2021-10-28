#!/usr/bin/env python3

import rospy
import argparse
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys
from scipy.io import savemat

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

class PingPong(object):
    def __init__(self, machine_type, iter, system, trial) -> None:
        super().__init__()

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
            self.ping_pub = rospy.Publisher("ping",Image,queue_size=1)
            ## subscriber
            self.pong_sub = rospy.Subscriber("pong",Image,self.pong_cb,queue_size=1)

            self.init_ping()
        
        elif self.mach_type == 'pong':
            print("pong")
            ## publisher
            self.pong_pub = rospy.Publisher("pong",Image,queue_size=1)
            ## subscriber
            self.ping_sub = rospy.Subscriber("ping",Image,self.ping_cb,queue_size=1)
    
    def ping_cb(self, msg):
        
        self.pong_pub.publish(msg)
        print("pong cb")

    def pong_cb(self, msg):
        
        rec_stamp = rospy.Time.now()

        if self.it < 0:
            self.it += 1
            msg.header.stamp = rospy.Time.now()
            self.ping_pub.publish(msg)
        else:
            print("in else")
            dur = rec_stamp - msg.header.stamp
            self.durations = np.append(self.durations,dur.to_sec())

            self.it += 1

            if self.it >= self.iter_amount:
                print("End ping-pong")
                mdic = {str(self.system)+'_'+str(self.iter_amount)+'_'+str(self.trial):self.durations}
                savemat("duration_"+str(self.system)+'_'+str(self.iter_amount)+'_'+str(self.trial)+'.mat', mdic)
                return
            else:
                msg.header.stamp = rospy.Time.now()
                self.ping_pub.pubish(msg)
    
    def init_ping(self):

        img = cv2.imread("logo.png")
        img = cv2.resize(img, (1200,900))
        img_msg = cv2_to_imgmsg(img)
        self.ping_pub.publish(img_msg)
        print("init ping")

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="ros ping-pong time test")
    parser.add_argument("--ping-pong", type=str, help="ping (client) or pong (server)")
    parser.add_argument("--system", type=str, help="ubuntu or windows")
    parser.add_argument("--iter",type=int ,default=1000, help="iterations")
    parser.add_argument("--trial", type=str, help="trials")

    args,_ = parser.parse_known_args()
    
    rospy.init_node(args.ping_pong)
    ping_pong_obj = PingPong(args.ping_pong, args.iter, args.system, args.trial)

    rospy.spin()