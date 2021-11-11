# ping pong

import time
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *
import numpy as np
import cv2
import sys
import argparse
from datetime import datetime as dt
from scipy.io import savemat

def cv2_to_imgmsg(img):
    img_msg = RRN.NewStructure("experimental.pingpongimage.MsgImage")
    img_msg.width = img.shape[1]
    img_msg.height = img.shape[0]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = img.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height

    return img_msg


class PongTest(object):
    def __init__(self):

        print("pong class")
    
    def RRServiceObjectInit(self, ctx, service_path):
        
        print("pong class connect RR")

        # connect call back
        self.ping.InValueChanged += self.ping_cb

    def ping_cb(self, w, value, t):
        
        self.pong.OutValue = w
        
def run_pong(args):

    with RR.ServerNodeSetup("Pong_Service",2356,argv=sys.argv) as node_setup:

        #Initialize the pong service
        RRN.RegisterServiceTypeFromFile("robdef/experimental.pingpongimage")

        #Create Object
        pong_obj = PongTest()
        #Register service with service name
        RRN.RegisterService("Pong","experimental.pingpongimage.PingPong",pong_obj)

        input("Pong Started. Press any key to quit.")

class PingClient(object):
    def __init__(self, args) -> None:
        super().__init__()

        # parameters
        self.mach_type = args.ping_pong
        self.iter_amount = args.iter
        self.system_config = args.system
        self.trial = args.trial

        # variable
        self.it = -1
        self.durations = np.array([])
        self.last_stamp = None
        
        url='rr+local:///?nodeid=8c94bbcf-f443-4c70-aa18-b8b8e8293a2b&service=Pong'

        self.cli=RRN.ConnectService(url)

        # Connect a wire connection
        self.pong_wire = self.cli.pong.Connect()
        self.ping_wire = self.cli.ping.Connect()

        #Add callback when the wire value change
        self.pong_wire.WireValueChanged += self.pong_cb
    
    def pong_cb(self, w, value, t):
        rec_stamp = dt.now()

        if self.it < 0:
            self.it += 1
            self.last_stamp = dt.now()
            self.ping_wire.OutValue = value
        else:
            dur = rec_stamp - self.last_stamp
            self.durations = np.append(self.durations,dur.total_seconds())

            self.it += 1

            if self.it >= self.iter_amount:
                print("End ping-pong")
                print("Msg duration Ave:",np.mean(self.durations))
                print("Msg duration Std:",np.std(self.durations))
                mdic = {str(self.system_config)+'_'+str(self.iter_amount)+'_'+str(self.trial):self.durations}
                savemat("../duration_"+str(self.system_config)+'_rr_'+str(self.iter_amount)+'_'+str(self.trial)+'.mat', mdic)
                return
            else:
                self.last_stamp = dt.now()
                self.ping_wire.OutValue = value
    
    def init_ping(self):

        print("Wait for 3 secs")
        time.sleep(3)
        img = cv2.imread("../logo.png")
        img = cv2.resize(img, (640,480))
        img_msg = cv2_to_imgmsg(img)
        self.ping_wire.OutValue = img_msg
        print("init ping")

def run_ping(args):

    pcli = PingClient(args)
    RRN.RegisterServiceTypeFromFile("robdef/experimental.pingpongimage")

    pcli.init_ping()

    input("Ping Started. Press any key to quit.")

def main():
    
    parser = argparse.ArgumentParser(description="ros ping-pong time test")
    parser.add_argument("--ping-pong", type=str, help="ping (client) or pong (server)")
    parser.add_argument("--system", type=str, help="ubuntu or windows")
    parser.add_argument("--iter",type=int ,default=1000, help="iterations")
    parser.add_argument("--trial", type=str, help="trials")

    args,_ = parser.parse_known_args()

    if args.ping_pong == 'ping':
        run_ping(args)
    elif args.ping_pong == 'pong':
        run_pong(args)
    else:
        print("Please specify ping or pong")
        return

if __name__ == '__main__':
    main()