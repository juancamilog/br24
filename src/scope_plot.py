#!/usr/bin/python
import rospy
import numpy as np
from br24.msg import BR24Scanline
from dynamic_reconfigure.msg import Config
from matplotlib import pyplot as plt
from time import time

class scope_viz:
    def __init__(self):
        plt.ion()
        self.line = None
        self.curr_time = time()
        self.angle_increment = 360.0/4096.0
        self.ax = None

    def draw_scanline(self,msg):
        scope = np.power((np.fromstring(msg.scanline_data, dtype=np.uint8)/10.0),1.0)
        scan_radius = 5.0*np.ceil(msg.scan_radius*4.0/5.0)
        r = np.array(range(len(scope)))*scan_radius/len(scope)
        if r[-1] <= 500:
            plt.xlabel("Range (meters)")
        else:
            r = r/1000.0
            plt.xlabel("Range (kms)")

        if self.line is None:
            self.line, = plt.plot(r,scope)
            self.ax = plt.axes()
            plt.autoscale(True)
            plt.show()
        self.line.set_data(r,scope)
        self.ax.relim()
        self.ax.autoscale_view(True,True,False)
        plt.title("Radar scope at a bearing of %f degrees"%(msg.angle*self.angle_increment))

        plt.draw()

    def update_params(self,msg):
        print msg

if __name__ == '__main__':
    scope_v = scope_viz()
    rospy.init_node('br24_scope_viz')
    radar_topic = rospy.get_param('radar_topic','/br24/scanline')
    brsub = rospy.Subscriber(radar_topic,BR24Scanline, scope_v.draw_scanline,queue_size=1)
    parsub = rospy.Subscriber('/br24/parameter_updates',Config, scope_v.update_params,queue_size=1)
    rospy.spin()

