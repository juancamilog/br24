#!/usr/bin/env python
from pyOpenBR24.br24_driver import br24
import rospy
import time
import zlib
from std_msgs.msg import Header
from br24.msg import BR24Scanline
import math
#from br24.srv import br24_setting
from dynamic_reconfigure.server import Server as dyn_rec_server
from br24.cfg import br24_paramsConfig

class parameter_handler():
    def __init__(self, br):
        self.br = br

    def callback(self, config, level):
        for param in config.keys():
            self.process_param(param,config[param])
        return config

    def process_param(self, param,value):
        br = self.br
        if param == 'radar_on':
            if value: br.start_radar()
            else: br.stop_radar()
        elif param == 'radar_speed':
            if value == 0:
                br.reset_scan_speed()
            else:
                br.increase_scan_speed(value)
        elif param == 'radar_range':
            br.set_radar_range(value)
        elif param == 'interference_reject':
            br.set_interference_rejection(value)
        elif param == 'target_boost':
            br.set_target_boost(value)
        elif param == 'local_interference_filter':
            br.set_local_interference_filter(value)
        elif param == 'gain':
            if value == 0:
                br.set_filters_and_preprocessing('auto_gain')
            else:
                br.set_filters_and_preprocessing('manual_gain', value)
        elif param == 'rain_clutter_filter':
            br.set_filters_and_preprocessing('rain_clutter_manual', value)
        elif param == 'sea_clutter_filter':
            if value == 0:
                br.set_filters_and_preprocessing('sea_clutter_auto')
            else:
                br.set_filters_and_preprocessing('sea_clutter_manual', value)

if __name__ == '__main__':
    rospy.init_node('br24')

    # this is to guarantee that the packets leave on the appropriate interface
    if_ip = rospy.get_param('interface_ip',None)
    br = br24(interface_ip = if_ip)

    #setting up the dynamic reconfigure server
    dyn_rec_params = parameter_handler(br)
    server = dyn_rec_server(br24_paramsConfig,dyn_rec_params.callback)

    br.start()
    brpub = rospy.Publisher('scanline', BR24Scanline)

    # process radar parameters
    dyn_rec_params.process_param('radar_on',rospy.get_param('radar_on',True))
    dyn_rec_params.process_param('radar_speed', rospy.get_param('radar_speed',0))
    dyn_rec_params.process_param('radar_range', rospy.get_param('radar_range',6))
    dyn_rec_params.process_param('interference_reject', rospy.get_param('interference_reject',1))
    dyn_rec_params.process_param('target_boost', rospy.get_param('target_boost',1))
    dyn_rec_params.process_param('local_interference_filter', rospy.get_param('local_interference_filter',1))
    dyn_rec_params.process_param('gain', rospy.get_param('gain',0))
    dyn_rec_params.process_param('rain_clutter_filter', rospy.get_param('rain_clutter_filter',77))
    dyn_rec_params.process_param('sea_clutter_filter', rospy.get_param('sea_clutter_filter',0))

    try:        
        angle_increment = 360.0/4096.0
        teninvsqrt2 = 10.0/math.sqrt(2)
        msg = BR24Scanline()
        msg.header = Header()
        msg.header.frame_id = 'radar'

        #r = rospy.Rate(10000)
        while not rospy.is_shutdown():
            #get a scanline from the queue
            sc = br.get_scanline()
            # build the ros message
            msg.header.stamp = rospy.Time.from_sec(sc['time'])
            msg.header.seq +=1
            msg.angle = sc['angle']#*angle_increment
            msg.scan_radius = sc['scale']#*teninvsqrt2
            msg.scanline_data = sc['data']
            # publish it to the scanline topic
            brpub.publish(msg)
            #r.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("Stopping radar..")
        br.stop_radar()
        br.stop()

    # just being paranoid :)
    br.stop_radar()
    br.stop()
