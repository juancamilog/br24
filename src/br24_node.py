#!/usr/bin/env python
from pyOpenBR24.br24_driver import br24
import rospy
import time
import zlib
from std_msgs.msg import Header
from br24.msg import BR24Scanline
import math
#from br24.srv import br24_setting
#import dynamic_reconfigure.client

def dynamic_reconfigure_callback(config):
    pass

def process_param(param,value,br):
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
    rospy.loginfo("Starting radar..")
    rospy.init_node('br24')

    # this is to guarantee that the packets leave on the appropriate interface
    if_ip = rospy.get_param('interface_ip',None)
    br = br24(interface_ip = if_ip)
    br.start()
    brpub = rospy.Publisher('scanline', BR24Scanline)

    # process radar parameters
    process_param('radar_on',rospy.get_param('radar_on',True),br)
    process_param('radar_speed', rospy.get_param('radar_speed',0),br)
    process_param('radar_range', rospy.get_param('radar_range',6),br)
    process_param('interference_reject', rospy.get_param('interference_reject',1),br)
    process_param('target_boost', rospy.get_param('target_boost',1),br)
    process_param('local_interference_filter', rospy.get_param('local_interference_filter',1),br)
    process_param('gain', rospy.get_param('gain',0),br)
    process_param('rain_clutter_filter', rospy.get_param('rain_clutter_filter',77),br)
    process_param('sea_clutter_filter', rospy.get_param('sea_clutter_filter',0),br)

    angle_increment = 360.0/4096.0
    teninvsqrt2 = 10.0/math.sqrt(2)
    try:        
        last_angle = -1
        start_time = time.time()
        count = 0
        while not rospy.is_shutdown():
            #get a scanline from the queue
            sc = br.get_scanline()

            # build the ros message
            msg = BR24Scanline()
            msg.header = Header()
            msg.header.stamp = rospy.Time.from_sec(sc['time'])
            msg.header.frame_id = 'radar'
            msg.header.seq = count
            msg.status = sc['status']
            msg.index = sc['index']
            msg.angle = sc['angle']*angle_increment
            msg.scan_radius = sc['scale']*teninvsqrt2
            msg.scanline_data = ''.join(sc['data'])

            # publish it to the scanline topic
            brpub.publish(msg)


            curr_angle = sc['angle']*angle_increment
            curr_idx = sc['index']
            #print (sc.index,curr_angle)
            if last_angle > curr_angle:
                # e.g.  360 > 1
                curr_time = time.time()-start_time
                print "finished full scan: %s %s"%(curr_time,last_angle)
                print "processed %d scan lines"%(count)
                print "socket queue size: %d"%(br.data_q.qsize())
                print "scanline queue size: %d"%(br.scan_data_decoder.scanlines.qsize())
                start_time = time.time()
            last_angle = curr_angle
            count+=1

    except KeyboardInterrupt:
        rospy.loginfo("Stopping radar..")
        br.stop_radar()
        br.stop()

    # just being paranoid :)
    br.stop_radar()
    br.stop()
