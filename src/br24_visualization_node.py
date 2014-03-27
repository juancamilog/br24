#!/usr/bin/python
import rospy
import Tkinter as tk
from br24.msg import BR24Scanline
from pyOpenBR24.br24_ui import br24_image_window

if __name__ == '__main__':
    rospy.init_node('br24_viz')
    radar_topic = rospy.get_param('radar_topic','/br24/scanline')


    root = tk.Tk()
    img_window = br24_image_window(root, refresh_period=50)
    brsub = rospy.Subscriber(radar_topic,BR24Scanline, img_window.draw_scanline_ros)
    root.mainloop()

