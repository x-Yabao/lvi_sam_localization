#!/usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

import rospy
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA, Float32

env_density = "unknown"

def callback(data):
  global env_density
  env_density = data.data

def show():
  rospy.init_node("panel")
  rospy.Subscriber("info/env_density", String, callback)
  text_pub = rospy.Publisher("panel_tabel", OverlayText, queue_size=1)
  rate = rospy.Rate(100)
  while not rospy.is_shutdown():
    text = OverlayText()
    text.width = 400
    text.height = 1000
    text.left = 10
    text.top = 10
    text.text_size = 12
    text.line_width = 2
    text.font = "DejaVu Sans Mono"
    text.text = """LVI-SAM-LOCALIZATION yabao
    Here is a Panel.
    Enironment density: %s       
    (dense, sparse, open)
    """ % (env_density)
    text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
    text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)
    text_pub.publish(text)
    rate.sleep()

if __name__ == '__main__':
  try:
    show()
  except rospy.ROSInterruptException: 
    pass


