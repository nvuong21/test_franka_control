#!/usr/bin/env python
import os
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  publish_rate = rospy.get_param('~publish_rate', 1.0)
  frame_id = rospy.get_param('~frame_id', 'base_link')
  ip_address = rospy.get_param('~ip_address', '192.168.0.12')
  pub = rospy.Publisher('/ft_sensor/diagnostics', DiagnosticArray, queue_size=3)
  msg = DiagnosticArray()
  msg.header.frame_id = frame_id
  status = DiagnosticStatus()
  status.level = DiagnosticStatus.OK
  status.name = 'NetFT RDT Driver'
  status.message = 'OK'
  status.hardware_id = 'ATI Gamma'
  status.values.append(KeyValue('IP Address', ip_address))
  msg.status.append(status)
  rate = rospy.Rate(publish_rate)
  while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rate.sleep()
