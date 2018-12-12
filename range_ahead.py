#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

# Funkcija se poziva svaki puta kada postoji ocitanje na lidaru
# Po jednom za svaku rotaciju skenera

def scan_callback(msg):
  range_ahead = msg.ranges[len(msg.ranges)/2]
  print "range ahead: %0.1f" % range_ahead

# Kreiranje cvora range_ahead
rospy.init_node('range_ahead')

# Deklariranje pretplatnika na temu imena scan i poruke LaserScan
# Za svako ocitanje pozovi funkciju scan_callback
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

# Ne zatvaraj petlju dok ne ocitas ctrl+C na tipkovnici
rospy.spin()
