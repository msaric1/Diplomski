#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Funkcija se poziva svaki puta kada postoji ocitanje
# Spremamo najmanju udaljenost do ocitanog objekta - najbliza prepreka
# Bez obzira  u kojem smjeru se prepreka nalazi
# Potrebna nam je globalna varijabla g_range_ahead kako bi joj mogli pristupiti izvan funkcije

def scan_callback(msg):
	global g_range_ahead
	g_range_ahead=min(msg.ranges)
	
g_range_ahead=1 # za pocetak bilo koja vrijednost
# Deklariranje pretplatnika na temu imena 'scan' klase LaserScan
scan_sub=rospy.Subscriber('scan',LaserScan,scan_callback)
# Deklariranje izdavaca na temu cmd_vel_mux/input/teleop klase Twist
cmd_vel_pub=rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size=1)
# Deklariranje cvora wander
rospy.init_node('wander')

state_change_time=rospy.Time.now()

# kretanje prema naprijed(driving_forward): forward(true) vs. vrti se u mjestu (false)
#   TRUE: dok x sekundi ne prode ili se priblizis prepreci, kreci se prema naprijed
#   FALSE: dok y sekundi ne prode, vrti se u mjestu 
driving_forward=True

rate=rospy.Rate(10)

while not rospy.is_shutdown():
	if driving_forward:
		if(g_range_ahead<0.9 or rospy.Time.now()>state_change_time):
			driving_forward=False
			state_change_time=rospy.Time.now()+rospy.Duration(5)
  # provjeri nalazi li se sto na udaljenosti manjoj od x ili ako je vrijeme koje provodis u mjestu isteklo, onda se pocni vrtiti u mjestu
	else: # ne krecemo se prema naprijed
		if rospy.Time.now()>state_change_time:
			driving_forward=True  # zavrsi s vrtnjom u  mjestu, kreci se prema naprijed
			state_change_time=rospy.Time.now()+rospy.Duration(50)
# Kreiraj praznu/nul Twist() poruku. Napomena: u svakom prolasku petlje kreira se nova poruka
	twist=Twist()
  # Ako se krecemo prema naprijed postavi linearnu i kutnu brzinu
	if driving_forward:
		twist.linear.x=0.5
	else:
		twist.angular.z=0.5
 # Objavi cmd_vel sa zeljenim kretanjem
	cmd_vel_pub.publish(twist)
# Zaustavi petlju na 1/rate sekudi
	rate.sleep()	
