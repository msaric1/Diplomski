#!/usr/bin/env python

# termios knjiznica nam omogucava pristup ulaznim podacima sa konzole/terminala 

import sys, select, tty, termios
import rospy
from std_msgs.msg import String


if __name__ == '__main__':
#Deklariranje cvora izdavaca na temu naziva keys i poruke tipa String
    key_pub = rospy.Publisher('keys', String, queue_size=1) 
# Kreiranje cvora pod nazivom keyboard_driver
    rospy.init_node("keyboard_driver")
# Postavljanje fiksne frekvencije od 100 Hz
    rate = rospy.Rate(100)

    # Spremanje atributa sa tipkovnice 
    old_attr = termios.tcgetattr(sys.stdin) 
    tty.setcbreak(sys.stdin.fileno())
    print "Izdajem unos znakova sa tipkovnice. \n \nZa upravljanje robotom koristite tipkovnicu na sljedeci nacin: \n \n znak 'w': vozi prema naprijed \n znak 'a': okreci se u lijevu stranu \n znak 'd': okreci se u desnu stranu \n znak 'x': vozi unatrag \n znak 's': zaustavi se \n ------------------------------------ \n Zelim vam ugodnu voznju! :) \n \n \n \n Unesite Ctrl + C za prekid... " 
    
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]: 
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()
    
    # Vracanje atributa sa tipkovnice
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
