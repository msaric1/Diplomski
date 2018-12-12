#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String 
from geometry_msgs.msg import Twist

key_mapping = { 'w': [ 0, 1], 'x': [ 0, -1],
                'a': [ 1, 0], 'd': [-1,  0],
                's':[0,0]}

g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_send_time = None
g_vel_scales = [0.1, 0.1] # pocetno na vrlo sporo
g_vel_ramps = [1, 1] # jedinica: metar po sekundi

# Ovo je kljucna funkcija. Obzirom na trenutno i prethodno vrijeme kada je pozvana funkcija, te obzirom na trenutnu i ciljanu vrijednost, izracunaj i azuriraj vrijednost (ili postigni cilj)
# Ova funkcija se koristi za obje brzine: linearnu i kutnu brzinu
def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate): # izracunaj maksimalni korak brzine 
    # korak = koliki je korak potrebno poduzeti, obzirom koliko je vremena proslo od zadnjeg poziva funkcije
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1.0 if (v_target > v_prev) else -1.0
    error = math.fabs(v_target - v_prev) # math.fabs() vraca apsolutnu vrijednost argumenta
    
    # do cilja mozemo stici unutar ovog vremenskog koraka, zavrsili smo
    if error < step: 
        return v_target
    else:
        return v_prev + sign * step # poduzmi korak prema cilju

# Vrati Twist objekt ispunjen novim vrijednostima temeljenim na proteklo vrijeme i zeljenu stopu promjene
def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
    return tw

# Posalji Twist za ovaj prolazak petljom
def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist, g_vel_scales, g_vel_ramps, g_twist_pub

    # Obzirom na dano novo vrijeme, kreiraj Twist za slanje uzimajuci u obzir stopu promjene
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist, g_last_twist_send_time, t_now, g_vel_ramps)

    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)

def keys_cb(msg):
    global g_target_twist, g_last_twist, g_vel_scales
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return # nepoznat kljuc
    vels = key_mapping[msg.data[0]]
    # Ciljane vrijednosti su azurirane u globalnoj varijabli. One se koriste za racunanje aktualne Twist poruke koja se salje u svakoj petlji
    g_target_twist.angular.z = vels[0] * g_vel_scales[0]
    g_target_twist.linear.x = vels[1] * g_vel_scales[1]

# Koristimo ROS posluzitelj parametara kako bi dohvatili parametre. Radi se provjera i na komandnoj liniji i na posluzitelju parametara. Postoje 4 parametra. Napomena: jedan od argumenata funkcije "default" zadaje se ako neki parametar nije isporucen
def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parametar [%s] nije definiran. Primjenjujem default vrijednost na %.3f" % (name, default)
    return default

if __name__ == '__main__':
    rospy.init_node('keys_to_twist')
    g_last_twist_send_time = rospy.Time.now()
    g_twist_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    
    rospy.Subscriber('keys', String, keys_cb)

    # Prikupi parametre
    g_target_twist = Twist() # inicijaliziraj na 0
    g_last_twist = Twist()
    g_vel_scales[0] = fetch_param('~angular_scale', 0.5)
    g_vel_scales[1] = fetch_param('~linear_scale', 0.5)
    g_vel_ramps[0] = fetch_param('~angular_accel', 1.0)
    g_vel_ramps[1] = fetch_param('~linear_accel', 1.0)

# Petlja se vrti 20 puta u sekundi. Napomena: pozivamo funkciju send_twist() jer se globalne vrijednosti (koje koristimo za izracunavanje potrebne stvarne linearne i kutne brzine) azuriraju kada "dohvate" kljucnu temu 
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()
        
