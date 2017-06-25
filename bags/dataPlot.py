#! /usr/bin/env python

import rosbag

if __name__ == '__main__':
    bag = rosbag.Bag('myBag.bag')
    
    x_a = [msg.pose.pose.position.x for (topic, msg, t) in bag.read_messages(topics=['/robot0/odom'])]
    y_a = [msg.pose.pose.position.y for (topic, msg, t) in bag.read_messages(topics=['/robot0/odom'])]
    t_t = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/robot0/odom'])]
    
    omega_a = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/omega'])]
    omega_t = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/omega'])]
    nulti = omega_t[0]
    for i in range(0,len(omega_t)):
        omega_t[i] = omega_t[i] -nulti
        
    
    error_a = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/error'])]
    error_t = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/error'])]
    nulti2 = error_t[0]
    for i in range(0,len(error_t)):
        error_t[i] = error_t[i] -nulti2
    
    modul_a = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/modul'])]
    modul_t = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/modul'])]
    nulti2 = modul_t[0]
    for i in range(0,len(modul_t)):
        modul_t[i] = modul_t[i] -nulti2


    kut_a = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/kutjedan'])]
    kut_t = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/kutjedan'])]
    nulti2 = kut_t[0]
    for i in range(0,len(kut_t)):
        kut_t[i] = kut_t[i] -nulti2

    kut_a1 = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/kutpet'])]
    kut_t1 = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/kutpet'])]
    nulti2 = kut_t1[0]
    for i in range(0,len(kut_t1)):
        kut_t1[i] = kut_t1[i] -nulti2

    kut_a2 = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/kutosam'])]
    kut_t2 = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/kutosam'])]
    nulti2 = kut_t2[0]
    for i in range(0,len(kut_t2)):
        kut_t2[i] = kut_t2[i] -nulti2

    kut_a3 = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/kutdamjan'])]
    kut_t3 = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/kutdamjan'])]
    nulti2 = kut_t3[0]
    for i in range(0,len(kut_t3)):
        kut_t3[i] = kut_t3[i] -nulti2

    kut_a4 = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/kutja'])]
    kut_t4 = [t.to_sec() for (topic, msg,t) in bag.read_messages(topics = ['/kutja'])]
    nulti2 = kut_t4[0]
    for i in range(0,len(kut_t4)):
        kut_t4[i] = kut_t4[i] -nulti2

    pathX = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/pathX'])]
    pathY = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/pathY'])]
    
    robotX = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/robotX'])]
    robotY = [msg.data for (topic, msg, t) in bag.read_messages(topics=['/robotY'])]
    
  
   
        
