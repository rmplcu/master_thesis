#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

PUB_RATE = 10
PUB_TOPIC = '/husky1/front/scan'
FRAME_ID = 'husky1_tf/base_link'

if __name__ == '__main__':
    rospy.init_node('fake_laser')

    laser_pub = rospy.Publisher(name=PUB_TOPIC, data_class=LaserScan, queue_size=5)
    #laser_sub = rospy.Subscriber(name='/husky2/scan', data_class=LaserScan, callback=lambda x: print(x))

    empty_scan = LaserScan()
    
    empty_scan.header.frame_id = FRAME_ID
    empty_scan.angle_increment = 0.01745329238474369
    empty_scan.angle_max = -3.14
    empty_scan.angle_min = 3.14
    empty_scan.scan_time = 0.3
    empty_scan.range_max = 13
    empty_scan.range_min = 0.1
    empty_scan.ranges = [float('inf') for _ in range(360)]
    
    assert PUB_RATE != 0

    while not rospy.is_shutdown():
        
        empty_scan.header.stamp = rospy.Time.now()
        empty_scan.header.seq += 1

        laser_pub.publish(empty_scan)

        rospy.sleep(rospy.Duration(1/PUB_RATE))