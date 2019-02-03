#!/usr/bin/env python

import rospy
import Adafruit_GPIO
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from time import sleep

rospy.init_node('estop_loop')

estop_pin = rospy.get_param('~estop_pin')
estop_service = rospy.get_param('~estop_service')
percentage_estop = rospy.get_param('~percentage', 0.5)
num_tests = rospy.get_param('~num_tests', 50)
sleep_time = int(rospy.get_param('~sleep_time', 15))

rospy.wait_for_service(estop_service)
stop = rospy.ServiceProxy(estop_service, Empty)

signal_pin = rospy.get_param('~signal_pin', None)
signal_topic = rospy.get_param('~signal_topic', None)

gpio = Adafruit_GPIO.get_platform_gpio()
gpio.setup(estop_pin, Adafruit_GPIO.IN)


def signal_cb(data):
    gpio.output(signal_pin, data.data)


if signal_pin is not None and signal_topic is not None:
    gpio.setup(signal_pin, Adafruit_GPIO.OUT)
    rospy.Subscriber(signal_topic, Bool, signal_cb)

rospy.loginfo('estop_loop watching pin ' + str(estop_pin) + ' for broken loop')

while not rospy.is_shutdown():
    num_hi = 0
    for _ in range(num_tests):
        if gpio.input(1):
            num_hi = num_hi + 1
        sleep(sleep_time)
    if (num_hi / float(num_tests)) < percentage_estop:
        rospy.logerr_throttle(60, 'eStop triggered from broken loop')
        stop()
