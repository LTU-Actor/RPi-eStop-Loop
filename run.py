#!/usr/bin/env python3


import rospy

import os
os.environ['GPIOZERO_PIN_FACTORY'] = os.environ.get('GPIOZERO_PIN_FACTORY', 'mock')

from gpiozero import LineSensor, Button, LED
from gpiozero.pins.pigpio import PiGPIOFactory
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from time import sleep

rospy.init_node('rpi_estop_loop')

hostpi           = rospy.get_param('~host', None)
estop_pin        = rospy.get_param('~estop_pin')
estop_service    = rospy.get_param('~estop_service')
signal_pin       = rospy.get_param('~signal_pin', None)
estop_pin_alt    = rospy.get_param('~estop_pin_alt', None)
threshold        = rospy.get_param('~threshold', 0.5)
sample_rate      = rospy.get_param('~sample_rate', 100)
queue_len        = rospy.get_param('~queue_len', 10)

rospy.wait_for_service(estop_service)
stop = rospy.ServiceProxy(estop_service, Empty)

factory = None
led     = None
alt_in  = None

if hostpi is not None:
    factory = PiGPIOFactory(host=hostpi)

loop = LineSensor(estop_pin, pin_factory=factory, threshold=threshold, sample_rate=sample_rate, queue_len=queue_len)
loop2 = LineSensor(estop_pin_alt, pin_factory=factory, threshold=threshold, sample_rate=sample_rate, queue_len=queue_len)


def do_stop():
    stop()


if signal_pin is not None:
    led = LED(signal_pin, pin_factory=factory)
    loop.when_no_line = led.off
    loop.when_line = led.on

# if estop_pin_alt is not None:
#     rospy.loginfo('setting up button for pin; ' + str(estop_pin_alt))
#     alt_in = Button(estop_pin_alt, pin_factory=factory, bounce_time=0.01, pull_up=True)
#     alt_in.when_pressed = do_stop

rospy.loginfo('estop_loop watching pin ' + str(estop_pin) + ' for broken loop')
rospy.loginfo('estop_pin_alt watching pin ' + str(estop_pin_alt) + ' for broken buttpn')


while not rospy.is_shutdown():
    if not loop.value:
        do_stop()
    if not loop2.value:
        do_stop()
    sleep(0.05)

