#!/usr/bin/env python

import rospy
from gpiozero import LineSensor, LED
from gpiozero.pins.pigpio import PiGPIOFactory
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from signal import pause

rospy.init_node('rpi_estop_loop')

hostpi           = rospy.get_param('~host', None)
estop_pin        = rospy.get_param('~estop_pin')
estop_service    = rospy.get_param('~estop_service')
signal_pin       = rospy.get_param('~signal_pin', None)
threshold        = rospy.get_param('~threshold', 0.5)
sample_rate      = rospy.get_param('~sample_rate', 100)

rospy.wait_for_service(estop_service)
stop = rospy.ServiceProxy(estop_service, Empty)

factory = None
led     = None

if hostpi is not None:
    factory = PiGPIOFactory(host=hostpi)

loop = LineSensor(estop_pin, pin_factory=factory, threshold=threshold, sample_rate=sample_rate, pull_up=False)


def do_stop():
    stop()
    if led is not None:
        led.on()


if signal_pin is not None:
    led = LED(signal_pin, pin_factory=factory)
    loop.when_no_line = led.off

loop.when_line = do_stop
rospy.loginfo('estop_loop watching pin ' + str(estop_pin) + ' for broken loop')
pause()
