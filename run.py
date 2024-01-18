#!/usr/bin/env python3


import rospy

import os
os.environ['GPIOZERO_PIN_FACTORY'] = os.environ.get('GPIOZERO_PIN_FACTORY', 'mock')

from gpiozero import LineSensor, Button, LED
from gpiozero.pins.pigpio import PiGPIOFactory
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from time import sleep

global factory
global led
led = None
global alt_in

def dbw_enablement_changed_callback(data):
    if data.data == True:
        if signal_pin is not None and factory is not None:
            global led
            if led is None:
                led = LED(signal_pin, pin_factory=factory)
    else:
        led = None

### Main loop:
if __name__ == "__main__":
    rospy.init_node('rpi_estop_loop')

    hostpi           = rospy.get_param('~host', None)
    estop_pin        = rospy.get_param('~estop_pin')
    estop_service    = rospy.get_param('~estop_service')
    signal_pin       = rospy.get_param('~signal_pin', None)
    estop_pin_alt    = rospy.get_param('~estop_pin_alt', None)
    threshold        = rospy.get_param('~threshold', 0.5)
    sample_rate      = rospy.get_param('~sample_rate', 100)
    queue_len        = rospy.get_param('~queue_len', 10)
    dbw_enable_topic = rospy.get_param('~dbw_enabled_topic', None)

    factory = None
    alt_in  = None


    rospy.wait_for_service(estop_service)
    stop = rospy.ServiceProxy(estop_service, Empty)

    rospy.Subscriber(dbw_enable_topic, Bool, dbw_enablement_changed_callback, queue_size=1)

    if hostpi is not None:
        factory = PiGPIOFactory(host=hostpi)

    loop = LineSensor(estop_pin, pin_factory=factory, threshold=threshold, sample_rate=sample_rate, queue_len=queue_len)
    loop2 = LineSensor(estop_pin_alt, pin_factory=factory, threshold=threshold, sample_rate=sample_rate, queue_len=queue_len)


    def do_stop():
        stop()

    rospy.loginfo('estop_loop watching pin ' + str(estop_pin) + ' for broken loop')
    rospy.loginfo('estop_pin_alt watching pin ' + str(estop_pin_alt) + ' for broken buttpn')


    while not rospy.is_shutdown():
        if not loop.value:
            do_stop()
        if not loop2.value:
            do_stop()
        sleep(0.05)

