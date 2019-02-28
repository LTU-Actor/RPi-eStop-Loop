# eStop Loop Break Detection

Detect a break in an eStop loop on platforms supported by
[Adafruit-GPIO](https://github.com/adafruit/Adafruit_Python_GPIO)

### Required ROS Params

`~estop_service` sets the `std_srvs/Empty` service to call to trigger an eStop.

`~estop_pin` sets the pin to listen on. It expects a True/High for all-okay and a
False/Low for a eStop. Check with the Adafruit GPIO library for pin numbers and
values.

### Other ROS Params

`~percentage` sets the percentage of readings that can be Low without an eStop
being triggered. This helps deal with noise in exceptionally large loops.
Capacitors between the pin being read and ground can help a lot too. Defaults
to 50%.

`~num_tests` sets the number of times a pin is polled per percentage decision.
Defaults to 15.

`~sleep_time` is the amount of time to sleep between pin tests in milliseconds.
It is meant to help keep CPU usage to a minimum. Defaults to 15.

### Signal Output

If configured, this node will listen to a specified topic of type
`std_msgs/Bool` and output that value to a pin.

`~signal_pin` the pin to set Hi/Low based on what is seen on the topic

`~signal_topic` sets the topic to listen on


### Example launch file

```xml
<launch>
  <node pkg="ltu_actor_rpi_estop_loop" type="run.py" name="rpi_estop_loop">
    <param name="estop_service" value="/estop/stop" />
    <param name="estop_pin" value="2" />
    <param name="signal_pin" value="3" />
    <param name="signal_topic" value="/vehicle_active" />
  </node>
</launch>
```
