# eStop Loop Break Detection

Detect a break in an eStop loop on RaspberryPi. Supportes remote GPIO.

### Required ROS Params

`~estop_service` sets the `std_srvs/Empty` service to call to trigger an eStop.

`~estop_pin` sets the pin to listen on. It expects a True/High for all-okay and a
False/Low for a eStop. Check with the Adafruit GPIO library for pin numbers and
values. **Do NOT use GPIO2/3 as they have soldered-on pull-up resistors!**. The
system needs a pull-down resistor!

### Other ROS Params

`~host` allows specifying the remote GPIO host IP. If not set, assumed local.

`~threshold` Defaults to 0.5. When the average of all loop values in the
internal queue rises above this value, the loop will be considered stoped

`~sample_rate` The number of estop values to read from the device (and append
to the internal queue) per second. Defaults to 100.

### Signal Output

If configured, this node will listen to a specified topic of type

`std_msgs/Bool` and output that value to a pin.

`~signal_pin` the pin to set Hi/Low based on if there is eStop currently active


### Example launch file

```xml
<launch>
  <node pkg="ltu_actor_rpi_estop_loop" type="run.py" name="rpi_estop_loop">
    <param name="estop_service" value="/estop/stop" />
    <param name="estop_pin" value="3" />
    <param name="signal_pin" value="4" />
  </node>
</launch>
```
