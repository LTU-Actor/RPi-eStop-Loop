# eStop Loop Break Detection

Detect a break in an eStop loop on RaspberryPi. Supportes [remote GPIO](https://gpiozero.readthedocs.io/en/stable/remote_gpio.html).

### Physical setup

Create a voltage loop comming from the rpi's own 3.3v output pin, through any number of interruptor estop switches (normally closed), and back into any GPIO pin. This pin will read in pull-down mode so that any break in the loop, or accidental short to ground will be read as an estop trigger. This node will call the estop service when this is noticed.

### Required ROS Params

`~estop_service` sets the `std_srvs/Empty` service to call to trigger an eStop.

`~estop_pin` sets the pin to listen on. It expects a True/High for all-okay and a
False/Low for a eStop. Check with the Adafruit GPIO library for pin numbers and
values. **Do NOT use GPIO2/3 as they have soldered-on pull-up resistors!**. The
system needs a pull-down resistor!

### Other ROS Params

`~host` allows specifying the remote GPIO host IP. If not set, assumed local.

`~threshold` Defaults to 0.5. When the average of all loop values in the
internal queue rises above this value, the loop will be considered stoped. Multiple low readings over a (short) period are needed to avoid false triggers from noise (estop loops can be very noisy as they often string all through the vehicle).

`~sample_rate` The number of estop values to read from the device (and append
to the internal queue) per second. Defaults to 100.

`~signal_pin` Rhe pin to set Hi/Low based on if there is eStop currently active. If configured, this node will publish to a specified topic of type `std_msgs/Bool` and output that value to a pin. 

### Example launch file

```xml
<launch>
  <node pkg="ltu_actor_rpi_estop_loop" type="run.py" name="rpi_estop_loop">
    <param name="host" value="192.168.0.10" /> <!-- gpiozero remote gpio -->
    <param name="estop_service" value="/estop/stop" /> <!-- the service to call when triggered -->
    <param name="estop_pin" value="3" /> <!-- loop input -->
    <param name="signal_pin" value="4" /> <!-- status output pin -->
  </node>
</launch>
```
