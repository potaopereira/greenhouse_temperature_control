# How to run application

```make```

1. It will create the image
2. It will create the container
3. It will build the project using colcon_build
4. It will call the launch file that starts all the necessary nodes

When you are finished

```make stop_container```

# To generate documentation

```make --directory=doxygen```

# Controlling the Temperature of a Greenhouse

![greenhouse model](images/greenhouse.png)


In order to control the temperature of a greenhouse, we create four nodes:
1. One node for the controller, which will be listening to the temperature sensors topics
2. Two nodes for the two temperature sensors
3. A gui node to monitor the temperature progress and change useful controller parameters

Requirements:
1. If no temperature readings are available, the controller tells the window to be closed

# Changing target temperature

![changing the target temperature](images/target_position_changed.gif)

Notice how the window openness changes throughtout the day, since the heat exchange (due to the sun exposure and the temperature gradient) also changes throughout the day.

# Changing the PWM period

![changing the pwm period](images/change_pwm_period.gif)

When choosing a smaller pwm period, the window set of available positions is smaller, and the command looks more "staircase-like". Controlling the temperature with a smalle PWM period is still possible nonetheless.

# Node Graph

## Without the simulator
![node graph without simulator](images/controller_and_sensors_diagram.png)

## Without the simulator
![node graph with simulator](images/simulator.png)

# Diagram sketch
<!-- ![greenhouse model](images/greenhouse_temperature_control.png) -->
<img src="images/greenhouse_temperature_control.png" width="400"/>