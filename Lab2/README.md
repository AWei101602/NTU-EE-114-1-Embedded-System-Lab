Basic problem(85 points):

Start/Modify a project in STM32 IoT node to read the sensor value, such as 3D Accelerator or 3D gyro, and send the data (using wifi with http or a tcp protocol) to a Linux(RPi)/Windows/Mac host and Visualize with some kind of GUI tools (such as using Python, https://mode.com/blog/python-data-visualization-libraries/Links to an external site.Links to an external site.)
      http sender example: https://github.com/iot2tangle/STM32_B-L475E-IOT01A/tree/main/http-senderLinks to an external site. 

====option problem_1==== (plus 5 points)

Based on the basic problem, initialize and use the "significant motion detection" features provided in the LSM6DSL BSP library (LSM6DSL will send a signal to the STM32 IoT node via GPIO EXTI interrupt when a significant motion detected). If such event triggers, show on your Linux(RPi)/Windows/Mac host.
