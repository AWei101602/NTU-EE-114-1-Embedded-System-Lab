STM32Cubeide BLE programming – Peripheral (GATT server) in STM32 IoT node and Central (GATT client) in Python (RPi)

====basic problem====
In this BLE project, treat your STM32 as a server and RPi as a client. You need to build a GATT accelerator service in your STM32 with two main characteristics: (a) the 3-axis acceleration values and (b) the accelerator sampling frequency. Your RPi can assign a sampling frequency for the STM32 accelerator via writing to GATT characteristic_b. Each time when your STM32 samples acceleration data, the GATT characteristic_a is updated by new values and the STM32 notifies the new values to RPi.

In STM32, please meet the requirements with RTOS by creating TASK_BLE and TASK_ACC. TASK_BLE is in charge of the basic BLE event handling for GAP connection, GAP disconnection, GATT write requests, etc. TASK_ACC is responsible for reading the accelerator values with the assigned frequency and notifying RPi.

p.s. the 3-axis acceleration values should be real data sampled from LSM6DSM instead of the simulated values.

=================

====option problem_1==== (plus 5 points)

Use another STM32 board. Replace the RPi client with STM32 client.
