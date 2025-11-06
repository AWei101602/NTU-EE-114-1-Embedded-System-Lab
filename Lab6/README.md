1. (40 points) https://github.com/Egoruch/Internal-Temperature-Sensor-STM32-HAL/tree/main
Please setup ADC (ADC1) triggered by timer (TIM1) to sample the signal of STM32 internal temperature sensor at a fixed frequency and generating an interrupt when each conversion finishes. You have to determine the trigger frequency (explain to TA what the parameters you set), and interrupts should inform the RTOS task to print the sensor values out through UART.
請同學用cubeIDE進行設定，請參考助教 demo 以及網路資源，例如:
https://refcircuit.com/articles/21-get-temperature-from-stm32-internal-temperature-sensor-simple-library.htmlLinks to an external site.
https://github.com/Egoruch/Internal-Temperature-Sensor-STM32-HAL/tree/main

 

2. (40 points) 因為HAL library無法完全實現本題要求(需透過LL library)，提供程式架構供同學參考及修改
We provide an example code in “timer_trigger_adc_DMA_ver” folder to setup ADC triggered by timer, sampling the signal of STM32 internal temperature sensor at a fixed frequency. Further, the code setups DMA to transfer the data from ADC data register to a specific buffer when each conversion finishes. When the top half of buffer is filled, the interrupt will be generated and print all data in the top half of buffer. When the bottom half of buffer is filled, the interrupt makes all data in the bottom half of buffer printed. We left some “to do”s in the code, please finish them.

====option problem_1==== (optional personal report, plus 10 points)
Here is an MbedOS microphone example which can record the sound in two second(https://github.com/janjongboom/b-l475e-iot01a-audio-mbed Links to an external site.). Please port it to the cubeIDE environment running CMSIS-RTOS. Then, modify the code such that the program executes continuously and periodically (You don’t need to preserve audio data or print it out). This code setups DMA to transfer audio data to PCM_Buffer. When the top and bottom half of PCM_Buffer are filled, the corresponding interrupts will generate. Please choose two GPIO pins as output and connect them to logic analyzer. Once the PCM_Buffer top half event occurs, toggle pin1’s output voltage. Once the PCM_Buffer bottom half event occurs, toggle pin2’s output voltage. By this way, you can observe the frequency of audio sampling.
