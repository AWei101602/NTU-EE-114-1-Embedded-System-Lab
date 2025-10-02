Task_1: Press the button to trigger LED2 blinking (keep blinking in 5s with 1Hz frequency).

Hint: The EXTI interrupt handler detects the button press and then inform Task_1 via a binary semaphore to blink LED.

Task_2: LED2 blinks (keep blinking in 2s with 10Hz frequency) automatically every 10s.

Hint: The timer interrupt handler detects the timeup event and then inform Task_2 via another binary semaphore to blink LED.

option problem A:
The difference from the basic problem is, you need to identify the long button press event (don't release the button over 1s). When detecting such event, Task_1 executes another blinking procedure (keep blinking in 5s with 10Hz frequency). There are two types of blinking chosen by Task_1 according to long press or normal press, so Task_1 should be informed by distinguishable messages from a queue rather than a binary semaphore.
