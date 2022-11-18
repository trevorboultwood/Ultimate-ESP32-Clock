# Ultimate-ESP32-Clock
Combined various pxmatrix related projects into a fabulous clock.

The PxMatrix is a brilliant project which allows us to interface with p2.5 and other displays. More information can be found via https://github.com/2dom/PxMatrix .

The main clock display has been taken from witnessmenow (https://github.com/witnessmenow/WiFi-Tetris-Clock) and have combined this with various effects from the Aurora_Demoaura example from pxmatrix itself. I ave re-implemented the abstraction layer a lot allowing the main loop to be in control of the patterns. Additional patterns have been taken from other projects such as the analog clock from gentschi (https://github.com/gentschi/64x64_analog_clock). I have finally added the wifi manager which allows this to be given to friends and family so they can configure the wifi.

Lastly within the project I have created a case which can house the screen and the esp32 which protects the clock from being damanged. I found that the edge leds are prone to become faulty after a small fall.
