Arduino Nano Controlller for VESC and ESC applications


**PLEASE UPDATE YOUR LIBRARY BEFORE CONTINUE**

VESC >>> https://github.com/RollingGecko/VescUartControl/tree/VESC6

RF24 >>> http://tmrh20.github.io/RF24/index.html

U8G2 >>> https://github.com/olikraus/u8g2


Modification from original source at:
  - https://github.com/SolidGeek/nRF24-Esk8-Remote
  - http://www.electric-skateboard.builders/t/simple-3d-printed-nrf-remote-arduino-controlled/


**¤ ADDED**
  * SUPPORT UP TO 3 PROFILE MODE *
  - One controller for 3 electric skateboard profile settings.

**¤ FEATURES**
  * Options control modes
    - SAFETY with a trigger switch (aka Killswitch). *Throttle must be in the neutral position. This is to prevent accident runaway board if throttle is pushed before trigger switch is apply*. Except Braking.
    - NORMAL
    - Cruise Control PID (*Experimental)
    - Beginner (Smoother throttle and braking) (*Experimental)

  * [SETTING for Cruise Controller] Adjustable PID = Kp, Ki, Kd.
  * [SETTING] **Stepper** add linear throttle to the Cruise controller and Beginner.
   * [SETTING] Rate TX-RX (1ms-50ms). By default is 50ms max.
   * [SETTING] Exit Setting screen. No more switching On/Off.

**¤ FIXED**
  - Improving a better AckPayLoads between TX and RX.
  - Changing a few things on the display screen.
  - Sorting out the code. *Arduino Nano capacity mems is **30720** bytes*
  - To Enter a Setting mode now is -> (Hold Trigger Switch + Throttle Down).
    - added [EXIT SETTING] page.

-----------------------------------------------------------------------------
**Discord Builder Channel --> https://discord.gg/SYQup3S**

**Help me afford coffee and sandwich --> https://www.paypal.me/deakbannok**
