Arduino Nano Controlller for VESC and ESC applications

**PLEASE UPDATE YOUR LIBRARY BEFORE CONTINUE**
VESC >>> https://github.com/RollingGecko/VescUartControl/tree/VESC6
RF24 >>> http://tmrh20.github.io/RF24/index.html
U8G2 >>> https://github.com/olikraus/u8g2


Modification from original source at:
  - https://github.com/SolidGeek/nRF24-Esk8-Remote
  - http://www.electric-skateboard.builders/t/simple-3d-printed-nrf-remote-arduino-controlled/


**¤ ADDED**
  * Options to a Trigger switch
    - Safety ON/OFF
    - Cruise Mode
  * [Cruise Control] Based on throttle output not PID.
      - Enable Acceleration/Deacceleration cruise control.
      - Introducing *Cruise Stepper* For a smooth linear speed adjustment.
      - Added Setting for *Cruise Stepper* from 1-10. Default is 1. Higher N° is faster stepping. 
      - Added extra safety features for throttling or braking when is on Cruise mode.
   * [SETTING] PWM adjustable analog output between 0/2300 [MIN/MAX] for a variety applications.
   * [SETTING] Rate TX-RX (1ms-50ms). By default is 50ms max.
   * [SETTING] Exit Setting screen. No more switching On/Off.
      
**¤ FIXED**
  - Improving a better AckPayLoads between TX and RX with a data variable comparison.
  - Changing a few things on the display screen.
  - Sorting out the code. *Arduino Nano capacity mems is **30720** bytes*
  - To Enter a Setting mode now is -> (Hold Trigger Switch + Throttle Down).
    - added [EXIT SETTING] page.

-----------------------------------------------------------------------------
**Discord Builder Channel --> https://discord.gg/SYQup3S**

**Help me afford coffee and sandwich --> https://www.paypal.me/deakbannok**
