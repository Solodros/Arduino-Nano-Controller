Arduino Nano Controlller for VESC and ESC applications

Modification from original source at:
  - https://github.com/SolidGeek/nRF24-Esk8-Remote
  - http://www.electric-skateboard.builders/t/simple-3d-printed-nrf-remote-arduino-controlled/


**¤ ADDED**
  * Options to a Trigger switch
    - Safety ON/OFF
    - Cruise Mode
  * [Cruise Control]
      - Enable Acceleration/Deacceleration cruise control.
      - Introducing *Cruise Stepper* For a smooth linear speed adjustment.
      - Added Setting for *Cruise Stepper* from 1-10. Default is 1. Higher N° is faster stepping. 
      - Added extra safety features for throttling or braking when is on Cruise mode.
   * [SETTING] Min/Max PWM adjustable analog output between 0-255 for a variety applications.
   * [SETTING] Rate TX-RX (1ms-50ms). By default is 50ms max.
      - The lower N°ms will increase the transmitting rate. But it causes delay to the receiver to process.
      - The higher N°ms will decrease the transmitting rate. Will throw off the throttle responds data.
      - Conclusion "Depends on the code and the Arduino CHIP" for more SPECS: https://www.arduino.cc/en/Products/Compare
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
