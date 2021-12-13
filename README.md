# esp_32
ESP32 Motor Controller &amp; Sensor Data Acquisition

Note: SW should be superseded by MicroROS

## Design
The ESP32 sub-module is crucial to expanding the capabilities of the AGV. GPIO on the Jetson Nano is limited, especially when it comes to IO like UART and PWM. By having a UART connection with the Jetson Nano, the ESP32 can receive serial commands and perform tasks on behalf of the Jetson. This design also improves the real-time constraints of time sensitive tasks and reduces the work load on the Jetson.

## Pinout
|        |ESP       |ESP       |        |
|:------:|:---------|---------:|:------:|
|        |**3V3**   |**GND**   |        |
|        |**EN**    |**GPIO23**|left servo|
|        |**GPIO36**|**GPIO22**|right servo|
|        |**GPIO39**|**TX**    |        |
|        |**GPIO34**|**RX**    |        |
|        |**GPIO35**|**GPIO21**|motor pwm|
|        |**GPIO32**|**GND**   |        |
|        |**GPIO33**|**GPIO19**|motor fwd|
|Encoder B|**GPIO25**|**GPIO18**|motor bck|
|Encoder A|**GPIO26**|**GPIO5** |        |
|        |**GPIO27**|**GPIO17**|Jetson 10(RX)|
|        |**GPIO14**|**GPIO16**|Jetson 8(TX)|
|        |**GPIO12**|**GPIO4** |        |
|        |**GND**   |**GPIO0** |        |
|        |**GPIO13**|**GPIO2** |        |
|*NC*    |**GPIO9** |**GPIO15**|        |
|*NC*    |**GPIO10**|**GPIO8** |        |
|        |**GPIO11**|**GPIO7** |        |
|5V      |**VIN**   |**GPIO6** |        |
