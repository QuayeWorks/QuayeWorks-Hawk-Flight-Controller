The QuayeWorks Hawk Flight Controller by QuayeWorks
 

Rev 0.1A - Created the first revision of the hawk flight controller. Included 2x Atmega328p processor(s), 1x Stm32f103c8t6 processor(s), (MPU6050, BMP180, GY-271) ports via I2C, GPS, NRF2401, SONAR, and SD card expansion.

The 1st revision (0.1A) included 0-6 pins for motor PWM output. A dedicated 48->5v Power converter expansion area (LM2596) for better reliance.  Dedicated USB-TTL IC's (CH340G-C). External Ports for 5v in and out. A buzzer for dedicated

sound. Dedicated 5v LED ports. *Due to improper layout planning, the CH340G/C IC's did not connect to the individual MCU's due to this only the STM32 MCU could be programmed by use of the GPS port and an external USB-TTL converter*

<A failed Design>

 

Rev 2.0A - Created the second revision of the hawk flight controller. Included 2x Stm32f103c8t6 processor(s), (MPU6050, BMP180, GY-271) ports via I2C, GPS, NRF2401, SONAR, and SD card expansion.

The 1st revision (0.2A) included 0-8 pins for motor PWM output. An OSD port for use with external OSD control. A dedicated 48->5v Power converter expansion area (LM2596) for better reliance.  Dedicated USB-TTL IC's (CH340G-C). External Ports for 5v in and out. A buzzer for dedicated sound. Dedicated 3.3v LED ports. The  pcb layout was changed to better incoporate a cleaner, simpler design.*The 0.1A board, The GPS ports and OSD ports can be used to program the respective STM32 IC's if the USB ports should fail with the redesigned CH340G/C IC's*

*The 5.0v ports for the Servo's and Sonar sensors, use a 0.178mm trace. This may cause serious problems when trying to use more than two servos at once. This will be fixed with an update*

<An untested design as of 8/7/2023>

 

Rev 2.1 - An update to the second revision of the hawk flight controller. Includes a trace width increase for the 5.0v ports for the servo and sonar connectors. This dedicated trace of 0.78mm in width should allow for enough current while running all five of the servos at the same time.

<An untested design as of 8/7/2023>

 

Rev 3.0...
