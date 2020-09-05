## PlatformIO framework for the LPC176x MCU series
This framework is part of the custom [PlatformIO platform](https://github.com/p3p/pio-nxplpc-arduino-lpc176x)
that [MarlinFirmware](https://github.com/MarlinFirmware/Marlin) uses to build for the LPC176x MCU.

The framework aims to eventually be Arduino API compatible, though Arduino libraries will probably not work,
this is a work in progress with most basic functionality available.

### Hardware Serial Ports

Without any #define, the pins are selected as with "no define".
If your board uses other these default pins for RX/TX, set the appropriate #define(s) as in the tables below.

| Serial | TX | RX | FUNCNUM |
| --- | --- | --- | --- |
| no define | P0_02 | P0_03 | 1 |
 

| Serial1 | TX | RX | FUNCNUM |
| --- | --- | --- | --- |
| no define | P0_15 | P0_16 | 1 | 
| UART1_P2_00 | P2_00 | P2_01 | 2 | 

| Serial2 | TX | RX | FUNCNUM |
| --- | --- | --- | --- |
| no define | P0_10 | P0_11 | 1 | 
| UART2_P2_08 | P2_08 | P2_09 | 2 |  

| Serial3 | TX | RX | FUNCNUM |
| --- | --- | --- | --- |
| no define | P0_00 | P0_01 | 2 |
| UART3_P0_25 | P0_25 | P0_26 | 3 |
| UART3_P4_28 | P4_28 | P4_29 | 3 |

### AnalogRead  
There are 8 ADC channels available  

| Pin | Channel Number|
| --- | --- |
| P0_02  | 7 | 
| P0_03 | 6 | 
| P0_23 | 0 | 
| P0_24 | 1 |   
| P0_25  | 2 | 
| P0_26 | 3 | 
| P1_30 | 4 | 
| P1_31 | 5 |   

### AnalogWrite
Although all pins can be used for PWM (software mode) there are 6 Hardware PWM channels, if a hardware channel is
available for a pin but it is in use by another pin it will fall back into software mode, All channels
software and hardware share a period (default 20ms).

| Hardware channel | Attached pins |
| --- | --- |
| 1 | P1_18, P2_00  |
| 2 | P1_20, P2_01, P3_25 | 
| 3 | P1_21, P2_02, P3_26 | 
| 4 | P1_23, P2_03 | 
| 5 | P1_24, P2_04 | 
| 6 | P1_26, P2_05 |  
