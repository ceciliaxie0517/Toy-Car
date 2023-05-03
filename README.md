# Toy-Car
Sixing Chen & Yan Xie

## Introduction
This project aims to design and develop a small toy car that can be controlled using Bluetooth, with distance sensors to detect nearby obstacles. The car will also have a uLCD screen that displays information about the car's and distance, as well as a speaker that emits alarms or horns when necessary. The project will be implemented using a Microcontroller (LPC1768) and a CPU (Raspberry Pi).

## Project Goals
1. Design and develop a small toy car with Bluetooth control and multiple distance sensors
2. Implement the car's control system using a Microcontroller (LPC1768) and a CPU (Raspberry Pi)
3. Display the car's distance information on a uLCD screen
4. Implement a speaker to emit alarms or horns when necessary
5. Ensure the car is safe and easy to use


## Hardware Setup

### Parts Needed
* 1 mbed LPC1768
* 1 Raspberry Pi Zero
* 1 speaker
* 4 distance sensors VL53L0X 
* 1 uLCD-144-G2
* 1Adafruit Bluefruit LE UART Friend
* 1 motor driver TB6612FNG
* 2 DC motors
* 2 wheels


### Pin Connection
mbed  | Raspberry Pi Zero | Left Motor | Right Motor | Motor Driver | uLCD | Battery1 | Battery2 | speaker | Bluetooth
------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- | -------------
USB serial port  | USB serial port |
&#xfeff;| PWR IN | | | | | Power Supply
&#xfeff;| | | red | A01
&#xfeff;| | | black | A02
&#xfeff;| | red | | B01
&#xfeff;| | black | | B02
VOUT| | | |VCC
GND| | | |GND
&#xfeff;| | | |VM| | |Power Supply
p21| | | |PWMA
p22| | | |A12
p23| | | |A11
p15| | | |STBY
p26| | | |B11
p25| | | |B12
p24| | | |PWMB
p9| | | | |RX
p10| | | | |TX
p11| | | | |RESET
VU| | | | |+5V
GND| | | | |GND
GND| | | | | | | |-
p18| | | | | | | |+
GND| | | | | | | | |CTS
p14| | | | | | | | |TXO
p13| | | | | | | | |RXI
VU| | | | | | | | |VIN
GND| | | | | | | | |GND

mbed | Left Distance Sensor | Right Distance Sensor | Front Distance Sensor | Back Distance Sensor
VOUT | VIN | VIN | VIN | VIN
GND | GND | GND | GND | GND
p28 | SDA | SDA | SDA | SDA
p27 | SCL | SCL | SCL | SCL
p16 | XSHUT
p17 | | XSHUT
p19 | | |XSHUT
p20 | | | |XSHUT


### Block Diagram
![block diagram1](https://user-images.githubusercontent.com/93750274/235618277-cd95d08c-150c-4229-9200-f7bce5920d75.png)

### Breadboard
![捕获1](https://user-images.githubusercontent.com/93750274/235613685-558bae87-7eda-4137-a5b1-72a2be0ca755.PNG)


## Software Setup
1. Import the project into mbed cloud compiler
2. Build the project
3. Push the .bin file

## Demo
[![IMAGE_ALT](https://img.youtube.com/vi/rskGaw2-Giw/0.jpg)](https://www.youtube.com/watch?v=rskGaw2-Giw)
