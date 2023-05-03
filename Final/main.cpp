// Sweep the motor speed from full-speed reverse (-1.0) to full speed forwards (1.0)
#include "mbed.h"
#include "Motor.h"
#include "rtos.h"
#include "VL53L0X.h"
#include "uLCD_4DGL.h"
#include <stdio.h>
#include "Speaker.h"
Serial pc(USBTX, USBRX);
DigitalOut led1(LED1);
uLCD_4DGL uLCD(p9,p10,p11); // serial tx, serial rx, reset pin;
Motor motorA(p21, p22, p23); // pwm, fwd, rev
Motor motorB(p24, p26, p25);
DigitalOut stby(p15);

Speaker speaker(p18);

RawSerial blue(p13,p14);
I2C         i2c(p28, p27);

VL53L0X     vl_sensors[4] = {(&i2c),(&i2c),(&i2c),(&i2c)};
BusOut      vl_shutdown(p16,p17,p19,p20);


int hornCheck = 0;
int alarm = 0;
double L = 200;
double R = 200;
double F = 200;
double B = 200;
Mutex uLCD_mutex;


void turnleft() 
{
    motorB.speed(0.5);
    motorA.speed(-0.5);
}
void turnright()
{
    motorB.speed(-0.5);
    motorA.speed(0.5);
}
void forward()
{
    motorB.speed(0.5);
    motorA.speed(0.5);
}
void backward()
{
    motorB.speed(-0.5);
    motorA.speed(-0.5);
}
void stop()
{
    motorB.speed(0);
    motorA.speed(0);
}
//Bluetooth Thread

void bluetoothReadIn(void const *args) {
    char bnum=0;
    char bhit=0;
    while(1) {
        if (blue.getc()=='!') {
            if (blue.getc()=='B') { //button data
                bnum = blue.getc(); //button number
                bhit = blue.getc();
                switch (bnum) 
                {
                    case '1':
                        
                    case '4':
                    if (bhit=='1') {
                        hornCheck = 1;
                        }
                    else {
                        hornCheck = 0;
                    }
                        break;
                    case '5': //button 5 up arrow
                    if (bhit=='1') {
                        pc.printf("forward");
                        forward();
                        }
                    else {
                        stop();
                    }  
                        break;
                    case '6': //button 6 down arrow
                        if (bhit=='1') {
                        backward();
                        }
                        else {
                        stop();
                        }
                        
                        break;
                    case '7': //button 7 left arrow
                        if (bhit=='1') {
                        turnleft();
                        }
                        else {
                        stop();
                        }
                        break;
                    case '8': //button 8 right arrow
                        if (bhit=='1') {
                        turnright();
                        }
                        else {
                        stop();
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        Thread::wait(10);
    }
}
// Thread Speaker
void horn(void const *argument)
{
    while(1)
    {
        if (hornCheck || alarm)
        {
            speaker.PlayNote(969.0, 0.5, 1.0);
            speaker.PlayNote(800.0, 0.5, 1.0);
        }
        else
        {
            hornCheck = 0;
            alarm = 0;
        }
        hornCheck = 0;
        alarm = 0;
        Thread::wait(10);
    }
}

// Thread Map
void drawmap(void const *argument)
{
    int x1,x2,y1,y2;
    while(true) {
        uLCD_mutex.lock();
        uLCD.cls();
        uLCD.filled_circle(64,64,5,RED);
        //pc.printf("Activated mutex lock on uLCD screen: ulcd2_thread\n\r");
        //uLCD.line(int(64-(L/400)*64),int(64-(F/400)*64),int(64 + (R/400)*64), int(64-(F/400)*64),WHITE);
        
        //uLCD.line(64-10,64-20,64+30,64-20,WHITE);
        //uLCD.line(0,(B/1000)*127,0,127,WHITE);
        //uLCD.rectangle(64-10,64-20,64 +30,64 + 40,WHITE);

        y2 = int(64+(L/400)*64);
        x1= int(64-(R/400)*64);
        x2 = int(64+(F/400)*64);
        y1 = int(64-(B/400)*64);
        uLCD.rectangle(x1,y1,x2,y2,WHITE);
        uLCD_mutex.unlock();
        Thread::wait(300);
        }
    
}
// Thread Distance Sensing

void distanceSensors(void const *argument){
    uint8_t expander_shutdown_mask = 1;
    char str[80];
   for(uint8_t i = 0; i < 4 ; i++)
   {
     vl_shutdown = expander_shutdown_mask;
     expander_shutdown_mask = (expander_shutdown_mask << 1) + 1;
     vl_sensors[i].init();
     vl_sensors[i].setDeviceAddress(0x40 + i);
     vl_sensors[i].setModeContinuous();
     vl_sensors[i].startContinuous();
   }
   uint16_t results[6];
   while(1)
   {
     for(uint8_t i = 0; i < 4 ; i++)
     {
       results[i] = vl_sensors[i].getRangeMillimeters();
       if (results[i] >= 300){
           results[i] = 300;
       }
       if (results[i] <= 100){
           results[i] = 100;
       }
       
       
    }
    if ((results[0] <= 100) && (results[1] <= 100) && (results[2] <= 100) && (results[3] <= 100))
        {
            alarm = 1;
        }
    L = results[0];
    R = results[1];
    F = results[2];
    B = results[3];
    pc.printf("1: %4imm 2: %4imm 3: %4imm) 4: %4imm)\n\r", results[0], results[1], results[2], results[3]);
     
    Thread::wait(1000);
    }
}

// Thread uLCD
int main() {
    stby = 1;
    //speaker.period(1.0/800.0);
    pc.printf("Before horn");
    Thread thread1(horn);
    pc.printf("Before bluetooth");
    Thread thread2(bluetoothReadIn);
    pc.printf("Before distance sensor");
    Thread thread3(distanceSensors);
    pc.printf("Beore uLCD sensor");
    Thread thread4(drawmap);
    //Thread thread4(IMUSensor);
    while(1)
    {
        
        //pc.printf("LED1 = 1");
        led1 = 1;
        Thread::wait(100);
        //pc.printf("LED1 = 0");
        led1 = 0;
        Thread::wait(100);
        
    }
}
