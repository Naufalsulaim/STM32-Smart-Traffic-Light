#include "mbed.h"
#include "hcsr04.h"
// pins for ultrasound sensor
HCSR04 sensor(PB_4, PB_10);

//  pins for pedestrian button interrupt
InterruptIn pedestrianButton(PA_1);  // Pedestrian button pin (GPIO interrupt)

//USB serial communication
Serial pc(USBTX, USBRX);  // Define Serial object for USB communication

Timer carTimer;  // Timer object to track the duration of the obstacle detection

//  pins for traffic lights
DigitalOut junction1Green(PC_9);  // Green LED for junction 1
DigitalOut junction1Yellow(PB_8);  // Yellow LED for junction 1
DigitalOut junction1Red(PB_9);    // Red LED for junction 1
DigitalOut junction2Green(PA_13);  // Green LED for junction 2
DigitalOut junction2Yellow(PA_14);  // Yellow LED for junction 2
DigitalOut junction2Red(PA_15);    // Red LED for junction 2
DigitalOut PedestrianGreen1(PA_12);  // GreenPedestrian LED for Pedetrian
DigitalOut PedestrianGreen2(PA_11);  // GreenPedetrian LED for Pedestrian
DigitalOut PedestrianRed(PB_12);    // RedPedestrian LED for Pedestrian

//Output for 7 segmment Counter Display
DigitalOut dataIn(PA_9);
DigitalOut serialClk(PA_8);
DigitalOut parallelClk(PB_5);
DigitalOut displayD1(PA_5);
DigitalOut displayD2(PA_6);
DigitalOut displayD3(PA_7);
DigitalOut displayD4(PB_6);

int num[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF};
int seg[4] = {0xF1, 0xF2, 0xF4, 0xF8};

int cnt = 0;
int16_t counter = 0;
int8_t d1, d2, d3, d4;

void LEDs(uint8_t x)
{
    displayD1 = (x & (1 << 0)) ? 1 : 0;
    displayD2 = (x & (1 << 1)) ? 1 : 0;
    displayD3 = (x & (1 << 2)) ? 1 : 0;
    displayD4 = (x & (1 << 3)) ? 1 : 0;
}

void ledcnt(void)
{
    LEDs(cnt);
    wait_ms(500);
    if (++cnt > 15)
        cnt = 0;
}

void displaySeg(uint8_t data, uint8_t seg)
{
    int8_t i;
    uint16_t dataseg;
    dataseg = (data << 8) + seg;

    parallelClk = 0;
    for (i = 15; i >= 0; i--)
    {
        serialClk = 0;
        if (dataseg & (1 << i))
            dataIn = 1;
        else
            dataIn = 0;
        serialClk = 1;
    }
    parallelClk = 1;
    wait_ms(1);
}


// Define variables
bool pedestrianCrossingActive = false;  // Flag to track pedestrian crossing phase


void CarDetection()
{
    long distance = sensor.distance();  // Get the distance from the ultrasound sensor
    pc.printf("Distance: %ld cm\n", distance);
    
    if (distance < 7) {
        pc.printf("Car detected\n\r");
        // Additional code for car detection action
    } else {
        pc.printf("No car detected\n\r");
        // Additional code for no car detection action
    }
    
    wait(1);
}

void TrafficJunction1Off()

    {
    
        pedestrianButton.disable_irq();

        counter = 179;  

        while (counter > 170)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 169;  

        while (counter > 160)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 1;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 159;  

        while (counter > 150)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 149;  

        while (counter > 140)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 1;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 139;  

        while (counter > 130)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 129;  

        while (counter > 120)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 1;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 119;  

        while (counter > 110)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 109;  

        while (counter > 100)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 1;
                junction1Yellow = 0;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        // Junction 1: Yellow for 5 seconds
       
        counter = 99;  

        while (counter > 40)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 1;
                junction1Red = 0;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        //Junction 1 red for 3 sec
        
        counter = 39;  

        while (counter > 10)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }
        pedestrianButton.enable_irq();
    }

void TrafficJunction2On()
       // Junction 2: green for 25 sec  
    {  
        pedestrianCrossingActive = true;

        counter = 380;  

        while (counter > 130)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 1;
                junction2Yellow = 0;
                junction2Red = 0;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }
        
    }

void TrafficJunction2Off()
    {
        pedestrianButton.disable_irq();

        counter = 129;  

        while (counter > 120)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
        
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 0;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;
            }
            counter--;  // Decrement counter
        }

        counter = 119;  

        while (counter > 110)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);

                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 1;
                junction2Yellow = 0;
                junction2Red = 0;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 109;  

        while (counter > 100)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 0;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        counter = 99;  

        while (counter > 90)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 1;
                junction2Yellow = 0;
                junction2Red = 0;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }

        // Junction 2: Yellow for 5 seconds
       
        counter = 89;  

        while (counter > 40)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 1;
                junction2Red = 0;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }


        //Junction 2 red for 3 sec
        
        counter = 39;  

        while (counter > 10)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }


        pedestrianButton.enable_irq();
    }


void PedestrianOn()
  
    {   
        // Pedestrian: Green for 15 seconds
        
        counter = 209;  

        while (counter > 200)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 0;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }


        counter = 199;  

        while (counter > 190)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 189;  

        while (counter > 180)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 179;  

        while (counter > 170)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 0;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 169;  

        while (counter > 160)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 159;  

        while (counter > 150)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 149;  

        while (counter > 140)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 0;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 139;  

        while (counter > 130)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 129;  

        while (counter > 120)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 119;  

        while (counter > 110)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 0;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

       counter = 109;  

        while (counter > 100)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

       counter = 99; 

        while (counter > 90)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 89;  

        while (counter > 80)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 0;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 79;  

        while (counter > 70)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 1;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        counter = 69;  

        while (counter > 60)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 1;
                PedestrianRed = 0;

            }
            counter--;  // Decrement counter
        }

        // Pedestrian: Red for 5 seconds

        counter = 59;  

        while (counter > 10)
        {
            d1 = (counter / 10) % 10;
            d2 = (counter / 100) % 10;
            d3 = (counter / 1000) % 10;
            d4 = (counter / 10000) % 10;
            for (int x = 0; x < 20; x++)
            {
                displaySeg(num[d4], seg[0]);
                displaySeg(num[d3], seg[1]);
                displaySeg(num[d2], seg[2]);
                displaySeg(num[d1], seg[3]);
                
                junction1Green = 0;
                junction1Yellow = 0;
                junction1Red = 1;

                junction2Green = 0;
                junction2Yellow = 0;
                junction2Red = 1;

                PedestrianGreen1 = 0;
                PedestrianGreen2 = 0;
                PedestrianRed = 1;

            }
            counter--;  // Decrement counter
        }
    }

void Pedestrianplay()
{
    if (pedestrianCrossingActive) {
        // Pedestrian button pressed during pedestrian crossing phase at TrafficJunction1
        TrafficJunction2Off();
        PedestrianOn();
    } else {
        // Pedestrian button pressed during normal function run
        TrafficJunction1Off();
        PedestrianOn();
    }
}
void junction2()
{
    TrafficJunction1Off();
    TrafficJunction2On();
    TrafficJunction2Off();
}

void normal(float timer)
{
    pedestrianCrossingActive = false;

    junction1Green = 1;
    junction1Yellow = 0;
    junction1Red = 0;

    junction2Green = 0;
    junction2Yellow = 0;
    junction2Red = 1;

    PedestrianGreen1 = 0;
    PedestrianGreen2 = 0;
    PedestrianRed = 1;

    displaySeg(num[0], seg[0]);
    displaySeg(num[0], seg[1]);
    displaySeg(num[0], seg[2]);
    displaySeg(num[0], seg[3]);

}
void obstacleDetection()
{
    float obstacleTimer = 0.0;  // Variable to track the duration of obstacle detection

    while (1)
    {
        // Check if obstacle is detected
        if (sensor.distance() < 7)
        {
            if (obstacleTimer == 0.0)
            {
                carTimer.reset();  // Reset the timer when the obstacle is first detected
                carTimer.start();  // Start the timer
            }

            // Update the obstacleTimer with the elapsed time
            obstacleTimer = carTimer.read();

            // Check if the obstacle is detected for more than 5 seconds
            if (obstacleTimer >= 5.0)
            {
                junction2();  // Call the junction2 function
                break;  // Exit the obstacle detection loop
            }
        }
        else
        {
            obstacleTimer = 0.0;  // Reset the obstacleTimer if no obstacle is detected
        }

        normal(1000.0);  // Call the normal function

        wait(1.0);  // Delay for 1 second
    }
}

int main()
{
    pedestrianButton.fall(&Pedestrianplay);


    while (1)
    {
        {
            normal(1000);
        }

        CarDetection();
        obstacleDetection();  // Call the obstacleDetection function
    }
}
