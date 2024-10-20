// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <vector>
// Our own resources
#if defined(TESTBED)
#   include "project/TESTBED.h"
#else
#   include "project/CORE.h"
#endif
#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraCAN.h"

using namespace std;

//Setting up for CAN0 line
AstraCAN Can0;

//AstraMotors(AstraCAN* setCanObject, int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Motor1(&Can0, 2, 1, false, 50, 1.00F);  // Front Left
AstraMotors Motor2(&Can0, 4, 1, false, 50, 1.00F);  // Back Left
AstraMotors Motor3(&Can0, 1, 1, true, 50, 1.00F);  // Front Right
AstraMotors Motor4(&Can0, 3, 1, true, 50, 1.00F);  // Back Right

AstraMotors motorList[4] = {Motor1, Motor2, Motor3, Motor4};//Left motors first, Right motors Second

// Use Serial when using directly with Laptop, use Serial1 when using over UART with main ESP32
// Purposefully override TESTBED.h for motor controller mcu for testing
#define COMMS_UART Serial


//Prototypes

void turnCW();
void turnCCW();
void Stop();
void Brake(bool enable);
void goForwards(float speed);
void goBackwards(float speed);
void loopHeartbeats();
void safety_timeout();


unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;
unsigned long lastFeedback = 0;
unsigned long lastCtrlCmd;
unsigned long clockTimer = millis();
unsigned long heartBeatNum = 1;
uint32_t lastBlink = 0;
bool ledState = false;
String feedback;


void setup() 
{
    //-----------------//
    // Initialize Pins //
    //-----------------//

    pinMode(LED_BUILTIN, OUTPUT);
    Serial1.begin(SERIAL_BAUD);
    Serial.begin(SERIAL_BAUD);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // Initalization for using CAN with the sparkmax
    /* Old code for reference
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    */

    // Setup CAN
    if (Can0.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) 
    {
        COMMS_UART.println("CAN bus started!");
    } 
    else 
    {
        COMMS_UART.println("CAN bus failed!");
    }

}


//------------//
// Begin Loop //
//------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//

void loop() 
{
    // Blink the LED
    if (millis() - lastBlink >= 1000) 
    {
        lastBlink = millis();
        ledState = !ledState;
        if (ledState)
            digitalWrite(LED_BUILTIN, HIGH);
        else
            digitalWrite(LED_BUILTIN, LOW);
    }

    // Every 50 milliseconds, update the speed for all motors
    //safety_timeout();

    // Accelerate the motors
    if (millis()-lastAccel >= 50)
    {
        lastAccel = millis();
        for (int i = 0; i < 4; i++)
        {
            motorList[i].accelerate();
        }
  
    }

    //----------------------------------//
    // send heartbeat                   //
    //----------------------------------//

    if ((millis()-lastFeedback)>=3)
    {
        sendHeartbeat(Can0, heartBeatNum);
        lastFeedback = millis();
        heartBeatNum++;
        if (heartBeatNum > 4)
        {
            heartBeatNum = 1;
        }
    }

    // Send identify command to all motors
    /*if((millis()-clockTimer)>=1000)
    {
        identifyDevice(Can0, 1);
        clockTimer = millis();
    }*/


    //------------------//
    // Command Receiving //
    //------------------//
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////    //\\        ////    //////////      //
    //    //             //  \\    //  //    //        //    //
    //    //             //    \\//    //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //      /////////    //            //    //////////      //
    //                                                       //
    //-------------------------------------------------------//
    //
    // The giant CMD helps with finding this place
    //
    // Commands will be received as a comma separated value string
    // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
    // The program parses the string so that each piece of data can be used individually
    // For examples of parsing data you can use the link below
    // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

    if (COMMS_UART.available()) 
    {

        String command = COMMS_UART.readStringUntil('\n');  // Command is equal to a line in the Serial1
        command.trim();
// How to do this???????????
#if COMMS_UART == Serial1
            Serial.println(command);
#endif
        String prevCommand;  // Shouldn't this be static???

        std::vector<String> args = {}; 
        parseInput(command, args, ',');


        if (args[0] == "ping") 
        {
            COMMS_UART.println("pong");
        } 
        else if (args[0] == "time") 
        {
            COMMS_UART.println(millis());
        }

        else if (args[0] == "ctrl") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {   

            //Serial1.println("ctrl cmd received");
            lastCtrlCmd = millis();
            if (command != prevCommand)
            {

                //Serial1.println("NEW COMMAND RECEIVED");

                prevCommand = command;

                motorList[0].setDuty(args[1].toFloat());
                motorList[1].setDuty(args[1].toFloat());

                motorList[2].setDuty(args[2].toFloat());
                motorList[3].setDuty(args[2].toFloat());
        
            }
            else
            {
                //pass if control command is same as previous
            }

        }

        else if (args[0] == "brake") 
        {

            if (args[1] == "on") 
            {
                Brake(true);
            }

            else if (args[1] == "off")
            {
                Brake(false);
            }

        }

        else if (args[0] == "auto") // Commands for autonomy
        { 

            lastCtrlCmd = millis();
            if (command != prevCommand)
            {

                if (args[1] == "forwards") // auto,forwards
                {  
                    goForwards(args[2].toFloat());
                }

                else if (args[1] == "backwards") // auto,backwards
                { 
                    goBackwards(args[2].toFloat());
                }

                else if (args[1] == "TurnCW") // auto,backwards
                { 
                    turnCW();
                }

                else if (args[1] == "TurnCCW") // auto,backwards
                { 
                    turnCCW();
                }

                else if (args[1] == "stop") // auto,stop
                {  
                    Stop();
                }

            }
            else
            {
                //pass if control command is same as previous
            }

        }

    }

}

//-------------------------------------------------------//
//                                                       //
//    ///////////    //\\          //      //////////    //
//    //             //  \\        //    //              //
//    //             //    \\      //    //              //
//    //////         //      \\    //    //              //
//    //             //        \\  //    //              //
//    //             //          \\//    //              //
//    //             //           \//      //////////    //
//                                                       //
//-------------------------------------------------------//

void safety_timeout()
{
    if (millis() - lastCtrlCmd > 2000)  // if no control commands are received for 2 seconds
    {
        lastCtrlCmd = millis();  // just update the var so this only runs every 2 seconds.
        Stop();
        COMMS_UART.println("No Control, Safety Timeout");
    }
}

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW()
{
    for (int i = 0; i < 4; i++)
        motorList[i].sendDuty(0.6);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW()
{
    for (int i = 0; i < 4; i++)
        motorList[i].sendDuty(-0.6);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop()
{
    for (int i = 0; i < 4; i++)
        motorList[i].sendDuty(0.0);
}

// Enables or disables brake mode for all motors
void Brake(bool enable) {
    for (int i = 0; i < 4; i++)
        motorList[i].setBrake(enable);
}

// Tells the rover to go forwards
// Does not bypass acceleration
// Autonomy
void goForwards(float speed)
{
    for (int i = 0; i < 4; i++ )
        motorList[i].setDuty(speed);
}

// Tells the rover to go backwards
// Does not bypass acceleration
// Autonomy
void goBackwards(float speed)
{
    float temp = (-1) * speed;
    for (int i = 0; i < 4; i++ )
        motorList[i].setDuty(temp);
}
