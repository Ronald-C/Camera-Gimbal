/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 *          FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 *          @see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include <stdio.h>
#include "LPC17xx.h"
#include "tasks.hpp"
#include "freertos_timer.hpp"
#include "printf_lib.h"
#include "examples/examples.hpp"
//#include "i2cTask.hpp"
#include "lpc_pwm.hpp"
#include "io.hpp"
#include "storage.hpp"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <array>
#include "event_groups.h"

#include "MPU9250.hpp"

PWM PWMOne(PWM::pwm1);
PWM PWMTwo(PWM::pwm2);
PWM PWMThree(PWM::pwm3);
PWM PWMFour(PWM::pwm4);
PWM PWMfFive(PWM::pwm5);
PWM PWMSix(PWM::pwm6);

QueueHandle_t accelSendX = 0;
QueueHandle_t accelSendY = 0;





class servoControl : public scheduler_task
{
    public:
        servoControl(uint8_t priority) :
            scheduler_task("servoControl", 512*16, priority)
    {
           // setRunDuration(1);

    }
        bool init(void){
            //initGPIO();


            return true;
        }
        bool run(void *p)
        {
            TickType_t xLastWakeTime;
            const TickType_t xFrequency = 1;
            xLastWakeTime = xTaskGetTickCount ();
            while(1){
                vTaskDelayUntil( &xLastWakeTime, xFrequency );

                AcX = (accel.getX()*(-1)) - 150;  //-1044 - 1044
                AcY = (accel.getY()*(-1));

                fAcX = AcX * AVG + (fAcX * (1 - AVG));
                fAcY = AcY * AVG + (fAcY * (1 - AVG));

                MAcX = map(fAcX, -1024, 1024, -90, 90);
                MAcY = map(fAcY, -1024, 1024, -90, 90);

                diffX = MAcX - posX;
                diffY = MAcY - posY;

                if(     diffX >  20) posX+=0.5;
                else if(diffX < -20) posX-=0.5;
                if(diffX >  1 ) posX+= 0.1;
                else if(diffX < -1 ) posX-=0.1;

                if(     diffY >  20) posY+=0.5;
                else if(diffY < -20) posY-=0.5;
                 if(diffY >  1 ) posY+= 0.1;
                else if(diffY < -1 ) posY-=0.1;

                //u0_dbg_printf("<posX: %2.2f,comX: %2.2f> | <posY: %2.2f | comY: %2.2f>\n",MAcX, posX,MAcY, posY);

                setX = map(posX, -90, 90, 800, 2200);
                setY = map(posY, -90, 90, 800, 2200);
                PWMTwo.set(setMicro(setX));
                PWMThree.set(setMicro(setY));
            }
            return true;
        }
        ////////////////////////////////////
        float setMicro(int in){
            if(in>=0 && in<=2500){
                return 0.0251*in;
            }
            else return 0;
        }
        float map(float x, float in_min, float in_max, float out_min, float out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        //////////////////////////////////////
    private:
        const float AVG = 0.01;
        float AcX=0, AcY=0, AcZ=0;
        float fAcX=0,fAcY=0,fAcZ=0;
        float MAcX=0,MAcY=0,MAcZ=0;
        float posX = 0, posY = 0; //800 2200
        float diffX = 0, diffY = 0;
        int setX = 1500, setY = 1500;
        Acceleration_Sensor & accel = AS;

};

class AccelTask : public scheduler_task
{
    public:
        AccelTask(uint8_t priority) :
            scheduler_task("AccelTask", 512*16, priority)
    {

    }
        bool init(void){
            return true;
        }
        bool run(void *p)
        {

            return true;
        }
        float map(long x, long in_min, long in_max, long out_min, long out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
    private:
        int16_t MAcX=0,MAcY=0,AcX=0,AcY=0,AcZ=0;
        int fAcX=0,fAcY=0,fAcZ=0;
        float degrees=0;
        int count =0;
        const float AVG = 0.5;
        Acceleration_Sensor & accel = AS;
};




bool hold=1;
int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */

    accelSendX = xQueueCreate( 1, sizeof( int ) );
    accelSendY = xQueueCreate( 1, sizeof( int ) );
    scheduler_add_task(new terminalTask(PRIORITY_CRITICAL));
    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    //scheduler_add_task(new AccelTask(PRIORITY_HIGH));
    scheduler_add_task(new servoControl(PRIORITY_MEDIUM));

    //scheduler_add_task(new MPU9250(PRIORITY_MEDIUM));

    //scheduler_add_task(new setLEDS(PRIORITY_MEDIUM));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
#if 0
    scheduler_add_task(new periodicSchedulerTask());
#endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
#if 0
    scheduler_add_task(new example_io_demo());
#endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
#if 0
    scheduler_add_task(new example_task());
    scheduler_add_task(new example_alarm());
    scheduler_add_task(new example_logger_qset());
    scheduler_add_task(new example_nv_vars());
#endif

    /**
     * Try the rx / tx tasks together to see how they queue data to each other.
     */
#if 0
    scheduler_add_task(new queue_tx());
    scheduler_add_task(new queue_rx());
#endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
#if 0
    scheduler_add_task(new producer());
    scheduler_add_task(new consumer());
#endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
#if 0
    Uart3 &u3 = Uart3::getInstance();
    u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
    scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
#endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
