/**
 * @file 00std_rrdv_robot.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @brief This is an example application for Ultrasound ranging with HC-SR04 sensor and nRF52840.
 * This app receives a radio packet from the gateway, parse the packet and sets up the Timer offset for triggering the US ranging.
 * The US readings are then transmitted back to the gateway by radio using Long Range BLE.
 *
 * @copyright Inria, 2022
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nrf.h"

//============================ defines ==========================================

// defines for the robot ID
#define DEVICE_ID_MASK   0xffffffffffff
#define MOTE_ID          ((((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) + (uint64_t)(NRF_FICR->DEVICEADDR[0])) & DEVICE_ID_MASK)
#define nRF1             0x041d8f7b3eb9
#define nRF2             0x0e519397b037
#define nRF3             0xf744aa09cc95
#define nRF4             0x71f564980c7d
#define nRF5             0x000000000000

// radio
#define NUMBER_OF_BYTES_RECEIVED 3UL
#define NUMBER_OF_BYTES_TO_SEND  3UL

// timer
#define cmdDur        //TODO
#define maxCmdDur     //TODO
#define triggerOffset //TODO
#define triggerDur    0.01 //in ms
#define echoDuration  180  //in ms
#define tdmaTimeSlot  3    //TODO in ms 
#define tdmaDelayTx(id) (tdmaTimeSlot*id) //TODO  
#define robotTxOffset //TODO
#define wdTx          //TODO
#define notifDur      //TODO
#define ifsDur        0.01 //TODO in ms
#define maxNotifDur   //TODO

// US sensor
// GPIOTE for turning ON US sensor and READING the range measurement 
#define US_ON_PORT              1UL        // output port number
#define US_ON_PIN               10UL       // output pin number
#define US_READ_PORT            1UL        
#define US_READ_PIN             6UL
#define US_ON_CH                0UL
#define US_READ_CH_LoToHi       1UL
#define US_READ_CH_HiToLo       2UL

//=========================== typedefs ========================================

typedef enum {
    S_RADIORXLISTEN   = 1UL,
    S_RADIORX         = 2UL,
    S_USPREPARE       = 3UL,
    S_TRIGGER         = 4UL,
    S_WAITECHOHIGH    = 5UL,
    S_WAITECHOLOW     = 6UL,
    S_ECHODONE        = 7UL,
    S_RADIOTXDELAY    = 8UL,
    S_RADIOTXPREPARE  = 9UL,
    S_RADIOTXREADY    = 10UL,
    S_RADIOTX         = 11UL,
    S_IFS             = 12UL
} robot_state_t;

typedef struct {
    //robot
    uint8_t       robot_id;
    uint8_t       tdmaDelayTx;
    robot_state_t state;
    //radio
    uint8_t       dataToSendRadio[NUMBER_OF_BYTES_TO_SEND];
    uint8_t       dataReceivedRadio[NUMBER_OF_BYTES_RECEIVED];
    //us sensor
    uint32_t      timeEchoLowHigh;
    uint32_t      timeEchoHighLow;
    uint16_t      usEchoDuration;
} robot_vars_t;  

                           
//=========================== prototypes ========================================

void activity_ri0(void);

void activity_ri1(void);

void activity_rie0(void);

void activity_ri2(void);

void activity_ri3(void);

void activity_ri4(uint32_t capturedTime);

void activity_ri5(uint32_t capturedTime);

void activity_ri6(void);

void activity_ri7(void);

void activity_rie1(void);

void activity_ri8(void);

void activity_rie2(void);

void activity_ri9(void);

void activity_ri10(void);

void changeState(robot_state_t newstate);

//=========================== variables =========================================

robot_vars_t robot_vars;

//============================= main ============================================
int main(void) {
    
    memset(&robot_vars, 0, sizeof(robot_vars_t));

    if      (MOTE_ID == nRF1) {robot_vars.robot_id = 0x01;}
    else if (MOTE_ID == nRF2) {robot_vars.robot_id = 0x02;}
    else if (MOTE_ID == nRF3) {robot_vars.robot_id = 0x03;}
    else if (MOTE_ID == nRF4) {robot_vars.robot_id = 0x04;}
    else if (MOTE_ID == nRF5) {robot_vars.robot_id = 0x05;}

    robot_vars.tdmaDelayTx = tdmaDelayTx(robot_vars.robot_id);

    printf("%d\n", robot_vars.robot_id);
    
    while (1);
}

//============================= ISRs ============================================

/**
@brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/

void TIMER0_IRQHandler(void){

    if((NRF_TIMER0->EVENTS_COMPARE[0] != 0) && ((NRF_TIMER0->INTENSET & TIMER_INTENSET_COMPARE0_Msk) != 0))
    {   
        NRF_TIMER0->EVENTS_COMPARE[0] = 0; //Clear compare register 0 event	 
        
        switch (robot_vars.state) {
        case S_RADIORX:
            activity_rie0();
            break;       
        case S_USPREPARE:
            activity_ri2();
            break;  
        case S_TRIGGER:
            activity_ri3();
            break;      
        case S_ECHODONE:
            activity_ri6();
            break;      
        case S_RADIOTXDELAY:
            activity_ri7();
            break; 
        case S_RADIOTXREADY:
            activity_rie1();
            break;    
        case S_RADIOTX:
            activity_rie2();
            break; 
        case S_IFS:
            activity_ri10();
            break;                                  
        default:
            printf("ERROR FSM STATE IN THE TIMER ISR");
            break;
        }   
    }
}

/**
 * @brief ISR for reading ranging value from the sensor. It captures the value of the Timer1 (pulse width) and calls the callback for US reading
 * 
 */
void GPIOTE_IRQHandler(void){
  
    if((NRF_GPIOTE->EVENTS_IN[US_READ_CH_LoToHi] != 0) && (robot_vars.state == S_WAITECHOHIGH))
    {
        NRF_GPIOTE->EVENTS_IN[US_READ_CH_LoToHi] = 0UL;

        // capture time when Echo pin goes from LOW to HIGH
        robot_vars.timeEchoLowHigh = NRF_TIMER0->CC[0];
        activity_ri4(robot_vars.timeEchoLowHigh);

    }

    if((NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo] != 0) && (robot_vars.state == S_WAITECHOLOW))
    {
        NRF_GPIOTE->EVENTS_IN[US_READ_CH_HiToLo] = 0UL;

        // capture time when Echo pin goes from HIGH to LOW
        robot_vars.timeEchoHighLow = NRF_TIMER0->CC[0];
        activity_ri5(robot_vars.timeEchoHighLow);
    }
}

/**
 * @brief Interruption handler for the Radio.
 *
 * This function will be called each time a radio packet is received.
 *
 */

void RADIO_IRQHandler(void) {

    // Check if the interrupt was caused by the end of the frame
    if (NRF_RADIO->EVENTS_ADDRESS) {
        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_ADDRESS = 0;
        
        switch (robot_vars.state) {
        case S_RADIORXLISTEN:
            activity_ri0();
            break;              
        case S_RADIOTXPREPARE:
            activity_ri8();
            break;                                      
        default:
            printf("ERROR FSM STATE IN RADIO ISR START OF FRAME");
            break;
        }
    }

    // Check if the interrupt was caused by the end of the frame
    if (NRF_RADIO->EVENTS_END) {
        // Clear the Interrupt flag
        NRF_RADIO->EVENTS_END = 0;

        switch (robot_vars.state) {
        case S_RADIORX:
            activity_ri1();
            break;              
        case S_RADIOTX:
            activity_ri9();
            break;                                      
        default:
            printf("ERROR FSM STATE IN RADIO ISR END OF FRAME");
            break;
        }
    }
}
//=========================== functions =========================================

void activity_ri0(void){
    changeState(S_RADIORX);
    __NOP();
}

void activity_ri1(void) {
    changeState(S_USPREPARE);
    __NOP();
}

void activity_rie0(void) {
    __NOP();
    printf("ERROR rie0 Packet too long");
}

void activity_ri2(void) {
    changeState(S_TRIGGER);
    __NOP();
}

void activity_ri3(void) {
    changeState(S_WAITECHOHIGH);
    __NOP();
}

void activity_ri4(uint32_t capturedTime) {
    changeState(S_WAITECHOLOW);
    __NOP();
}

void activity_ri5(uint32_t capturedTime) {
    changeState(S_ECHODONE);
    __NOP();
}

void activity_ri6(void) {
    changeState(S_RADIOTXDELAY);
    __NOP();
}

void activity_ri7(void) {
    changeState(S_RADIOTXPREPARE);    
    __NOP();
    changeState(S_RADIOTXREADY); 
}

void activity_rie1(void) {
    __NOP();
    printf("ERROR rie1 packet not send at the right time");
}

void activity_ri8(void) {
    changeState(S_RADIOTX); 
    __NOP();
}

void activity_rie2(void) {
    __NOP();
    printf("ERROR rie2 packet too long");
}

void activity_ri9(void) {
    changeState(S_IFS); 
    __NOP();
}

void activity_ri10(void) {
    changeState(S_RADIORXLISTEN); 
    __NOP();
}

/**
\brief Changes the state of the RRDV Robot FSM.
Besides simply updating the state global variable, this function toggles the FSM debug pin.
\param[in] newstate The state the RRDV Robot FSM is now in.
*/
void changeState(robot_state_t newstate) {
    // update the state
    robot_vars.state = newstate;
    // set high first
    //debugpins_fsm_set();

    // wiggle the FSM debug pin
    switch (robot_vars.state) {
        case S_RADIORXLISTEN:
        case S_RADIORX:
        case S_USPREPARE:
        case S_TRIGGER:
        case S_WAITECHOHIGH:
        case S_WAITECHOLOW:
        case S_ECHODONE:
        case S_RADIOTXDELAY:
        case S_RADIOTXPREPARE:
        case S_RADIOTXREADY:
        case S_RADIOTX:
        case S_IFS:
            //toggle the pins
            //debugpins_fsm_toggle();
            break;
    }
}

//========================== End of File ========================================
