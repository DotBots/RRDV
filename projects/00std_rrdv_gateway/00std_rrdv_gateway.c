/**
 * @file 00std_rrdv_robot.c
 * @author Trifun Savic <trifun.savic@inria.fr>
 *
 * @brief This is an example application for Ultrasound ranging Gateway 
 *
 * @copyright Inria, 2022
 *
 */
//TODO

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nrf.h"

//============================ defines ==========================================

// gateway
#define GATEWAY_ID 0x77

// radio
#define RADIO_BASE_ADDRESS_0             0x12345678UL 
#define RADIO_BASE_ADDRESS_1             0xFEDCBA98UL
#define NUMBER_OF_RADIO_BYTES_IN_PACKET  5UL
#define NUMBER_OF_RADIO_BYTES_RECEIVED   5UL
#define NUMBER_OF_RADIO_BYTES_TO_SEND    5UL

// timer 
#define defaultCmpValue  0xDEADBEEF  // big value to initialize when we don't want timer to elapse
#define gatewayTxOffset_ms 1
#define cmdDur_ms          0.65 
#define maxCmdDur_ms       1 
#define wdTx_ms            0.01
#define usDuration_ms      180
#define rxPrepare_ms       //TODO 
#define rxDelay_ms         //TODO    
#define notifDur_ms        0.65 
#define notifDurAll_ms     //TODO 
#define ifsDur_ms          0.01 
#define listenDur_ms       //TODO
#define procDelay_ms       //TODO
#define txUartPrepare_ms   //TODO
#define maxUartTxDur_ms    //TODO

// UART
#define UART_MAX_BYTES     64UL//

// ppi
#define channel_gi0                     0UL
#define channel_gi1                     1UL
#define channel_gi2                     2UL
#define channel_gi3                     3UL
#define channel_gi4                     4UL
#define channel_gi5                     5UL
#define channel_gi6                     6UL
#define channel_gi7                     7UL
#define channel_gi8                     8UL
#define channel_gi9                     9UL
#define channel_gie0_or_gie1_or_gie2    11UL
#define ppiChannelClrEnable             1UL
#define ppiChannelSetEnable             1UL

// interrupts
#define TIMER0_INT_PRIORITY     0UL
#define RADIO_INT_PRIORITY      1UL
#define UART_INT_PRIORITY       2UL
 
// debug
#define FSM_DEBUG_PIN           31UL 
#define ISR_DEBUG_PIN           30UL

//=========================== typedefs ========================================

// FSM
typedef enum {
    S_UARTRXENABLE      = 1UL,
    S_UARTRXDATA        = 2UL,
    S_RADIOTXPREPARE    = 3UL,
    S_RADIOTXREADY      = 4UL,
    S_RADIOTX           = 5UL,
    S_SLEEP             = 6UL,
    S_RADIORXREADY      = 7UL,
    S_RADIORXLISTEN     = 8UL,
    S_RADIORX           = 9UL,
    S_IFS               = 10UL,
    S_RADIORXLISTENNEXT = 11UL,
    S_UARTTXPREPARE     = 12UL,
    S_UARTTXREADY       = 13UL,
    S_UARTTXDATA        = 14UL
} gateway_state_t;

typedef struct {
    //robot
    gateway_state_t state;
    //radio
    uint8_t         dataToSendRadio[NUMBER_OF_RADIO_BYTES_TO_SEND];
    uint8_t         dataReceivedRadio[NUMBER_OF_RADIO_BYTES_RECEIVED];
    uint8_t         radioPacket[NUMBER_OF_RADIO_BYTES_IN_PACKET];
    //uart
    uint8_t         dataToSendUart[UART_MAX_BYTES];
    uint8_t         dataReceivedUart[UART_MAX_BYTES];
} gateway_vars_t;  

                           
//=========================== prototypes ========================================

// ISR
void activity_gi0(void);
void activity_gi1(void);
void activity_gi2(void);
void activity_gie0(void);
void activity_gi3(void);
void activity_gie1(void);
void activity_gi4(void);
void activity_gi5(void);
void activity_gi6(void);
void activity_gi7(void);
void activity_gi8(void);
void activity_gi9(void);

// Init
void uart_init(void);
void timer_init(void);
void radio_init(void);
void radio_set_frequency(uint8_t freq);
void radio_rx_enable(void);
void hfclk_init(void);
void ppi_setup(void);

// Defaults
void gateway_init_setup(void);
void changeState(gateway_state_t newstate);

// Debug
void set_debug_pins(void);
void toggle_fsm_debug_pin(void);
void toggle_isr_debug_pin(void);

//=========================== variables =========================================

gateway_vars_t gateway_vars;

//============================= main ============================================
int main(void) {
    
    // initialize variables
    memset(&gateway_vars, 0, sizeof(gateway_vars_t));
   
    uart_init();
    timer_init();
    radio_init();
    radio_set_frequency(8);
    ppi_setup();
    hfclk_init();

    set_debug_pins();
    gateway_init_setup();  

    while (1) {        
      /*  __WFE();      
        __SEV();
        __WFE();*/
    }
    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//============================= ISRs ============================================

/**
@brief Indicates the FSM timer has fired.

This function executes in ISR mode, when the FSM timer fires.
*/

void TIMER0_IRQHandler(void){ 
    if(NRF_TIMER0->EVENTS_COMPARE[0] != 0)
    {   
        NRF_TIMER0->EVENTS_COMPARE[0] = 0; //Clear compare register 0 event	 
        
        switch (gateway_vars.state) {
        case S_RADIOTX: //TODO
            activity_gie0();
            break;                                         
        default:
            printf("ERROR FSM STATE IN THE TIMER ISR");
            break;
        }   
    }
}

/**
 * @brief ISR for handling UART Tx and Rx
 * 
 */
void UART0_IRQHandler(void){   
    //TODO
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
            
        switch (gateway_vars.state) {
        case S_RADIORXLISTEN: //TODO
            activity_gi0();
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

        switch (gateway_vars.state) { 
        case S_RADIORX: //TODO
            activity_gi1();
            break;                                                   
        default:
            printf("ERROR FSM STATE IN RADIO ISR END OF FRAME");
            break;
        }
    }
}
//=========================== functions =========================================

void activity_gi0(void){
    toggle_isr_debug_pin();
    // switch state
    changeState(S_UARTRXDATA);

    //TODO

    toggle_isr_debug_pin();
}

void activity_gi1(void) {
    toggle_isr_debug_pin();
    
    changeState(S_RADIOTXPREPARE);

    //TODO

    changeState(S_RADIOTXREADY);

    toggle_isr_debug_pin();
}

void activity_gi2(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIOTX);

    //TODO

    toggle_isr_debug_pin();
}

void activity_gie0(void) {
    toggle_isr_debug_pin();
    printf("ERROR gie0 command not sent at the right time\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);    
    __NOP();
    toggle_isr_debug_pin();
}

void activity_gi3(void) {
    toggle_isr_debug_pin();
    changeState(S_SLEEP);

    //TODO
    toggle_isr_debug_pin();
}

void activity_gie1(void) {
    toggle_isr_debug_pin();
    printf("ERROR gie1 command too long\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);
    __NOP();
    toggle_isr_debug_pin();
}

void activity_gi4(void) {
    toggle_isr_debug_pin(); 
    changeState(S_RADIORXREADY);
    
    //TODO

    toggle_isr_debug_pin();
}

void activity_gi5(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIORXLISTEN);

    //TODO

    toggle_isr_debug_pin();
}

void activity_gi6(void) {
    toggle_isr_debug_pin();
    changeState(S_RADIORX);

    //TODO

    toggle_isr_debug_pin();
}

void activity_gi7(void) {
    toggle_isr_debug_pin();
    changeState(S_IFS);    
    
    //TODO

    changeState(S_RADIORXLISTENNEXT);

    toggle_isr_debug_pin();
}

void activity_gi8(void) {
    toggle_isr_debug_pin();
    changeState(S_UARTTXPREPARE); 

    //TODO

    changeState(S_UARTTXREADY);

    toggle_isr_debug_pin();
}

void activity_gi9(void) {
    toggle_isr_debug_pin();
    changeState(S_UARTTXDATA); 

    //TODO

    changeState(S_UARTRXENABLE);
    toggle_isr_debug_pin();
}

void activity_gie2(void) {
    toggle_isr_debug_pin();    
    printf("ERROR gie2 UART packet too long\r\n");
    printf("current value of the timer: %d", NRF_TIMER0->CC[1]);
    __NOP();
    toggle_isr_debug_pin();
}

/**
 * @brief Function for initializing UART
 */
void uart_init(void) {
    //TODO
}

/**
 * @brief This function sets up Timer0 used as FSM timer 
 * we use only CC[0] compare register
 */
void timer_init(void) {
    // configure the Timer for US on
    NRF_TIMER0->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER0->PRESCALER = 0UL;

    // enable interrupts on compare 0 register
    NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

    // Configure the Interruptions for the Timer0 used as the ON US sensor timer
    NVIC_DisableIRQ(TIMER0_IRQn); 
    NVIC_SetPriority(TIMER0_IRQn, TIMER0_INT_PRIORITY);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);    // Clear the flag for any pending radio interrupt
    // enable interupts
    NVIC_EnableIRQ(TIMER0_IRQn);
       
}

/**
 * @brief Initializes the Long Range RADIO peripheral (125 kbps). 
 *
 * After this function you must explicitly set the frequency of the radio
 * with the db_radio_set_frequency function.
 *
 */
void radio_init(void) {

    // General configuration of the radio.
    NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos); // 8dBm Power output

    NRF_RADIO->MODE  = (RADIO_MODE_MODE_Ble_LR125Kbit << RADIO_MODE_MODE_Pos);       // Use Long Range 125 kbps modulation
    
    // Coded PHY (Long range)
    NRF_RADIO->PCNF0 =  (0                               << RADIO_PCNF0_S1LEN_Pos)    |
                        (1                               << RADIO_PCNF0_S0LEN_Pos)    |
                        (8                               << RADIO_PCNF0_LFLEN_Pos)    |
                        (3                               << RADIO_PCNF0_TERMLEN_Pos)  |
                        (2                               << RADIO_PCNF0_CILEN_Pos)    |
                        (RADIO_PCNF0_PLEN_LongRange      << RADIO_PCNF0_PLEN_Pos);

    NRF_RADIO->PCNF1 =  (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos)  |
                        (RADIO_PCNF1_ENDIAN_Little       << RADIO_PCNF1_ENDIAN_Pos)   |
                        (3                               << RADIO_PCNF1_BALEN_Pos)    |
                        (0                               << RADIO_PCNF1_STATLEN_Pos)  |
                        (NUMBER_OF_RADIO_BYTES_IN_PACKET << RADIO_PCNF1_MAXLEN_Pos);

    // Configuring the on-air radio address
    NRF_RADIO->BASE0    = RADIO_BASE_ADDRESS_0; // base address for prefix 0
    NRF_RADIO->BASE1    = RADIO_BASE_ADDRESS_1; // base address for prefix 1-7

    NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
    NRF_RADIO->TXADDRESS   = (0 << RADIO_TXADDRESS_TXADDRESS_Pos) & RADIO_TXADDRESS_TXADDRESS_Msk;

    // CRC Config
    NRF_RADIO->CRCCNF  =  (RADIO_CRCCNF_LEN_Two         << RADIO_CRCCNF_LEN_Pos);        // Checksum uses 2 bytes, and is enabled.
    NRF_RADIO->CRCINIT =  0xFFFFUL;                                                      // initial value
    NRF_RADIO->CRCPOLY =  0x11021UL;                                                     // CRC poly: x^16 + x^12^x^5 + 1

    // pointer to packet payload
    NRF_RADIO->PACKETPTR = (uint32_t)gateway_vars.radioPacket;

        // Configure the Short to expedite the packet transmission
    NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos) |  // yet the packet as soon as the radio is ready
                        (RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos);   // disable the radio as soon as the packet is sent/received
   
    // Configure the Interruptions
    NVIC_DisableIRQ(RADIO_IRQn);                                                // Disable interruptions while configuring

    // Enable interruption for when at address and end events to call ISR at the start and end of the frame
    NRF_RADIO->INTENSET = (RADIO_INTENSET_ADDRESS_Enabled << RADIO_INTENSET_ADDRESS_Pos) | 
                          (RADIO_INTENSET_END_Enabled     << RADIO_INTENSET_END_Pos);

    NVIC_SetPriority(RADIO_IRQn, RADIO_INT_PRIORITY);                     // Set priority for Radio interrupts to 1
    NVIC_ClearPendingIRQ(RADIO_IRQn);                                           // Clear the flag for any pending radio interrupt
    
    // Enable Radio interruptions
    NVIC_EnableIRQ(RADIO_IRQn);
}

/**
 * @brief Set the tx-rx frequency of the radio, by the following formula
 *
 * Radio frequency 2400 + freq (MHz) [0, 100]
 *
 * @param[in] freq Frequency of the radio [0, 100]
 */
void radio_set_frequency(uint8_t freq) {

    NRF_RADIO->FREQUENCY = freq << RADIO_FREQUENCY_FREQUENCY_Pos;
}

/**
 * @brief This function puts radio in the Rx mode
 */
void radio_rx_enable(void){
    // Radio initially in Rx mode
    NRF_RADIO->EVENTS_RXREADY = 0;                                    // Clear the flag before enabling the Radio.
    NRF_RADIO->TASKS_RXEN     = RADIO_TASKS_RXEN_TASKS_RXEN_Trigger;  // Enable radio reception.
    while (NRF_RADIO->EVENTS_RXREADY == 0) {}                         // Wait for the radio to actually start receiving.
}

/**
 * @brief This function is public and it sets the PPI channels to allow the US trigger and US echo. Ranging starts by calling this function
 */
void ppi_setup(void) {  
    // UART
    //TODO

    // TIMER
    uint32_t timer0_task_capture1_addr      = (uint32_t)&NRF_TIMER0->TASKS_CAPTURE[1];
    uint32_t timer0_task_clear_addr         = (uint32_t)&NRF_TIMER0->TASKS_CLEAR;
    uint32_t timer0_task_stop_addr          = (uint32_t)&NRF_TIMER0->TASKS_STOP;
    uint32_t timer0_events_compare_0_addr   = (uint32_t)&NRF_TIMER0->EVENTS_COMPARE[0];   
    // RADIO
    uint32_t radio_events_address_addr      = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    uint32_t radio_events_end_addr          = (uint32_t)&NRF_RADIO->EVENTS_END;
    uint32_t radio_tasks_txen_addr          = (uint32_t)&NRF_RADIO->TASKS_TXEN;
    uint32_t radio_tasks_rxen_addr          = (uint32_t)&NRF_RADIO->TASKS_RXEN;

    // set channels
    //TODO
     
}

/**
 * @brief This function enables HFCLK
 */
void hfclk_init(void) {   
    // Configure the external High-frequency Clock. (Needed for correct operation)
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0x00;    // Clear the flag
    NRF_CLOCK->TASKS_HFCLKSTART    = 0x01;    // Start the clock
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {;} // Wait for the clock to actually start.
}

/**
\brief Changes the state of the RRDV Robot FSM.
Besides simply updating the state global variable, this function toggles the FSM debug pin.
\param[in] newstate The state the RRDV Robot FSM is now in.
*/
void changeState(gateway_state_t newstate) {
    // update the state
    gateway_vars.state = newstate;

    // wiggle the FSM debug pin
    switch (gateway_vars.state) {
        case S_UARTRXENABLE:
        case S_UARTRXDATA:
        case S_RADIOTXPREPARE:
        case S_RADIOTXREADY:
        case S_RADIOTX:
        case S_SLEEP:
        case S_RADIORXREADY:
        case S_RADIORXLISTEN:
        case S_RADIORX:
        case S_IFS:
        case S_RADIORXLISTENNEXT:
        case S_UARTTXPREPARE:
        case S_UARTTXREADY:
        case S_UARTTXDATA:
            // toggle the pins
            toggle_fsm_debug_pin();
            break;
    }
}


/**
 * @brief This function initializes the variables with default values for initializing gateway
 */ 
void gateway_init_setup(void) {
    // DEFAULTS
    //TODO
}

/**
 * @brief This function sets the debug pins for ISR and FSM
 */ 
void set_debug_pins(void) {

    // For debug FSM Set the test pin (P0.31) as an output
    NRF_P0->PIN_CNF[FSM_DEBUG_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                                     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.
    // HIGH by default
    NRF_P0->OUTSET = 1 << FSM_DEBUG_PIN;

    // For debug ISR Set the test pin (P0.30) as an output
    NRF_P0->PIN_CNF[ISR_DEBUG_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |  // Set Pin as output
                                     (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos); // Activate high current gpio mode.

}

/**
 * @brief This function toggles the FSM pin
 */ 
void toggle_fsm_debug_pin(void) {
    NRF_P0->OUT ^= 1 << FSM_DEBUG_PIN;
}

/**
 * @brief This function toggles the ISR pin
 */  
void toggle_isr_debug_pin(void) {
    // Toggle
    NRF_P0->OUT ^= 1 << ISR_DEBUG_PIN;
}

//========================== End of File ========================================
