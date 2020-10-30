/*------------------------------------------------
 *   Author = Dixit Gurung
 *   EGR 326 901
 *   Date = 10/22/2020
 *   Lab_7: Slave code for part III
 *   Description:
 *   Stores the data sent by the master code in the RXData[3] array and runs the motor accordingly
 *---------------------------------------------------*/


#include "msp.h"
#include <stdio.h>
//#include "egr326_lib.h"

void stepper_motor_initial(void);
void button_initial(void);
uint8_t debounce_button(void);
void delay_milli(int micro);

//For slave
#define LED2_RED BIT0
#define LED2_GREEN BIT1
#define LED2_BLUE BIT2
#define SLAVE_ADDRESS 0x48
char RXData[3];
void I2C_init(void);
void control_motor(int step_num, int direction);
int check_flag = 0;
int count = 0;

// steps for state macine
#define STEP1 1
#define STEP2 2
#define STEP3 3
#define STEP4 4

// Macros for direction of stepper motor
#define FORWARD 1
#define REVERSE 0

volatile int interrupt_flag = 0;
volatile int i = 0;// Used in for loop in control_motor function
volatile int state = STEP1;
volatile int direction = REVERSE;
volatile int debounce_flag = 0;
volatile int step_num = 360;

int main()
{
    //For slave
    //Init LEDs for testing
    P2->DIR = LED2_RED|LED2_GREEN|LED2_BLUE;
    P2->OUT = 0x00;
    I2C_init();//Initializing P1.6&1.7 for I2C purpose




    stepper_motor_initial();
    button_initial();

    NVIC->ISER[1] = 1 << ((PORT2_IRQn & 31)); // Port 2 Interrupt
    __enable_interrupt();

    //control_motor( step_num,  direction);
int position=0;
    while (1)
    {

        if (count == 3){
            if(RXData[0] == 45){ //If the first value is a "-"
                step_num = 0;
                step_num = 4 * ((RXData[1] - 48)*10 + (RXData[2] - 48));  //Convert to decimal from ascii
                direction = 1; //Negative rotation
            }else{
                step_num = 0;
                step_num = 4 * ((RXData[1] - 48)*10 + (RXData[2] - 48));   //Convert to decimal from ascii
                direction = 0; //Positive rotation
            }

                position += step_num;      //Keep track of the total position
                control_motor( abs(step_num),  direction);  //Move the stepper motor in the proper direction
                printf("Current position: %d\n", position);   //Print the current position
                count = 0;  //Reset for reading next I2C data transmission


            }
        }
}

/************************************************************************************
 * This function initializes the pins used for the stepper motor. All the outputs
 * are set to low at the start.
 * IN1 -> P10.0
 * IN2 -> P10.1
 * IN3 -> P10.2
 * IN4 -> P10.3
 ***********************************************************************************/
void stepper_motor_initial(void)
{
    P10->SEL0 &= ~(BIT0 | BIT1 | BIT2 | BIT3 );
    P10->SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3 );
    P10->DIR |= (BIT0 | BIT1 | BIT2 | BIT3 );
    P10->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 );
}

void button_initial(void)
{
    P2->SEL0 &= ~(BIT5);
    P2->SEL1 &= ~(BIT5);
    P2->DIR &= ~(BIT5);
    P2->REN |= (BIT5);
    P2->OUT |= (BIT5);
    P2->IES |= (BIT5);
    P2->IE |= (BIT5);

    P2->IFG = 0;

    SysTick->CTRL = 0;
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;

}

// This function debounces the first pushbutton using Jack's Ganssle's code.
uint8_t debounce_button(void)
{

    static uint16_t state = 0; // current state of the button

    // Note: 0xf800 is the delay time
    /*
     * This line shifts the current value of state over by 1. It then bit wise or's
     * the value of Oxf800 so that the first 5 bits are allow ones. This
     * makes it so that the first 4 bits are not used. It will also
     * bit wise or the value with the reading at P2.5 and shifted over so that
     * the read value located at bit 0 in the variable state. Eventually, if the
     * pushbutton keeps reading so, the pushbutton will be debounce when the 10
     * right most bits are all 0.
     */
    state = (state << 1) | (P2->IN & BIT5 ) >> 5 | 0xf800;

    // Checks to see the 10 right most bits are 0, if so, then it is debounced
    if (state == 0xfc00)
    {
        return 1;
    }

    return 0;
}


// Port 2 interrupt handler
void PORT2_IRQHandler(void)
{
    if (P2->IFG & BIT5)
    {
        interrupt_flag = 1; // Set flag to 1
    }

    P2->IFG = 0;
}

/*
 * Delay in milliseconds using SysTick Timer
 */
void delay_milli(int time)
{
    SysTick->CTRL = 0;
    SysTick->LOAD = (time * 3000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;

    while ((SysTick->CTRL & 0x00010000) == 0);
}


/************************************************************************************
 * This function controls the stepper motor
 *Takes two arguent steps and direction
 *step_num =  number of steps to rotate
 *direction = Determines forward or backward direction
 ***********************************************************************************/
void control_motor(int step_num, int direction){
    for(i = 0; i < step_num; i++)
                {
                    switch (state)
                    {
                        case STEP1:

                            P10->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);
                            P10->OUT |= (BIT0 | BIT2);
                            break;

                        case STEP2:
                            P10->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);
                            P10->OUT |= (BIT1 | BIT2);
                            break;

                        case STEP3:
                            P10->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);
                            P10->OUT |= (BIT1 | BIT3);
                            break;

                        case STEP4:
                            P10->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);
                            P10->OUT |= (BIT0 | BIT3);
                            break;

                        default:
                            P10->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);
                            state = STEP1;
                    }

                    // change state based on direction
                    if (direction == FORWARD){

                        state--;
                        if (state == 0) state = STEP4;
                    }

                    if (direction == REVERSE){

                        state++;
                        if (state == 5) state = STEP1;
                     }
                    delay_milli(10);
                }
}








//Initalize P1.6 and P1.7 for I2C protocol
void I2C_init(void){
    P1->SEL0 |= BIT6 | BIT7; // P1.6 and P1.7 as UCB0SDA and UCB0SCL

        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Hold EUSCI_B0 module in reset state
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_SYNC;
        EUSCI_B0->I2COA0 = SLAVE_ADDRESS | EUSCI_B_I2COA0_OAEN;
        EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Clear SWRST to resume operation
        EUSCI_B0->IFG &= ~EUSCI_B_IFG_RXIFG0; // Clear EUSCI_B0 RX interrupt flag
        EUSCI_B0->IE |= EUSCI_B_IE_RXIE0; // Enable EUSCI_B0 RX interrupt

        NVIC->ISER[0] = 0x00100000; // EUSCI_B0 interrupt is enabled in NVIC
}

void EUSCIB0_IRQHandler(void)
{
    printf("Start program\n");
    uint32_t status = EUSCI_B0->IFG; // Get EUSCI_B0 interrupt flag
    EUSCI_B0->IFG &=~ EUSCI_B_IFG_RXIFG0; // Clear EUSCI_B0 RX interrupt flag
    if(status & EUSCI_B_IFG_RXIFG0) // Check if receive interrupt occurs
    {
        RXData[count] = EUSCI_B0->RXBUF; // Load current RXData value to transmit buffer

        count++;
    }
}
