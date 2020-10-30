/*------------------------------------------------
 *   Author = Dixit Gurung
 *   EGR 326 901
 *   Date = 10/22/2020
 *   Lab_7: Master code for part III
 *   Description:
 *   Reads two digit number from keypad followed by a '#' key and has two switches to send data and determine direction of the motor
 *    when the switches are pressed the program sends the data(one character) using I2C protocol
 *---------------------------------------------------*/

#include "msp.h"
#include "Keypad.h"

#define S1 BIT3
#define S2 BIT2
#define SLAVE_ADDRESS 0x48

#define pound_key 30
#define star_key 20



uint32_t status1, status2;// Port interrupt Flag for two switches S1 and S2
char TXData[2];
int i = 0;
int keypressed;

void EUSCIB0_IRQHandler(void)
{
    EUSCI_B0->IFG &=~ EUSCI_B_IFG_TXIFG0;
    if(status1 && EUSCI_B_IFG_TXIFG0){//send pos value 0x2B = '+'
        EUSCI_B0->TXBUF = 0x2B;
        while(!(EUSCI_B0->IFG & 2));
        EUSCI_B0->TXBUF = TXData[1]+48;
        while(!(EUSCI_B0->IFG & 2));
        EUSCI_B0->TXBUF = TXData[0]+48;
        EUSCI_B0->IE &= ~EUSCI_B_IE_TXIE0;
        status1 = 0;}

    if(status2 && EUSCI_B_IFG_TXIFG0){//send neg value 0x2D = '-'
        EUSCI_B0->TXBUF = 0x2D;
        while(!(EUSCI_B0->IFG & 2));
        EUSCI_B0->TXBUF = TXData[1]+48;
        while(!(EUSCI_B0->IFG & 2));
        EUSCI_B0->TXBUF = TXData[0]+48;
        EUSCI_B0->IE &= ~EUSCI_B_IE_TXIE0;
        status2 = 0;}
}

void port_init(void){
    P3->DIR &= ~S1|S2;
    P3->REN = S1|S2;
    P3->OUT = S1|S2;
    P3->IE = S1|S2;
    P3->IES = S1|S2;
    P3->IFG = 0x00;
}


void I2C_init(void){

    P1->SEL0 |= BIT6 | BIT7; // P1.6 and P1.7 as UCB0SDA and UCB0SCL
    P1->SEL1 &= ~(BIT6 | BIT7);

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Hold EUSCI_B0 module in reset state
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_MST|EUSCI_B_CTLW0_SYNC;
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_UCSSEL_2; // Select SMCLK as EUSCI_B0 clock
    EUSCI_B0->BRW = 0x001E; // Set BITCLK = BRCLK / (UCBRx+1) = 3 MHz / 30 = 100 kHz
    EUSCI_B0->I2CSA = SLAVE_ADDRESS;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Clear SWRST to resume operation

}

void PORT3_IRQHandler(void)
{
    status1 = P3->IFG & S1;
    status2 = P3->IFG & S2;
    P3->IFG &= ~S1;
    P3->IFG &= ~S2;

    if((status1 & S1)|(status2 & S2))
    {
        EUSCI_B0->IE |= EUSCI_B_IE_TXIE0; // Enable EUSCI_A0 TX interrupt
    }
}






void main(void)
{

WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

port_init();
I2C_init();

NVIC->ISER[1] = 0x00000020; // Port P3 interrupt is enabled in NVIC
NVIC->ISER[0] = 0x00100000; // EUSCI_B0 interrupt is enabled in NVIC
__enable_irq(); // All interrupts are enabled

while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;



int index = 0;//variable to count the TXData array index
while(1){
    index = 0;
    keypressed = getKeypress();
            printf("\nEnter number of turns (0-9)followed '#'\n");

            while (keypressed != pound_key)
            {
                if (keypressed != -1 && keypressed != star_key)
                {
                    TXData[index] = keypressed;
                    index++;
                }
                keypressed = getKeypress(); //Reads the keypad again to repeat the process.
            }
            printf("\nYour Step:%d%d\n", TXData[0], TXData[1]);
}
}
