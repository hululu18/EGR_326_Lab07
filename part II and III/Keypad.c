
#include "msp.h"

#include <stdio.h>

#include <stdlib.h>





void setupKeypad(void){

    //columns setup
    P8-> SEL0 &= ~(BIT5|BIT6|BIT7);
    P8-> SEL1 &= ~(BIT5|BIT6|BIT7);
    P8-> DIR &= ~(BIT5|BIT6|BIT7);
    P8-> REN |= (BIT5|BIT6|BIT7);
    P8-> OUT |= (BIT5|BIT6|BIT7);



    //Rows setup
    P9-> SEL0 &= ~(BIT0|BIT1);
    P9-> SEL1 &= ~(BIT0|BIT1);
    P9-> DIR &= ~(BIT0|BIT1);
    P9-> REN |= (BIT0|BIT1);
    P9-> OUT |= (BIT0|BIT1);

/*    P1-> SEL0 &= ~(BIT6|BIT7);
    P1-> SEL1 &= ~(BIT6|BIT7);
    P1-> DIR &= ~(BIT6|BIT7);
    P1-> REN |= (BIT6|BIT7);
    P1-> OUT |= (BIT6|BIT7);*/

    P8-> SEL0 &= ~(BIT3|BIT4);
    P8-> SEL1 &= ~(BIT3|BIT4);
    P8-> DIR &= ~(BIT3|BIT4);
    P8-> REN |= (BIT3|BIT4);
    P8-> OUT |= (BIT3|BIT4);

}




int getKeypress(void) {

    setupKeypad();  //sets everything back to pull up resistor every time readKeypad is called



    //set P4.0 to output = 0

    P8 ->DIR |= BIT5;

    P8 -> OUT &= ~BIT5;



//check if a button in that row is pressed


    if(!(P9->IN &BIT1))
    {
        while(!(P9->IN &BIT1)); //sits at this line while the button is pressed - allows for a single button press
        return 9;   //choose 91 because not acceptable and then can change after determining what actually is
    }



    if(!(P9->IN &BIT0))
    {
        while(!(P9->IN &BIT0));
        return 30;
    }



    if(!(P8->IN &BIT4))
    {
        while(!(P8->IN &BIT4));
        return 3;
    }



    if(!(P8->IN &BIT3))
     {
         while(!(P8->IN &BIT3));
         return 6;
     }




    //set P4.0 back to pull-up resistor

    P8 ->DIR &= ~BIT5;

    P8 -> OUT |= BIT5;

    __delay_cycles(20);



    //set P4.2 to an output = 0

    P8 ->DIR |= BIT6;

    P8 -> OUT &= ~BIT6;



    //read each row

    //check if a button in that row is pressed

    if(!(P9->IN &BIT1))
      {
          while(!(P9->IN &BIT1)); //sits at this line while the button is pressed - allows for a single button press
          return 8;   //choose 91 because not acceptable and then can change after determining what actually is
      }



      if(!(P9->IN &BIT0))
      {
          while(!(P9->IN &BIT0));
          return 0;
      }



      if(!(P8->IN &BIT4))
      {
          while(!(P8->IN &BIT4));
          return 2;
      }



      if(!(P8->IN &BIT3))
       {
           while(!(P8->IN &BIT3));
           return 5;
       }




    P8-> DIR &=~ BIT6;

    P8-> OUT |=  BIT6;

    __delay_cycles(20);



    //set P6.1 to an output = 0

     P8 ->DIR |= BIT7;

     P8 -> OUT &= ~BIT7;



     //read each row

     //check if a button in that row is pressed

     if(!(P9->IN &BIT1))
     {
         while(!(P9->IN &BIT1)); //sits at this line while the button is pressed - allows for a single button press
         return 7;   //choose 91 because not acceptable and then can change after determining what actually is
     }



     if(!(P9->IN &BIT0))
     {
         while(!(P9->IN &BIT0));
         return 20;
     }



     if(!(P8->IN &BIT4))
     {
         while(!(P8->IN &BIT4));
         return 1;
     }



     if(!(P8->IN &BIT3))
      {
          while(!(P8->IN &BIT3));
          return 4;
      }





     //set P6.1 Back to High

      P8 ->DIR &= ~BIT7;

      P8 -> OUT |= BIT7;

      __delay_cycles(20);



      //if no button is pressed return -1

      if((P9->IN &BIT1)|(P9->IN &BIT0)|(P8->IN &BIT4)|(P8->IN &BIT3))

      {

          return -1;

      }

}




