

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include "pins.h"
#include "ST7735.h"



int8_t initHw()
{
    // CLOCK CONFIGURATION
    // Following procedure from p231 datasheet to transition to 16MHz clock
    // Extra commands added to ensure value in every field
    SYSCTL_RCC_R &= ~SYSCTL_RCC_MOSCDIS;                                // Enable main oscillator
    SYSCTL_RCC_R |= SYSCTL_RCC_BYPASS;                                  // Bypass PLL to change frequency
    SYSCTL_RCC_R &= ~SYSCTL_RCC_USESYSDIV;                              // Disable SYSDIV
    SYSCTL_RCC_R &= ~(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);         // Clear XTAL and OSCSRC fields
    SYSCTL_RCC_R |= SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN;     // Set XTAL to 16MHz and OSCSRC to main
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWRDN;                                  // Clear PWRDN
    SYSCTL_RCC_R &= ~SYSCTL_RCC_SYSDIV_M;                               // Clear SYSDIV
    SYSCTL_RCC_R |=  SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S); // Use divisor of 5 (SYSDIV=4) p223  ==> 40MHz
    while (!(SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS));                       // Wait for PLL to lock
    SYSCTL_RCC_R &= ~SYSCTL_RCC_BYPASS;                                 // Use PLL clock
    SYSCTL_RCC_R &= ~SYSCTL_RCC_ACG;                                    // Disable ACG (irrelevant to this project)
    SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;                               // Clear PWMDIV
    SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;         // Set up PWM clock


    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;


    // CLOCK GATING
    // Enable GPIO ports ABCDEF peripherals
    // Enable Timers
    SYSCTL_RCGCGPIO_R = BT_0 | BT_1 | BT_2 | BT_3 | BT_4 | BT_5;


    // CONFIGURE BUTTONS
    // Pin Mapping:  PC6 PC7 PD6 PD7 PF4

    // Define inputs (clear bits)
    GPIO_PORTC_DIR_R &= ~(  BT_7 | BT_6                                               )  ;
    GPIO_PORTD_DIR_R &= ~(  BT_7 | BT_6                                               )  ;
    GPIO_PORTF_DIR_R &= ~(                         BT_4                               )  ;

    // Enable pins for digital (set bits)
    GPIO_PORTC_DEN_R |=  (  BT_7 | BT_6                                               )  ;
    GPIO_PORTD_DEN_R |=  (  BT_7 | BT_6                                               )  ;
    GPIO_PORTF_DEN_R |=  (                         BT_4                               )  ;

    // Enable pull up resistors (set bits)
    GPIO_PORTC_PUR_R |=  (  BT_7 | BT_6                                               )  ;
    GPIO_PORTD_PUR_R |=  (  BT_7 | BT_6                                               )  ;
    GPIO_PORTF_PUR_R |=  (                         BT_4                               )  ;



    // CONFIGURE LED
    // LARGE    RED:PC4  GREEN:PF0  BLUE:PE4  BCKLGT: PE5
    // Configure pins as outputs with 2mA strength

    GPIO_PORTC_DIR_R  |=  BT_4;
    GPIO_PORTC_DR2R_R |=  BT_4;
    GPIO_PORTC_DEN_R  |=  BT_4;
    //GPIO_PORTC_AFSEL_R |= BT_4;
    //GPIO_PORTC_PCTL_R |=  GPIO_PCTL_PC4_M0PWM6 ;

    GPIO_PORTE_DIR_R  |=  (   BT_4    )  ;
    GPIO_PORTE_DR4R_R |=  (   BT_4    )  ;
    GPIO_PORTE_DEN_R  |=  (   BT_4    )  ;


    GPIO_PORTF_LOCK_R  =  0x4C4F434B;
    GPIO_PORTF_CR_R   |=  BT_0;
    GPIO_PORTF_LOCK_R  =  0x0;
    GPIO_PORTF_DIR_R  |=  ( BT_3 | BT_0);
    GPIO_PORTF_DR2R_R |=  ( BT_3 | BT_0);
    GPIO_PORTF_DEN_R  |=  ( BT_3 | BT_0);
    //GPIO_PORTF_AFSEL_R |= BT_0;
    //GPIO_PORTF_PCTL_R |=  GPIO_PCTL_PF0_M1PWM4 ;


    // CONFIGURE Fan
    // based on ABF0612VHC
    // TACH: PA6  PWM: PA7

    // Enable pins for digital (set bits)
    GPIO_PORTA_DEN_R  |=  (   BT_7 | BT_6   )  ;

    // Define inputs (clear bits)
    GPIO_PORTA_DIR_R  &= ~(          BT_6   )  ;

    // Define outputs (set bits)
    GPIO_PORTA_DIR_R  |=  (   BT_7          )  ;

    // Set alternative functions
    GPIO_PORTA_AFSEL_R |= (   BT_7 | BT_6   )  ;
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA7_M1PWM3  ;



/*
    // Configure PWM
    // M0PWM4
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;               // reset PWM0 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;              // leave reset state
    while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
    PWM0_2_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;        // turn-off PWM0 generator 2
    PWM0_2_GENB_R = PWM_2_GENB_ACTCMPBD_ONE | PWM_2_GENB_ACTLOAD_ZERO;
                                                     // Turn on when number hit going down; turn off at load
    PWM0_2_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                     // also using 4096 because load overrides cmpb
    PWM0_2_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
    PWM0_2_CTL_R |= PWM_2_CTL_ENABLE;                // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM4EN;               // enable outputs


    // M0PWM5
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;               // reset PWM0 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;              // leave reset state
    while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
    PWM0_2_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;        // turn-off PWM0 generator 2
    PWM0_2_GENB_R = PWM_2_GENB_ACTCMPBD_ONE | PWM_2_GENB_ACTLOAD_ZERO;
                                                     // Turn on when number hit going down; turn off at load
    PWM0_2_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                     // also using 4096 because load overrides cmpb
    PWM0_2_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
    PWM0_2_CTL_R |= PWM_2_CTL_ENABLE;                // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM5EN;               // enable outputs


    // M0PWM6
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;               // reset PWM0 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;              // leave reset state
    while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R0));     // wait until ready
    PWM0_3_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;        // turn-off PWM0 generator 3
    PWM0_3_GENB_R = PWM_3_GENB_ACTCMPBD_ONE | PWM_3_GENB_ACTLOAD_ZERO;
                                                     // Turn on when number hit going down; turn off at load
    PWM0_3_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                     // also using 4096 because load overrides cmpb
    PWM0_3_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
    PWM0_3_CTL_R |= PWM_3_CTL_ENABLE;                // turn-on PWM0 generator 3
    PWM0_ENABLE_R = PWM_ENABLE_PWM6EN;               // enable outputs


    // M1PWM3
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R1;               // reset PWM0 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R1;              // leave reset state
    while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R1));     // wait until ready
    PWM0_1_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;        // turn-off PWM0 generator 1
    PWM0_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // Turn on when number hit going down; turn off at load
    PWM0_1_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                     // also using 4096 because load overrides cmpb
    PWM0_1_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
    PWM0_1_CTL_R |= PWM_1_CTL_ENABLE;                // turn-on PWM0 generator 1
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN;               // enable outputs


    // M1PWM4
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R1;               // reset PWM0 module
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R1;              // leave reset state
    while (!(SYSCTL_PRPWM_R & SYSCTL_PRPWM_R1));     // wait until ready
    PWM0_2_CTL_R = 0;// &= ~PWM_0_CTL_ENABLE;        // turn-off PWM0 generator 2
    PWM0_2_GENB_R = PWM_2_GENB_ACTCMPBD_ONE | PWM_2_GENB_ACTLOAD_ZERO;
                                                     // Turn on when number hit going down; turn off at load
    PWM0_2_LOAD_R = 4096;                            // set period to 66.67 MHz sys clock / 2 / 4096 = 8.138 kHz
                                                     // also using 4096 because load overrides cmpb
    PWM0_2_CMPB_R = 0;                               // light off (0 is off, 4096 is always on)
    PWM0_2_CTL_R |= PWM_2_CTL_ENABLE;                // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM4EN;               // enable outputs
*/



    // CONFIGURE Ethernet
    // based on ENC28J60 chip
    // Clk: PB4  RST: PB5  MISO: PB6  MOSI: PB7 CS: PF1

    // Enable pins for digital (set bits)
    GPIO_PORTB_DEN_R |=  (    BT_7 | BT_6 | BT_5 | BT_4   )  ;
    GPIO_PORTF_DEN_R |=  (                         BT_1   )  ;

    // Define inputs (clear bits)
    GPIO_PORTB_DIR_R &= ~(           BT_6                 )  ;

    // Set pull-up resistors
//    GPIO_PORTB_PUR_R |=  (           BT_6                 )  ;
    GPIO_PORTB_PDR_R |=  (                         BT_4   )  ;

    // Define outputs (set bits)
    GPIO_PORTB_DIR_R |=  (    BT_7 |        BT_5 | BT_4   )  ;
    GPIO_PORTF_DIR_R |=  (                         BT_1   )  ;

    // Set output level
    GPIO_PORTB_DR2R_R |=  (   BT_7 |        BT_5 | BT_4   )  ;

//    *BITBAND(GPIO_PORTB_DATA_R,5) = 0;

    // Set alternative functions
    GPIO_PORTB_AFSEL_R |= (   BT_7 | BT_6 | BT_4  )  ;
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB4_SSI2CLK | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB7_SSI2TX;

    // Enable clock gating and wait a little
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");

    // Disable SSI2 and set other fields while here EOT=0   MS=0   SSE=0   LBM=0
    SSI2_CR1_R &= ~( SSI_CR1_SSE);
    SSI2_CR1_R &= ~(SSI_CR1_EOT | SSI_CR1_MS | SSI_CR1_LBM);

    // Select PLL clock
    SSI2_CC_R = SSI_CC_CS_SYSPLL;

    // Set the prescale to 2 (minimum value).  Leave SCR field of CR0 at zero.  20MHz result.  p969 Datasheet
    SSI2_CPSR_R = 2;

    // SCR=0   SPH=0   SPO=0   FRF=0   DSS=7 (8bit data) Top 16 bits are reserved and untouched
    SSI2_CR0_R &= 0xffff0000;
    SSI2_CR0_R |= 0x0007;
    SSI2_CR0_R |= 0x2 << 3;


    SSI2_CR1_R |= SSI_CR1_SSE;






    // CONFIGURE Screen
    // based on ST7735
    // Clk: PA2  RST: PA3  MISO: PA4  MOSI: PA5  CS: PC5

    // Enable pins for digital (set bits)
    GPIO_PORTA_DEN_R |=  (    BT_5 | BT_4 | BT_3 | BT_2   )  ;
    GPIO_PORTC_DEN_R |=  (    BT_5                        )  ;

    // Define inputs (clear bits)
    GPIO_PORTA_DIR_R &= ~(           BT_4                 )  ;

    // Set pull-up resistors
    //    GPIO_PORTB_PUR_R |=  (           BT_6                 )  ;
    GPIO_PORTA_PDR_R |=  (                         BT_2   )  ;

    // Define outputs (set bits)
    GPIO_PORTA_DIR_R |=  (    BT_5 |               BT_2   )  ;
    GPIO_PORTC_DIR_R |=  (    BT_5                        )  ;

    // Set output level
    GPIO_PORTA_DR2R_R |=  (   BT_5 |               BT_2   )  ;
    GPIO_PORTC_DR2R_R |=  (   BT_5                        )  ;

    //    *BITBAND(GPIO_PORTB_DATA_R,5) = 0;

    // Set alternative functions
    GPIO_PORTA_AFSEL_R |= (   BT_5 | BT_4 |         BT_2  )  ;
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA2_SSI0CLK | GPIO_PCTL_PA4_SSI0RX | GPIO_PCTL_PA5_SSI0TX;

    // Enable clock gating and wait a little
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R0;
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");
    __asm("     NOP");


    ST7735_CS=0;
    ST7735_RST=1;
    Delay1ms(500);
    ST7735_RST=0;
    Delay1ms(500);
    ST7735_RST=1;
    Delay1ms(500);
    ST7735_CS=1;





    //Page 965, step 5
    // Disable SSI0 and set other fields while here EOT=0   MS=0   SSE=0   LBM=0
    SSI0_CR1_R &= ~( SSI_CR1_SSE);
    SSI0_CR1_R &= ~(SSI_CR1_EOT | SSI_CR1_MS | SSI_CR1_LBM);
    
    // Set GPIODEN
    

    // Select PLL clock
    SSI0_CC_R = SSI_CC_CS_SYSPLL;

    // Set the prescale to 2 (minimum value).  Leave SCR field of CR0 at zero.  20MHz result.  p969 Datasheet
    SSI0_CPSR_R = 8;

    // SCR=0   SPH=0   SPO=0   FRF=0   DSS=7 (9bit data) Top 16 bits are reserved and untouched
    SSI0_CR0_R &= 0xffff0000;
    SSI0_CR0_R |= 0x0008;
    SSI0_CR0_R |= 0x2 << 3;


    SSI0_CR1_R |= SSI_CR1_SSE;

    ST7735_CS = 1;



    return 0;
}


int main(void)
{

    initHw();


    RED_LED=0;
    GREEN_LED=0;
    BLUE_LED=0;
  //  RED_LED=1;
  //  GREEN_LED=1;
  //  BLUE_LED=1;
 //   COMP_FAN=1;

    BCKLGT_LED=0;
    BCKLGT_LED=1;
    ST7735_InitR(INITR_GREENTAB);
    uint16_t testColor = ST7735_Color565 (10, 160, 160);
//    ST7735_FillRect(10, 20, 30, 30, ST7735_TEAL);
    ST7735_FillRect(10, 20, 30, 30, testColor);
       //printf ("Hello World!");

   while (1)
    {
       BLUE_LED=1;
    }

    RED_LED=0;
    GREEN_LED=0;
    BLUE_LED=0;
    BCKLGT_LED=0;
    COMP_FAN=0;

	return 0;
}
