#ifndef PINS_H_
#define PINS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bitband and Pin Definitions
//
///////////////////////////////////////////////////////////////////////////////////////////////////


#define BITBAND(BYTE_ADDR,BIT) (((volatile uint32_t *)(   (((uint32_t)&(BYTE_ADDR))&(0xF0000000)) + 0x02000000 + (((uint32_t)&(BYTE_ADDR)) << 5) + ((BIT) << 2)    )))

#define  LED_LG_RED      BITBAND(GPIO_PORTC_DATA_R,4)
#define  LED_LG_GREEN    BITBAND(GPIO_PORTF_DATA_R,0)
#define  LED_LG_BLUE     BITBAND(GPIO_PORTE_DATA_R,4)
#define  LED_SCRN_BCKLGT BITBAND(GPIO_PORTF_DATA_R,3)

#define  RED_LED      *LED_LG_RED
#define  GREEN_LED    *LED_LG_GREEN
#define  BLUE_LED     *LED_LG_BLUE
#define  BCKLGT_LED   *(LED_SCRN_BCKLGT)



#define  BUTTON_LEFT     BITBAND(GPIO_PORTC_DATA_R,6)
#define  BUTTON_UP       BITBAND(GPIO_PORTC_DATA_R,7)
#define  BUTTON_DOWN     BITBAND(GPIO_PORTD_DATA_R,6)
#define  BUTTON_RIGHT    BITBAND(GPIO_PORTD_DATA_R,7)
#define  BUTTON_SELECT   BITBAND(GPIO_PORTF_DATA_R,4)



#define  FAN_OUTSIDE     BITBAND(GPIO_PORTA_DATA_R,7)

#define  COMP_FAN     *FAN_OUTSIDE



#define  ST7735_CS       * ( BITBAND(GPIO_PORTC_DATA_R,5) )
#define  ST7735_RST      * ( BITBAND(GPIO_PORTA_DATA_R,3) )



#define BT_0 (0x01)
#define BT_1 (0x02)
#define BT_2 (0x04)
#define BT_3 (0x08)
#define BT_4 (0x10)
#define BT_5 (0x20)
#define BT_6 (0x40)
#define BT_7 (0x80)
#define BT_8 (0x0100)
#define BT_9 (0x0200)
#define BT_10 (0x0400)
#define BT_11 (0x0800)
#define BT_12 (0x1000)
#define BT_13 (0x2000)
#define BT_14 (0x4000)
#define BT_15 (0x8000)




#endif /* PINS_H_ */
