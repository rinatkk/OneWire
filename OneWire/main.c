#include "stm32f1xx.h"
#include "inc.h"
#include "src.c" 
u32 test_time = 0x40000;





void main(void)
{
 
  //RCC->APB2ENR |= RCC_APB2Periph_GPIOC;
  ClockInit();
  PortInit();
  TIMER_init();
  TIM3->CR1 = TIM_CR1_CEN;
//  USART_init();
//  Systick_init();
  //USART1->DR = 10;	// Send Byte!
//  USART_TrStr(uart_kosh, 17);
//  USART_TrByte('\r');
//  USART_TrStr(uart_com, 20);
//  USART_TrByte('\r');
//  USART_TrStr(uart_wait, 5);
//  USART_TrByte('\r');
//  USART_TrByte('\r');
  
  //cs_pin[24].gpio->BRR = cs_pin[24].pin;

  while(1)
  {
    TempMeas_Num(temp_res);
//    LED_ON_BRR;
//    LED_OFF_BSRR;
//    DATA_1_OUTCONF(0);
//    DATA_1_UP(0);
//    DATA_1_DN(0);
//    DATA_1_INCONF(0);
//    st = DATA_1_IN(0);

    
    
    
    
//    if(rec_com == 0x31)
//    {
//      if(transmit_timer>=1000)
//      {
//        transmit_timer=0;
//        transmit_cn++;
//        TempMeas_Num(temp_res);
//        //for(int cn=0; cn<12; cn++){uart_d[cn]=0;}
//        ConvTime(transmit_cn, uart_d);
//        USART_TrStr(uart_d, 4);
//        USART_TrStr(uart_sec, 3);
//        USART_TrByte('\t');
//        for(sens_num=0; sens_num<24; sens_num++)
//        {
//          for(int cn=0; cn<12; cn++){uart_d[cn]=0;}
//          ConvFloat(temp_res[sens_num], uart_d);
//          USART_TrStr(uart_d, 7);
//          USART_TrByte(' ');
//        }
//        USART_TrByte('\r');
//        
//        
//        //Blink();
//      }
//      if(pre_com!=0x31) { pre_com = rec_com;}
//    }
//    if(rec_com == 0x30 && pre_com == 0x31)
//    {
//      transmit_cn = 0;
//      LED_High_BSRR;
//      USART_TrByte('\r');
//      USART_TrStr(uart_stop, 4);
//      USART_TrByte('\t');
//      USART_TrStr(uart_com, 20);
//      USART_TrByte('\r');
//      USART_TrByte('\r');
//      if(pre_com!=0x30) { pre_com = rec_com;}
//      //pre_com = rec_com;
//    }
//    
//    if(rec_com != 0x00 && rec_com != 0x31)
//    {
//      if(rec_com!=0x30)
//      {
//        if(rec_err_cn != rec_cn) 
//        {
//          rec_err_cn = rec_cn;
//          //pre_com = rec_com;
//          if(pre_com == 0x31)
//          {
//            rec_com = pre_com;
//            //if(pre_com == 0x30) {USART_TrByte('\r');}
//            USART_TrStr(uart_err, 14);
//            USART_TrByte('\t');
//            USART_TrStr(uart_com, 20);
//            USART_TrByte('\r');
//            if(pre_com == 0x30) {USART_TrByte('\r');}
//          }
//          else
//          {
//            pre_com = rec_com;
//            //USART_TrByte('\r');
//            USART_TrStr(uart_err, 14);
//            USART_TrByte('\t');
//            USART_TrStr(uart_com, 20);
//            USART_TrByte('\r');
//            USART_TrByte('\r');
//          }
//        }
//      }
//    }
//    //temp[1] = TempMeas_Num(buff_2, SENS02);
  }
}