void Blink(void)
{
  if(GPIOC->ODR & (1<<13)) LED_ON_BRR;
    else LED_OFF_BSRR;
}
//------------------------------------------------------------------------------
int ClockInit(void)
{
  RCC->CR |=(1<<RCC_CR_HSEON_Pos);
  int StartUpCounter;
  for(StartUpCounter=0;;StartUpCounter++){
    if(RCC->CR & (1<<RCC_CR_HSERDY_Pos))
      break;
    if(StartUpCounter > 0x1000){
      RCC->CR &= ~(1<<RCC_CR_HSEON_Pos);
      return 1;
    }
  }
  RCC->CFGR |= (0x07<<RCC_CFGR_PLLMULL_Pos)
            | (0x01<<RCC_CFGR_PLLSRC_Pos);
  RCC->CR |= (1<<RCC_CR_PLLON_Pos);
  for(StartUpCounter=0; ; StartUpCounter++){
    if(RCC->CR & (1<<RCC_CR_PLLRDY_Pos))
      break;
    if(StartUpCounter > 0x1000){
      RCC->CR &= ~(1<<RCC_CR_HSEON_Pos);
      RCC->CR &= ~(1<<RCC_CR_PLLON_Pos);
      return 2;
    }
  }
  FLASH->ACR |= (0x02<<FLASH_ACR_LATENCY_Pos);
  RCC->CFGR  |= (0x00<<RCC_CFGR_PPRE2_Pos)|(0x04<<RCC_CFGR_PPRE1_Pos)
             |(0x00<<RCC_CFGR_HPRE_Pos);
  RCC->CFGR |= (0x02<<RCC_CFGR_SW_Pos);
  while((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0x02<<RCC_CFGR_SWS_Pos)){}
  RCC->CR &= ~(1<<RCC_CR_HSION_Pos);
  return 0;
}
//------------------------------------------------------------------------------
void PortInit(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

  GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 );
  
    
  GPIOA->CRL |= (0x02<<GPIO_CRL_MODE0_Pos) | (0x00 << GPIO_CRL_CNF0_Pos);

  GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
  GPIOC->CRH |= (0x03<<GPIO_CRH_MODE13_Pos) | (0x01 << GPIO_CRH_CNF13_Pos);
  
  RCC->APB2ENR  |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
}
//------------------------------------------------------------------------------
void TIMER_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC    = F_CPU/1000000-1; 
  //RCC_TIM3_EN = ON;
  //TIM03->TIMx_PSC.PSC = 7;
  //TIM03->TIMx_CR1.DIR = 1;
  //TIM03->TIMx_CR1.OPM = 1;
}
//------------------------------------------------------------------------------
void Systick_init(void)
{
  SysTick->LOAD = TimerTick;
  SysTick->VAL  = TimerTick;
  
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk|
                  SysTick_CTRL_TICKINT_Msk|
                  SysTick_CTRL_ENABLE_Msk;
}
//------------------------------------------------------------------------------
void SysTick_Handler(void)
{
  transmit_timer++;
  led_timer++;
  if(led_timer>=500 && rec_com==0x31) {Blink(); led_timer=0;}
}
//------------------------------------------------------------------------------
void USART1_IRQHandler(void)
{
  if((USART1->SR & USART_SR_RXNE)!=0)
  {
    rec_com = USART1->DR;
    rec_cn++;
  }
}

//------------------------------------------------------------------------------
void USART_init(void)
{
  RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;			// USART1 Clock ON
  NVIC_EnableIRQ (USART1_IRQn); 
  USART1->BRR 	= 0x271;					// Bodrate for 115200 on 72Mhz
  USART1->CR1 	|= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;	// USART1 ON, TX ON, RX ON						
}
//------------------------------------------------------------------------------
void USART_TrByte(u8 data)
{
  while(!(USART1->SR&USART_SR_TXE));
  USART1->SR &= ~USART_SR_TC;
  USART1->DR = data;
}
//------------------------------------------------------------------------------
u8 USART_RvByte()
{
  while(!(USART1->SR&USART_SR_RXNE));
  return USART1->DR;
}
//------------------------------------------------------------------------------
void USART_TrStr(u8 *buf, int num)
{
  for(int i=0; i<num; i++)
  {
    USART_TrByte(*(buf+i));
  }
  //USART_TrByte('\t');
  //USART_TrByte('\n');
  //USART_TrByte('\r');
}
//------------------------------------------------------------------------------
void ConvFloat(float f_data, u8 *buf)
{
  if(f_data<0) {*buf = '-';} else {*buf = '+';}
  
    //*(buf+1) =     0x30 + ((u16)f_data)/100;
    *(buf+1) = 0x30 + ((u16)f_data)/10;
    *(buf+2) = 0x30 + ((u16)f_data)%10;
    *(buf+3) = '.';
    *(buf+4) = 0x30 + ((u16)(f_data*1000)/100)%10;
    *(buf+5) = 0x30 + ((u16)(f_data*1000)/10)%10;
    *(buf+6) = 0x30 + ((u16)(f_data*1000))%10;    
}
//------------------------------------------------------------------------------
void ConvTime(u16 f_data, u8 *buf)
{
    *(buf+0) = 0x30 + (f_data/1000)%10;
    *(buf+1) = 0x30 + (f_data/100)%10;
    *(buf+2) = 0x30 + (f_data/10)%10;
    *(buf+3) = 0x30 + f_data%10;
}
//------------------------------------------------------------------------------
void TempMeas(void)
{
  st = ds18b20_Reset(2);
  ds18b20_WriteByte(0xCC, 2);      //разрешить доступ к датчику не использу€ адрес
  ds18b20_WriteByte(0x44, 2);      //запустить преобразование
  for(cn=0; cn<value; cn++)
  {
    TIM3->CNT = 0;
    while(TIM3->CNT < value){}
  }
  st = ds18b20_Reset(2);          //послать импульс сброса              
  ds18b20_WriteByte(0xCC, 2);      //разрешить доступ к датчику не использу€ адрес
  ds18b20_WriteByte(0xBE,2 );      //команда, заставл€юща€ датчик выдать 9 байт своей пам€ти
  for(cn=0; cn<9; cn++ )           //прочитать 9 байт в массив
    buff_1[cn] = ds18b20_ReadByte(2);
  temp[0] = (float)((buff_1[1]<<8)|(buff_1[0]))/16;
}
//------------------------------------------------------------------------------
void TempMeas_Num(float *buf)
{
//  for(sens_num=0; sens_num<1; sens_num++)
//  {
//    st = ds18b20_Reset(sens_num);
//    ds18b20_WriteByte(0xCC, sens_num);
//    ds18b20_WriteByte(0x44, sens_num);
//  }
//  for(cn=0; cn<value; cn++)
//  {
//    TIM3->CNT = 0;
//    while(TIM3->CNT < value){}
//  }
  
//  for(int j=0; j<=600; j++)
//  {
//    TIM3->CNT = 0;
//    while(TIM3->CNT <1000){} //задержка 
//  }
//  for(sens_num=0; sens_num<1; sens_num++)
//  {
  u8 cn_beep=0;
    st = ds18b20_Reset(sens_num);
  if(!st)
    
  {
    ds18b20_WriteByte(0x33, sens_num);
    TIM3->CNT = 0;
    while(TIM3->CNT <1000){} //задержка
    for(cn=0; cn<9; cn++ )           //прочитать 9 байт в массив
      buff_1[cn] = ds18b20_ReadByte(sens_num);
    while(buff_1[0] != 0x01 )
    {
      cn_beep++;
      st = ds18b20_Reset(sens_num);
      ds18b20_WriteByte(0x33, sens_num);
      TIM3->CNT = 0;
      while(TIM3->CNT <1000){} //задержка
      for(cn=0; cn<9; cn++ )           //прочитать 9 байт в массив
        buff_1[cn] = ds18b20_ReadByte(sens_num);
      if(cn_beep>=5)
      {
        LED_OFF_BSRR;
        cn_beep=0;
        break;
      }
    }
    if(buff_1[0] == 0x01)
    {
      LED_ON_BRR;
      for(int j=0; j<=100; j++)
      {
        TIM3->CNT = 0;
        while(TIM3->CNT <1000){} //задержка 
      }
    }
      
    st = 1;
    
  }
  else
  {
    LED_OFF_BSRR;
  }
//    ds18b20_WriteByte(0xCC, sens_num);
//    ds18b20_WriteByte(0xBE, sens_num);
    
    
    
//    ds18b20_WriteByte(0x01, sens_num);
//    for(cn=0; cn<9; cn++ )           //прочитать 9 байт в массив
//      buff_1[cn] = ds18b20_ReadByte(sens_num);
//    buff_1[0] = ds18b20_ReadByte(sens_num);
//    buff_1[1] = ds18b20_ReadByte(sens_num);
    
//    if(buff_1[1]&0xf0)
//    {
//      buff_1[0] = ~buff_1[0];
//      buff_1[1] = ~buff_1[1];
//      *(buf+sens_num)= -(float)((buff_1[1]<<8)|buff_1[0])/16;
//    }
    *(buf+sens_num) = (float)((buff_1[1]<<8)|buff_1[0])/16;
    
//  }

}
//------------------------------------------------------------------------------
u8 ds18b20_Reset(u8 pin)
{
  u16 status;
  
  DATA_1_OUTCONF(pin);
  TIM3->CNT = 0;
  DATA_1_DN(pin);//низкий уровень
  while(TIM3->CNT <485){} //задержка как минимум на 480 микросекунд
  DATA_1_UP(pin);//высокий уровень
  while(TIM3->CNT <(485+65)){} //задержка как минимум на 60 микросекунд
  
  DATA_1_INCONF(pin);
  status = DATA_1_IN(pin);//провер€ем уровень
  while(TIM3->CNT <(485+65+500)){}//задержка как минимум на 480 микросекунд
  //(на вс€кий случай подождЄм побольше, так как могут быть неточности в задержке)
  return (status ? 1 : 0);//вернЄм результат
}
//------------------------------------------------------------------------------
u8 ds18b20_ReadBit(u8 pin)
{
  uint8_t bit = 0;
  TIM3->CNT = 0;
  
  DATA_1_OUTCONF(pin);
  DATA_1_DN(pin);//низкий уровень
  while(TIM3->CNT <2){}
  DATA_1_UP(pin);//высокий уровень
  while(TIM3->CNT <(2+13)){}
//  while(TIM3->CNT <(2+9)){}
  
  DATA_1_INCONF(pin);
  bit = (DATA_1_IN(pin) ? 1 : 0);//провер€ем уровень
  while(TIM3->CNT <(2+13+45)){}
//  while(TIM3->CNT <(2+13+70)){}
  return bit;
}
//------------------------------------------------------------------------------
u8 ds18b20_ReadByte(u8 pin)
{
  uint8_t data = 0;
  for (uint8_t i = 0; i <= 7; i++)
  data += ds18b20_ReadBit(pin) << i;
  return data;
}
//------------------------------------------------------------------------------
void ds18b20_WriteBit(uint8_t bit, u8 pin)
{
  
  DATA_1_OUTCONF(pin);
  TIM3->CNT = 0;
  DATA_1_DN(pin);
  if(!bit) 
  {
    while(TIM3->CNT <65){}
    DATA_1_UP(pin);
    while(TIM3->CNT <(65+3)){}
//    while(TIM3->CNT <(65+10)){}
  }
  else 
  {
    while(TIM3->CNT <3){}
//    while(TIM3->CNT <10){}
    DATA_1_UP(pin);
    while(TIM3->CNT <(3+65)){}
  }
}
//------------------------------------------------------------------------------
void ds18b20_WriteByte(uint8_t dt, u8 pin)

{
  for (uint8_t i = 0; i < 8; i++)
  {
    TIM3->CNT = 0;
    ds18b20_WriteBit(dt >> i & 1, pin);
    //Delay Protection
    while(TIM3->CNT <5){}
  }
}
//------------------------------------------------------------------------------