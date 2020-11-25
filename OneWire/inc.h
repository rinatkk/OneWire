#define RCC_APB2Periph_GPIOC            ((uint32_t)0x00000010)
#define GPIO_Pin_13                     ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define LED_High_ODR                    GPIOC->ODR |=  (1<<13)
#define LED_Low_ODR                     GPIOC->ODR &= ~(1<<13)
#define LED_ON_BRR                     GPIOC->BRR |= (1<<13)
#define LED_OFF_BSRR                   GPIOC->BSRR |= (1<<13)

//#define DATA_1_UP(PIN)                  GPIOA->BSRR |= (1<<PIN)
//#define DATA_1_DN(PIN)                  GPIOA->BRR  |= (1<<PIN)
//#define DATA_1_IN(PIN)                  (GPIOA->IDR &  (1<<PIN))
//GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 );
//GPIOA->CRL |= (0x02<<GPIO_CRL_MODE0_Pos) | (0x00 << GPIO_CRL_CNF0_Pos);
#define DATA_1_OUTCONF(PIN)             cs_pin[PIN].gpio->CRL &= ~(0x3U <<PIN*4 | 0x3U << (PIN*4+2)); cs_pin[PIN].gpio->CRL |= (0x02<<PIN*4 | 0x1U << (PIN*4+2))
#define DATA_1_UP(PIN)                  cs_pin[PIN].gpio->BSRR |= cs_pin[PIN].pin
#define DATA_1_DN(PIN)                  cs_pin[PIN].gpio->BRR  |= cs_pin[PIN].pin
#define DATA_1_INCONF(PIN)              cs_pin[PIN].gpio->CRL &= ~(0x3U <<PIN*4 | 0x3U << (PIN*4+2)); cs_pin[PIN].gpio->CRL |= (0x00<<PIN*4 | 0x2U << (PIN*4+2))
#define DATA_1_IN(PIN)                  (cs_pin[PIN].gpio->IDR & cs_pin[PIN].pin)

#define SENS00                          00
#define SENS01                          02

#define F_CPU 		72000000UL
#define TimerTick  	F_CPU/1000-1

int cn = 200;
int cn_sens;
u16 value = 10;
u8 st;
u8 buff_1[10], buff_2[10];
float temp[24];
float temp_res[12];//24
u32 t_send[10];
u8 uart_d[12],
uart_kosh[] =  {'S', 'a', 'e', 'b', 'i', 'z', 'P', 'r', 'o', 'g', '.', ' ', 'v', '0', '.', '0', '®'},
uart_com[] =   {'B', 'a', 's', 't', 'a', 'u', ':', ' ', '1', ' ', ' ', ' ', 'T', 'o', 'k', 't', 'a', ':', ' ', '0'},
uart_wait[] =  {'W', 'a', 'i', 't', '!'},
uart_err[] =   {'K', 'a', 't', 'e', ' ', 'k', 'o', 'm', 'm', 'a', 'n', 'd', 'a', '!'},
uart_start[] = {'S', 'T', 'A', 'R', 'T'},
uart_stop[] =  {'S', 'T', 'O', 'P'},
uart_sec[] =   {'s', 'e', 'c'};
u16 transmit_timer, led_timer;
u16 transmit_cn;
u8 rec_com, pre_com;
u8 rec_cn, rec_err_cn;
u8 sens_num;

typedef struct{
GPIO_TypeDef* gpio;
u16 pin;
}custom_pin;
const custom_pin cs_pin[] = 
{ 
  {GPIOA, 1<<0},  {GPIOA, 1<<1},  {GPIOA, 1<<2},  {GPIOA, 1<<3},
  {GPIOA, 1<<4},  {GPIOA, 1<<5},  {GPIOA, 1<<6},  {GPIOA, 1<<7},
  {GPIOA, 1<<8},  {GPIOA, 1<<11}, {GPIOA, 1<<12}, {GPIOB, 1<<0},
  {GPIOB, 1<<1},  {GPIOB, 1<<5},  {GPIOB, 1<<6},  {GPIOB, 1<<7},  
  {GPIOB, 1<<8},  {GPIOB, 1<<9},  {GPIOB, 1<<10}, {GPIOB, 1<<11},
  {GPIOB, 1<<12}, {GPIOB, 1<<13}, {GPIOB, 1<<14}, {GPIOB, 1<<15}, 
  {GPIOA, 1<<15}, {GPIOB, 1<<3},  {GPIOB, 1<<4},  
  {GPIOC, 1<<13}, {GPIOC, 1<<14}, {GPIOC, 1<<15}
};

int ClockInit(void);
void PortInit(void);
void TIMER_init(void);
void TempMeas(void);
void TempMeas_Num(float *buf);
u8 ds18b20_Reset(u8 pin);
u8 ds18b20_ReadBit(u8 pin);
u8 ds18b20_ReadByte(u8 pin);
void ds18b20_WriteBit(uint8_t bit, u8 pin);
void ds18b20_WriteByte(uint8_t dt, u8 pin);
void USART_init(void);
void USART_TrByte(u8 data);
u8 USART_RcByte();
void USART_TrStr(u8 *buf, int num);
void Blink(void);
void ConvFloat(float f_data, u8 *buf);
void ConvTime(u16 f_data, u8 *buf);
void Systick_init(void);