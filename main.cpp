
#warning Check clock settings when switching between dev/prod boards!!
#warning SET THE ALTERNATIVE OPTION BYTE!!!! for PWM to work

#include "main.h"
#include "stm8s_gpio.h"
#include "stm8s.h"
#include "SPI_Bitbang.h"
#include "delay.h"
#include "LT8900.h"
#include "uart.h"
#include "utils.c"

//#include "milight.h"

#include <intrinsics.h>

//prototypes
void initHardware(void);
void loop(void);
void sendData(void);
static void RADIO_init(void);
static void TIM1_Config(void);
static void TIM4_Config(void);
static void TestColors(void);

//radio protos
static void handlePacket(uint8_t *buff, uint8_t);
static void BroadcastPresenseRequest(void);
static void RequestStateRequest(void);


//global objs
//the bitbanged SPI port
SPI_BitBang spi(LT8900_PORT_CLK,
                LT8900_PORT_MOSI,
                LT8900_PORT_MISO,
                PIN_BB_CLK, 
                PIN_BB_MOSI, 
                PIN_BB_MISO);

//the radio
LT8900 radio(&spi, 
             LT8900_PORT_SS, 
             LT8900_PORT_PKT, 
             LT8900_PORT_RESET,
             LT8900_PORT_DVSS,
             LT8900_PIN_SS, 
             LT8900_PIN_PKT, 
             LT8900_PIN_RESET,
             LT8900_PIN_DVSS);

uint8_t CURRENT_COLORS[4], SETPOINT_COLORS[4],
TIME_TO_NEW=0;


//Settings
#define BROADCAST_HW_TICK 10000L //iedere 10 sec?

#define RED_COMPARE(x) (TIM1_SetCompare2(gamma8[x]))    //as per pinout measured on rgbw Milight board
#define GREEN_COMPARE(x) (TIM1_SetCompare1(gamma8[x]))
#define BLUE_COMPARE(x) (TIM1_SetCompare4(gamma8[x]))
#define WHITE_COMPARE(x) (TIM1_SetCompare3(gamma8[x]))

#define PACKET_BROADCASTPRESENSE 0
#define PACKET_SETVALUES 1
#define PACKET_REQUESTSTATE 2

unsigned char myID[12];



//node settings
#define NODE_ID 1
#define NODE_TYPE 0

//periodic timer
long volatile tickMs = 0;
long lastTickMs =0;


//#pragma vector=25 //ITC_IRQ_TIM4_OVF
INTERRUPT_HANDLER(TIMER4_UPD_OVF_IRQHandler, ITC_IRQ_TIM4_OVF)
{
  //called every 1ms
  // GPIO_WriteReverse(GPIOB, GPIO_PIN_5); 
  tickMs ++;
  //don't forget to clear our interrupt!
  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
  
}

int main()
{
  initHardware();
#ifdef BREAD
//not supported on stm8s003??
  myUniqueId(myID);
  myID;
#endif
  
  //https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms/
  //R = (pwmIntervals * log10(2))/(log10(255)); // = 12.50882898658681381510320672289928392090409223903489242395

    
  loop();
  return 0;
}


void initHardware(){
  /*
  *     Init System clock to 16mhz
  */
  
  CLK_DeInit();
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1); //cpu prescaler = 1
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); //prescaler 1 , 16mhz  
  
#ifdef CRYSTALTIMER
  //use crystal 12mhz
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, //auto switch
                        CLK_SOURCE_HSE,         //internal osc
                        DISABLE,                //no clk-switch-inter
                        CLK_CURRENTCLOCKSTATE_DISABLE); //disable prev. clk
#endif
  
#ifdef INTERNALTIMER
  // use internal osc
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, //auto switch
                        CLK_SOURCE_HSI,         //internal osc
                        DISABLE,                //no clk-switch-inter
                        CLK_CURRENTCLOCKSTATE_DISABLE); //disable prev. clk
#endif
  
  // Configure Quartz Clock 
  
  
  
  
  /*
  *     Init all ports used 
  */
  GPIO_DeInit(GPIOA);
  //led port
  GPIO_DeInit(GPIOB);
  //sw spi
  GPIO_DeInit(GPIOC);
  //pins lt8900
  GPIO_DeInit(GPIOD);
  
  //configure onboard led:
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
  // GPIO_WriteLow(GPIOB, GPIO_PIN_5);
  //GPIO_WriteHigh(GPIOB, GPIO_PIN_5);
  
  /*
  *     Init Timer 1
  */
  //setup timer1 for pwm generation
  TIM1_Config();
  //setup timer to generate ticks
  TIM4_Config();
  
  //enable interrupts
  //NOTE DELAY FUNCTION depends on tick increments!!
  enableInterrupts();
  
  /*
  *     init SPI
  */
  
  spi.begin();
  
  
  //INIT UART
  //Uart_Init();
  //SerialPutString("Uart init Complete\n"); 
  //sendData();
  
  /*
  *     Init Radio
  */
  RADIO_init();
  
  //init 

  
  //ColorsTest
  TestColors();
  
  //broadcast my presense
  BroadcastPresenseRequest();
  
}

static void RADIO_init(void){
  
  radio.begin();
  radio.setCurrentControl(15, 15);
  radio.setDataRate(LT8900::LT8900_1MBPS);
  radio.setChannel(0x00);
  
  radio.writeRegister(36, 0x147a);
  radio.writeRegister(39, 0x258b);
  radio.writeRegister2(32, 72, 0);    // PL1167's Data Configure Register: LEN_PREAMBLE = 010 -> (0xAAAAAA) 3 bytes, LEN_SYNCWORD = 01 -> 32 bits, LEN_TRAILER = 000 -> (0x05) 4 bits, TYPE_PKT_DAT = 00 -> NRZ law data, TYPE_FEC = 00 -> No FEC
  radio.writeRegister2(41, 176, 0);   // PL1167's Miscellaneous Register: CRC_ON = 1 -> ON, SCR_ON = 0 -> OFF, EN_PACK_LEN = 1 -> ON, FW_TERM_TX = 1 -> ON, AUTO_ACK = 0 -> OFF, PKT_LEVEL = 0 -> PKT active high, CRC_INIT_DAT = 0
  
  
  //verify chip registers.
  for (int i = 0; i <= 50; i++)
  {
    uint16_t value = radio.readRegister(i);
    
    //char str[10] = {0};
    //printf("test");
    //sprintf(str, "%d = %04x\r\n", i, value);
    //SerialPutString((char*)str);
  }
}


static void TIM1_Config(void){
  /*
  *             WARNING, SET THE ALTERNATIVE OPTION BYTE!!!! for PWM to work
  */
  
#define PRESCALER 0
#define TOPVAL 255
#define REPEAT 0
  
#define CCR1_Val  ((uint16_t)0)
#define CCR2_Val  ((uint16_t)0)
#define CCR3_Val  ((uint16_t)0)
#define CCR4_Val  ((uint16_t)0)
  
  
  TIM1_DeInit();
  TIM1_TimeBaseInit(PRESCALER, TIM1_COUNTERMODE_UP, TOPVAL, REPEAT);
  
  
  
  TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
               CCR1_Val, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
               TIM1_OCNIDLESTATE_RESET); 
  
  /*TIM1_Pulse = CCR2_Val*/
  TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE, CCR2_Val,
               TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET, 
               TIM1_OCNIDLESTATE_RESET);
  
  /*TIM1_Pulse = CCR3_Val*/
  TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
               CCR3_Val, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
               TIM1_OCNIDLESTATE_RESET);
  
  /*TIM1_Pulse = CCR4_Val*/
  TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, CCR4_Val, TIM1_OCPOLARITY_LOW,
               TIM1_OCIDLESTATE_SET);
  
  /* TIM1 counter enable */
  TIM1_Cmd(ENABLE);
  
  /* TIM1 Main Output Enable */
  TIM1_CtrlPWMOutputs(ENABLE);
}

static void TIM4_Config(void){
  //see en.CD00190271.pdf SMT8S registers doc
  //page 249
  
  CLK_PeripheralClockConfig (CLK_PERIPHERAL_TIMER4 , ENABLE); 
  TIM4_DeInit(); 
  
  /* Time base configuration */  
#ifdef CRYSTALTIMER
  //CPU on milight board is powerd by 12mhz crystal
  //12.000 / 128 = ~93.75
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 0x5E); //94 DEC -> hex
#endif
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 0x7F); 
#ifdef INTERNALTIMER
  
#endif
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE); 
  TIM4_Cmd(ENABLE);    // Enable TIM4  
}


///
///     Test routine to ease indentifying pwm pin mapping
///
static void TestColors(void){
  for(int x=0;x<255;x++){
    RED_COMPARE(x) ; 
    delay_ms(1);
  }
  RED_COMPARE(0);
  
  for(int x=0;x<255;x++){
    GREEN_COMPARE(x);  
    delay_ms(1);
  }
  GREEN_COMPARE(0);
  
  for(int x=0;x<255;x++){
    BLUE_COMPARE(x) ;
    delay_ms(1); 
  }
  BLUE_COMPARE(0) ;
  
  for(int x=0;x<255;x++){
    WHITE_COMPARE(x);
    delay_ms(1); 
  }
  WHITE_COMPARE(0); 
}

static void BroadcastPresenseRequest(void){
  const int pkgLen = 13;
  uint8_t buff[pkgLen];
  buff[0] = NODE_ID;
  buff[1] = PACKET_BROADCASTPRESENSE;
  for (int x=0;x<12;x++){
    buff[x+2] = myID[x];
  }
  
  for (int x=0;x<10;x++)
    radio.sendPacket(buff,pkgLen);
  
}

static void RequestStateRequest(void){
  uint8_t buff[2];
  buff[0] = NODE_ID;
  buff[1] = PACKET_REQUESTSTATE;
  radio.sendPacket(buff,2);
  
}



///
///     Main program loop
///
void loop(){
  radio.startListening();
  
  while(1){
    if( lastTickMs != tickMs){
      //prevent multiple exec of routine within same tick
      lastTickMs = tickMs;
      
      if(tickMs % BROADCAST_HW_TICK ==0 ){
        //broadcast
        BroadcastPresenseRequest();
        RequestStateRequest();
      }
      
      if(radio.available()){
        
        uint8_t buff[32];
        int pLength = radio.read(buff,32);
        if(pLength >0){
          
          GPIO_WriteReverse(GPIOB, GPIO_PIN_5); 
          handlePacket(buff, pLength);
        }
        else{
          //failed
          
          
        }
        
        radio.startListening();
      }
      
      
      
      //calculate the increment/decrement for the setpoints
      if(TIME_TO_NEW > 0){
        long c = tickMs % (TIME_TO_NEW +1);
        if (c == 0 ){
          for(int x=0; x< 4;x++){
            if (CURRENT_COLORS[x] < SETPOINT_COLORS[x])
              CURRENT_COLORS[x] ++;
            else if (CURRENT_COLORS[x] > SETPOINT_COLORS[x])
              CURRENT_COLORS[x] --;
          }
          
          
          
          
          
        }
      }
      //load compare registers with modified colors
      RED_COMPARE(CURRENT_COLORS[0]);
      GREEN_COMPARE(CURRENT_COLORS[1]);
      BLUE_COMPARE(CURRENT_COLORS[2]);
      WHITE_COMPARE(CURRENT_COLORS[3]);
      
      
    }
  }
}
  
  
  
  
  static void handlePacket(uint8_t *buff, uint8_t pLength){
    uint8_t nodeId = buff[0];
    char action = buff[1];
    
    if(nodeId != NODE_ID){
      //not for us, ignore
      return;
    }
    
    
    switch(action){
    case PACKET_REQUESTSTATE:
      // ignore;
      break;
    case PACKET_BROADCASTPRESENSE:
      //ignore
      break;
    case PACKET_SETVALUES:
      ///
      //copy time to time-change
      TIME_TO_NEW = buff[6];
      
      for(int x = 2; x<6;x++){
        //set colors to future registers  
        if (TIME_TO_NEW == 0){
          //copy colors to CURRENT to immediately latch
          CURRENT_COLORS[x-3] = buff[x]; 
        }
        //copy colors to setpoints
        SETPOINT_COLORS[x-3] = buff[x]; 
      }
      
      break;
      
      default:
        //ignore
        break;
      }
    }
    
    
    
#ifdef USE_FULL_ASSERT
    
    /**
    * @brief  Reports the name of the source file and the source line number
    *   where the assert_param error has occurred.
    * @param file: pointer to the source file name
    * @param line: assert_param error line source number
    * @retval : None
    */
    void assert_failed(u8* file, u32 line)
    { 
      /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
      
      /* Infinite loop */
      while (1)
      {
      }
    }
#endif
    
    