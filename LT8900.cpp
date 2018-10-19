
#include "LT8900.h"

#define _BV(bit) (1 << (bit)) 


#define REGISTER_READ       0x80  
#define REGISTER_WRITE      0x00  
#define REGISTER_MASK       0x7f  

#define R_CHANNEL           7
#define CHANNEL_RX_BIT      7
#define CHANNEL_TX_BIT      8
#define CHANNEL_MASK        0x7f  ///bin
#define DEFAULT_CHANNEL     9

#define R_CURRENT           9
#define CURRENT_POWER_SHIFT 12
#define CURRENT_POWER_MASK  0xf000
#define CURRENT_GAIN_SHIFT  7
#define CURRENT_GAIN_MASK   0x0780

/* LT8910S only */
#define R_DATARATE          44
#define DATARATE_MASK       0x00FF
#define DATARATE_1MBPS      0x0100
#define DATARATE_250KBPS    0x0400
#define DATARATE_125KBPS    0x0800
#define DATARATE_62KBPS     0x1000

#define R_SYNCWORD1         36
#define R_SYNCWORD2         37
#define R_SYNCWORD3         38
#define R_SYNCWORD4         39

#define R_PACKETCONFIG      41
#define PACKETCONFIG_CRC_ON             0x8000
#define PACKETCONFIG_SCRAMBLE_ON        0x4000
#define PACKETCONFIG_PACK_LEN_ENABLE    0x2000
#define PACKETCONFIG_FW_TERM_TX         0x1000
#define PACKETCONFIG_AUTO_ACK           0x0800
#define PACKETCONFIG_PKT_FIFO_POLARITY  0x0400

#define R_STATUS            48
#define STATUS_CRC_BIT      15


#define R_FIFO              50
#define R_FIFO_CONTROL      52

#define CS_LOW GPIO_WriteLow(_port_ss, (GPIO_Pin_TypeDef) _pin_ss)
#define CS_HIGH GPIO_WriteHigh(_port_ss, (GPIO_Pin_TypeDef) _pin_ss)

LT8900::LT8900(SPI_BitBang * spi, 
         GPIO_TypeDef * port_ss, 
         GPIO_TypeDef * port_pkt, 
         GPIO_TypeDef * port_reset, 
         GPIO_TypeDef * port_dvss,
         const uint8_t ss, 
         const uint8_t pkt, 
         const uint8_t rst,
         const uint8_t dvss)
{
  _spi = spi;
  _port_ss = port_ss;
  _port_pkt = port_pkt;
  _port_reset = port_reset;
  _port_dvss = port_dvss;
  _pin_ss = ss;
  _pin_pkt = pkt;
  _pin_reset = rst;
  _pin_dvss=dvss;
  
  
  _channel = DEFAULT_CHANNEL;
  _isLT8910 = false;
  
  
}

void LT8900::setChannel(uint8_t channel)
{
  _channel = channel;
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK));
}

bool LT8900::setDataRate(DataRate rate)
{
  uint16_t newValue;
  
  switch (rate)
  {
  case LT8900_1MBPS:
    newValue = DATARATE_1MBPS;
    break;
  case LT8910_250KBPS:
    newValue = DATARATE_250KBPS;
    break;
  case LT8910_125KBPS:
    newValue = DATARATE_125KBPS;
    break;
  case LT8910_62KBPS:
    newValue = DATARATE_62KBPS;
    break;
  default:
    return false;
  }
  
  writeRegister(R_DATARATE, newValue);
  
  //verify
  return (readRegister(R_DATARATE) == newValue);
}

bool LT8900::sendPacket(uint8_t *data, uint8_t packetSize)
{
  if (packetSize < 1 || packetSize > 255)
  {
    return false;
  }

  writeRegister(R_CHANNEL, 0x0000);
  writeRegister(R_FIFO_CONTROL, 0x8000);  //flush tx

  //packets are sent in 16bit words, and the first word will be the packet size.
  //start spitting out words until we are done.

  uint8_t pos = 0;
  writeRegister2(R_FIFO, packetSize, data[pos++]);
  while (pos < packetSize)
  {
    uint8_t msb = data[pos++];
    uint8_t lsb = data[pos++];

    writeRegister2(R_FIFO, msb, lsb);
  }

  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) | _BV(CHANNEL_TX_BIT));   //enable RX

  //Wait until the packet is sent.
  while (GPIO_ReadInputPin(_port_pkt,(GPIO_Pin_TypeDef)_pin_pkt)== 0)
  {
      //do nothing.
  }

  return true;
}


LT8900::DataRate LT8900::getDataRate()
{
  uint16_t value = readRegister(R_DATARATE) & DATARATE_MASK;
  switch (value)
  {
  case DATARATE_1MBPS:
    return LT8900_1MBPS;
  case DATARATE_250KBPS:
    return LT8910_250KBPS;
  case DATARATE_125KBPS:
    return LT8910_125KBPS;
  case DATARATE_62KBPS:
    return LT8910_62KBPS;
  }
  return LT8900_1MBPS;
}

int LT8900::read(uint8_t *buffer, uint8_t maxBuffer)
{
  uint16_t value = readRegister(R_STATUS);
  uint16_t mask = (uint16_t)(1<< STATUS_CRC_BIT);
  if ((value & mask) == 0)
  {
    //CRC ok
    
    uint16_t data = readRegister(R_FIFO);
    uint8_t packetSize = data >> 8;
    if(maxBuffer < packetSize+1)
    {
      //BUFFER TOO SMALL
      return -2;
    }
    
    uint8_t pos=0;
    buffer[pos++] = (data & 0xFF);
    while (pos < packetSize)
    {
      data = readRegister(R_FIFO);
      buffer[pos++] = data >> 8;
      buffer[pos++] = data & 0xFF;
    }
    
    return packetSize;
  }
  else
  {
    //CRC error
    return -1;
  }
}


bool LT8900::available()
{
  //read the PKT_FLAG state; this can also be done with a hard wire.
  if(GPIO_ReadInputPin(_port_pkt,(GPIO_Pin_TypeDef)_pin_pkt)>0)
  {
    return true;
  }
  return false;
}

void LT8900::startListening()
{
  writeRegister(R_CHANNEL, _channel & CHANNEL_MASK);   //turn off rx/tx
  delay_ms(3);
  writeRegister(R_FIFO_CONTROL, 0x0080);  //flush rx
  writeRegister(R_CHANNEL,  (_channel & CHANNEL_MASK) | _BV(CHANNEL_RX_BIT));   //enable RX
  delay_ms(5);
}



void LT8900::setSyncWord(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
  
  writeRegister(R_SYNCWORD1, a);
  writeRegister(R_SYNCWORD2, b);
  writeRegister(R_SYNCWORD3, c);
  writeRegister(R_SYNCWORD4, d);
  writeRegister2(32, 72, 0);    // PL1167's Data Configure Register: LEN_PREAMBLE = 010 -> (0xAAAAAA) 3 bytes, LEN_SYNCWORD = 01 -> 32 bits, LEN_TRAILER = 000 -> (0x05) 4 bits, TYPE_PKT_DAT = 00 -> NRZ law data, TYPE_FEC = 00 -> No FEC

  
  
  //https://github.com/pmoscetta/authometion-milight/blob/master/Authometion-MiLight/MiLight.cpp
  
}

uint16_t LT8900::readRegister(uint8_t reg)
{
  CS_LOW;
  
  _spi->transfer(REGISTER_READ | (REGISTER_MASK & reg));
  uint8_t high =  _spi->transfer(0x00);
  uint8_t low = _spi->transfer(0x00);
  
  CS_HIGH;
  
  uint16_t res = (high << 8 | low);
  return res;
}

uint8_t LT8900::writeRegister(uint8_t reg, uint16_t data)
{
  uint8_t high = data >> 8;
  uint8_t low = data & 0xFF;
  
  return writeRegister2(reg, high, low);
}

uint8_t LT8900::writeRegister2(uint8_t reg, uint8_t high, uint8_t low)
{
  //_spi->setDataMode(SPI_BitBang::MODE1);
  CS_LOW;
  __delay_cycles(50);
  uint8_t r = _spi->transfer(REGISTER_WRITE | (REGISTER_MASK & reg));
  _spi->transfer(high);
  _spi->transfer(low);
   __delay_cycles(50);
  CS_HIGH;
  //todo
  return r;
}


void LT8900::setCurrentControl(uint8_t power, uint8_t gain)
{
  writeRegister(R_CURRENT,
                ((power << CURRENT_POWER_SHIFT) & CURRENT_POWER_MASK) |
                  ((gain << CURRENT_GAIN_SHIFT) & CURRENT_GAIN_MASK));
}

void LT8900::status(){
  uint16_t mode = readRegister(R_CHANNEL);
  uint8_t tx = ((mode & _BV(CHANNEL_TX_BIT)) != false);
  uint8_t rx = (( mode & _BV(CHANNEL_RX_BIT)) != false);
  uint8_t channel = ((mode & CHANNEL_MASK));
  
  uint16_t state = readRegister(R_STATUS);
  bool crc_error = (state & _BV(15));
  bool fec23_error = state & _BV(14);
  uint8_t framer_st = (state & 0x3f00) >> 8;
  bool pkt_flag = state & _BV(6);
  bool fifo_flag = state & _BV(5);
  
  
  delay_us(1);
  
}



void LT8900::begin(){
  //the outputs
  GPIO_Init(_port_ss, (GPIO_Pin_TypeDef)_pin_ss, GPIO_MODE_OUT_PP_LOW_FAST);
  
   
  //the inputs
  GPIO_Init(_port_pkt, (GPIO_Pin_TypeDef)_pin_pkt, GPIO_MODE_IN_FL_NO_IT);
  //drive CS high for inactive 
  CS_HIGH;

  if(_pin_dvss){
    GPIO_Init(_port_dvss, (GPIO_Pin_TypeDef)_pin_dvss, GPIO_MODE_OUT_PP_LOW_FAST);
    //wired on the board..
    GPIO_WriteLow(_port_dvss, (GPIO_Pin_TypeDef) _pin_dvss);
  }

  if(_pin_reset > 0)
  {
    GPIO_Init(_port_reset, (GPIO_Pin_TypeDef)_pin_reset, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_WriteLow(_port_reset, (GPIO_Pin_TypeDef) _pin_reset);
    //digitalWrite(_pin_reset, LOW);
    delay_ms(400);
    GPIO_WriteHigh(_port_reset, (GPIO_Pin_TypeDef) _pin_reset);
    //digitalWrite(_pin_reset, HIGH);
    delay_ms(400);
  }
  

  //wait for rq to settle
  
  writeRegister(0, 0x6fe0);
  writeRegister(1, 0x5681);
  writeRegister(2, 0x6617);
  writeRegister(4, 0x9cc9);    //why does this differ from powerup (5447)
  writeRegister(5, 0x6637);    //why does this differ from powerup (f000)
  writeRegister(8, 0x6c90);    //power (default 71af) UNDOCUMENTED

  setCurrentControl(4, 0);     // power & gain.

  writeRegister(10, 0x7ffd);   //bit 0: XTAL OSC enable
  writeRegister(11, 0x0000);   //bit 8: Power down RSSI (0=  RSSI operates normal)
  writeRegister(12, 0x0000);
  writeRegister(13, 0x48bd);   //(default 4855)

  writeRegister(22, 0x00ff);
  writeRegister(23, 0x8005);  //bit 2: Calibrate VCO before each Rx/Tx enable
  writeRegister(24, 0x0067);
  writeRegister(25, 0x1659);
  writeRegister(26, 0x19e0);
  writeRegister(27, 0x1300);  //bits 5:0, Crystal Frequency adjust
  writeRegister(28, 0x1800);

  //fedcba9876543210
  writeRegister(32, 0x5000);  //AAABBCCCDDEEFFFG  A preamble length, B, syncword length, c trailer length, d packet type
  //                  E FEC_type, F BRCLK_SEL, G reserved
  //0x5000 = 0101 0000 0000 0000 = preamble 010 (3 bytes), B 10 (48 bits)
  writeRegister(33, 0x3fc7);
  writeRegister(34, 0x2000);  //
  writeRegister(35, 0x0300);  //POWER mode,  bit 8/9 on = retransmit = 3x (default)

  //setSyncWord(0x03805a5a03800380);

  writeRegister(40, 0x4401);  //max allowed error bits = 0 (01 = 0 error bits)
  writeRegister(R_PACKETCONFIG,
      PACKETCONFIG_CRC_ON |
      PACKETCONFIG_PACK_LEN_ENABLE |
      PACKETCONFIG_FW_TERM_TX
  );

  writeRegister(42, 0xfdb0);
  writeRegister(43, 0x000f);

  //check the variant.
  _isLT8910 = setDataRate(LT8910_62KBPS);
  //return to defaults.
  setDataRate(LT8900_1MBPS);

  //15:8, 01: 1Mbps 04: 250Kbps 08: 125Kbps 10: 62.5Kbps

  writeRegister(R_FIFO, 0x0000);  //TXRX_FIFO_REG (FIFO queue)

  writeRegister(R_FIFO_CONTROL, 0x8080); //Fifo Rx/Tx queue reset

  delay_ms(200);
  writeRegister(R_CHANNEL, _BV(CHANNEL_TX_BIT));  //set TX mode.  (TX = bit 8, RX = bit 7, so RX would be 0x0080)
  delay_ms(2);
  writeRegister(R_CHANNEL, _channel);  // Frequency = 2402 + channel
}



