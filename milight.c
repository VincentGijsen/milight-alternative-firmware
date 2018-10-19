
#include "milight.h"
#include "stm8s.h"


#define ID_ONE 1
#define ID_TWO 2
#define ID_TRHEE 3
#define CMD 4
#define VALUE 5

#define RGBW_SETRESET 0
#define WW_SETRESET 1

/**PACKET INDEX
*/

#define IDX_ADRRA 1
#define IDX_ADRRB 2
#define IDX_VAL1 3
#define IDX_VAL2 4
#define IDX_CMD 5 

/**REMOTE CODES

**/
#define CMD_ALL_OFF 0
#define CMD_ALL_ON 1
#define CMD_ALL_NIGHT 0x12

#define CMD_COLOR 0x0f
#define CMD_BRIGHT 0x0e

#define CMD_SMIN 0x0c
#define CMD_M 0x0D
#define CMD_SPLUS 0x0B

#define CMD_GROUP1_ON 3
#define CMD_GROUP1_OFF 4
#define CMD_GROUP1_WHITE 13

#define CMD_GROUP2_ON 5
#define CMD_GROUP2_OFF 6
#define CMD_GROUP2_WHITE 15

#define CMD_GROUP3_ON 7
#define CMD_GROUP3_OFF 8
#define CMD_GROUP3_WHITE 17

#define CMD_GROUP4_ON 9
#define CMD_GROUP4_OFF 0x0A
#define CMD_GROUP4_WHITE 19


uint8_t MILIGHT_COL = 0;
uint8_t MILIGHT_BRIGHT = 0;
uint8_t MILIGHT_CMD = 0;
uint8_t MILIGHT_SEQNR = 0;
uint8_t R =0;
uint8_t G =0;
uint8_t B =0;
uint8_t W =0;




 
/**arduino map impl.*/
uint8_t map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
* Converts an HSL color value to RGB. Conversion formula
* adapted from http://en.wikipedia.org/wiki/HSL_color_space.
* Assumes h, s, and l are contained in the set [0, 1] and
* returns r, g, and b in the set [0, 255].
*
* @param   {number}  h       The hue
* @param   {number}  s       The saturation
* @param   {number}  l       The lightness
* @return  {Array}           The RGB representation
*/

float hue2rgb(float p, float q, float t){
  if(t < 0) t += 1;
  if(t > 1) t -= 1;
  if(t < 1.0/6) return p + (q - p) * 6.0 * t;
  if(t < 1.0/2) return q;
  if(t < 2.0/3) return p + (q - p) * (2.0/3 - t) * 6;
  return p;
}

/**
* Converts an HSL color value to RGB. Conversion formula
* adapted from http://en.wikipedia.org/wiki/HSL_color_space.
* Assumes h, s, and l are contained in the set [0, 1] and
* returns r, g, and b in the set [0, 255].
*
* @param   {number}  h       The hue
* @param   {number}  s       The saturation
* @param   {number}  l       The lightness
* @return  {Array}           The RGB representation
*/
uint8_t hslToRGB(float h, float s, float l, uint8_t color){
  if(s == 0){
    return 0; // achromatic
  }else{
    float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    float p = 2 * l - q;
    switch(color){
    case 0: //r
      return (int)hue2rgb(p, q, h + 1.0/3) * 255;
      break;
    case 1: //g
      return (int) hue2rgb(p, q, h) * 255;
      break;
    case 2:
      return (int)hue2rgb(p, q, h - 1.0/3) * 255;
      break;
      
    }
  }
  
  return 0;
}


void MilightApplication(uint8_t pkg[7]){
  if(pkg[0] == 0xB0){
     uint8_t brightness = 0;
    //we don't check the address for now
    
   
    
    //changes in brighness
    if(pkg[IDX_VAL2] !=MILIGHT_BRIGHT){
      MILIGHT_BRIGHT = pkg[IDX_VAL2];
      uint8_t brightness = 0;
      /*
      *Brighness goes from left 0x90 -> 0 middle ->
      */
      if(MILIGHT_BRIGHT <= 0x90){
        //from upperleft to mid
        brightness = map(MILIGHT_BRIGHT, 0, 0x90, 0, 0x7f); //upperhalf
        //invert value as left > mid!
        brightness = 0x7f - brightness;
        
      }else{
        //from right
        brightness = map(MILIGHT_BRIGHT, 0x90, 0xF9, 0x7F, 0xFF); //upperhalf
        //invert value as mid >right !
        //only left bar on remote is invertd!
        // brightness = 0xFF - brightness;
      }
      //set compare of RED
      R = gamma8[brightness];
      
      //change brightness
     
    }
    
     if(pkg[IDX_VAL1] !=MILIGHT_COL){
      MILIGHT_COL = pkg[IDX_VAL1];
      float col = map(MILIGHT_COL,0,255,0,1000);
      col = col / 1000.0;
      //change color
      uint8_t r = hslToRGB(col,0.5,0.5,0);
      uint8_t g = hslToRGB(col,0.5,0.5,1);
      uint8_t b = hslToRGB(col,0.5,0.5,2);
     
      TIM1_SetCompare2(R);
    }
    
    
    if(pkg[IDX_CMD] != MILIGHT_CMD){
      MILIGHT_CMD = pkg[IDX_CMD];
      //change command
      switch(MILIGHT_CMD){
      case CMD_GROUP1_ON:
        //
        TIM1_SetCompare2(R);
        break;
      case CMD_GROUP1_OFF:
        //
        //RED OFf
        TIM1_SetCompare2(0);
        break;
      case CMD_GROUP1_WHITE:
        //
        break;
      }
      
    }
  }
}