#ifndef __ARM_TOOLS_H
#define __ARM_TOOLS_H

#include "include.h"
#include "Data_analysis.h"
#include "arm_user.h"
#include <stdint.h>


#define TOOL2MOTOR     0.0f;               /* 8191.0f * 36.0f / 360.0f; */
#define CLAMP_TANGLE   0.0f *  TOOL2MOTOR;
#define CHUCK_TANGLE 180.0f *  TOOL2MOTOR;

#define CLAMP_X_OFFEST 0.0f
#define CLAMP_Y_OFFEST 0.0f 
#define CLAMP_Z_OFFEXT 0.0f

#define CLAMP_OPEN     1U
#define CLAMP_CLOSE    0U

#define CHUCK_X_OFFEST 0.0f
#define CHUCK_Y_OFFEST 0.0f
#define CHUCK_Z_OFFEXT 0.0f

#define CHUCK_OPEN     1U
#define CHUCK_CLOSE    0U

typedef struct{
    uint8_t state[2];//0: close, 1: open
    uint8_t flag[2]; //0: USART, 1: USB
    float real_angle;
    float target_angle;
    uint8_t safe_flag; //0: unsafe, 1: safe

  
}clamp_Handle_t;

typedef struct{
    uint8_t state[2];//0: close, 1: open
    uint8_t flag[2]; //0: USART, 1: USB
    float real_angle;
    float target_angle;
    uint8_t safe_flag; //0: unsafe, 1: safe

}chuck_Handle_t;







#endif /* __ARM_TOOLS_H */