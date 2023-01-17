#ifndef GESTPWM_H
#define GESTPWM_H
/*--------------------------------------------------------*/
// GestPWM.h
/*--------------------------------------------------------*/
//  Description :   PWM Management 
//                  For TP1 2022-2023
//
//	Auteur 		: 	A. Steffen & J. Chafla
//
//	Version		:	V1.1
//	Compilateur	:	XC32 V5.45 + Harmony 2.06
//
/*--------------------------------------------------------*/

#include <stdint.h>

#include "app.h"

#include "system_config.h"
#include "system_definitions.h"
#include "bsp.h"

#include "Mc32DriverAdc.h"
#include "Mc32DriverLcd.h"
#include "peripheral/oc/plib_oc.h"


//#define PERIODEMILLIEU 1125             // = PERIODEMAX - PERIODEMIN
#define ADC_RES 1023

#define MAX_SERVO 11999
#define MIN_SERVO 2999
#define SERVO_RANGE 180
#define SERVO_OFFSET 90

#define TIMER2_MOTOR_DC 2000
#define MOTOR_DC_BAND 99

#define AVERAGE_SIZE 10



/*--------------------------------------------------------*/
// Function Prototype
/*--------------------------------------------------------*/

typedef struct {
    uint8_t absSpeed;    // Absolute Speed 0 to 99
    uint8_t absAngle;    // Angle  0 to 180°
    int8_t SpeedSetting; // Defined Speed -99 à +99
    int8_t AngleSetting; // Defined Angle  -90° à +90°
} S_pwmSettings;

extern S_pwmSettings PWMData; 

void GPWM_Initialize(S_pwmSettings *pData);

// Functions use a pointer from the structure S_pwmSettings
void GPWM_GetSettings(S_pwmSettings *pData);    //Get Speed and Angle
void GPWM_DispSettings(S_pwmSettings *pData, int Remote);   //Update Display
void GPWM_ExecPWM(S_pwmSettings *pData);        // PWM execution and Motor management
void GPWM_ExecPWMSoft(S_pwmSettings *pData);    // Software PWM execution

uint16_t Avg_ADC_Value(uint16_t Avg_Table[]);

#endif
