/*--------------------------------------------------------*/
// GestPWM.c
/*--------------------------------------------------------*/
//	Description :	PWM Management 
//			        For TP1 2022-2023
//
//	Auteur 		: 	A. Steffen & J. Chafla
//
//	Version		:	V1.1
//	Compilateur	:	XC32 V5.45 + Harmony 2.06
//
/*--------------------------------------------------------*/

#include "GestPWM.h"

S_pwmSettings PWMData, PWMDataToSend ;  //For settings values
S_ADCResults AdcResult;
APP_DATA appData;

void GPWM_Initialize(S_pwmSettings *pData)
{
   //Init datas 
    pData -> AngleSetting = 0;
    pData -> SpeedSetting = 0;
    pData -> absAngle = 0;
    pData -> absSpeed = 0;
    
    //Init H Bridge
    BSP_EnableHbrige();
    
   //Init timers 1 to 4
   DRV_TMR0_Start();
   DRV_TMR1_Start();
   DRV_TMR2_Start();
   //DRV_TMR3_Start();
   
   //Init OC 2 and 3
   DRV_OC0_Start();
   DRV_OC1_Start(); 
   
   STBY_HBRIDGE_W = 1;  //H Bridge in standby, always at 1
}

// Get values for speed and angle 
void GPWM_GetSettings(S_pwmSettings *pData)	
{   
    static uint16_t Table_Avg_CH0[AVERAGE_SIZE];    //Table for rolling average
    static uint16_t Table_Avg_CH1[AVERAGE_SIZE];    //Table for rolling average

    uint16_t Avg_ADC_CH0, Avg_ADC_CH1;  //Output average values 
    uint8_t Val_Conv;
    
    AdcResult = BSP_ReadAllADC();  //Read all potentiometers

    Table_Avg_CH0[0] = AdcResult.Chan0;
    Table_Avg_CH1[0] = AdcResult.Chan1;
    
    Avg_ADC_CH0 = Avg_ADC_Value(Table_Avg_CH0); //Call function to get a rolling average
    Avg_ADC_CH1 = Avg_ADC_Value(Table_Avg_CH1); //Call function to get a rolling average   

    /******---Conversion---***********/
    pData->absSpeed = abs((Avg_ADC_CH0*MOTOR_DC_ABS)/ADC_RES-MOTOR_DC_BAND); //non-signed conversion: 0 to 99
    pData->SpeedSetting = (signed)((Avg_ADC_CH0*MOTOR_DC_ABS)/ADC_RES-MOTOR_DC_BAND); //signed conversion: -99 to 99
   

    pData->absAngle = (Avg_ADC_CH1*SERVO_RANGE)/ADC_RES;    //Raw value from 0 to 180
    pData->AngleSetting = (int8_t)pData->absAngle - SERVO_OFFSET;  //Signed value from -90 to 90
    
}

void GPWM_DispSettings(S_pwmSettings *pData, int Remote) //Display settings on LCD
{
    lcd_gotoxy(1, 1);
    if (Remote == REMOTE)
    {
        printf_lcd("Remote Settings");
    }
    else
    {
        printf_lcd("Local Settings");
    }
    
    lcd_gotoxy(1, 2);
    if(pData -> SpeedSetting >= 0)  //Check if value is positive/negative
    {
        printf_lcd("Speed Setting   +%2d", pData -> SpeedSetting);
    }
    else
    {
        printf_lcd("Speed Setting   %3d", pData -> SpeedSetting);
    }
    
    lcd_gotoxy(1, 3);
    printf_lcd("Absolute Speed   %2d", pData -> absSpeed ); 

    lcd_gotoxy(1, 4);
    printf_lcd("Angle           %3d", pData -> AngleSetting );
}

// Execute PWM and DC motor rotation from the structure
void GPWM_ExecPWM(S_pwmSettings *pData)
{
    if(pData->SpeedSetting > 0)     //Rotation clockwise
    {
        STBY_HBRIDGE_W = 1;
        AIN1_HBRIDGE_W = 0;
        AIN2_HBRIDGE_W = 1;
    }
    else if(pData->SpeedSetting == 0)   //Off
    {
        STBY_HBRIDGE_W = 0;
    }
    else    //Rotation anti-clockwise
    {
        STBY_HBRIDGE_W = 1;
        AIN1_HBRIDGE_W = 1;
        AIN2_HBRIDGE_W = 0;
    }
    
    /*//Get value for positive pulse for DC motor with formula
    appData.PulseWidthOC2 = (TIMER2_MOTOR_DC/MOTOR_DC_BAND*pData->absSpeed);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, appData.PulseWidthOC2);
    
    //Get value for positive pulse for servomotor formula
    appData.PulseWidthOC3 = (pData->absAngle * ((MAX_SERVO - MIN_SERVO)/SERVO_RANGE) + MIN_SERVO);
    PLIB_OC_PulseWidth16BitSet(OC_ID_3, appData.PulseWidthOC3);*/

    PLIB_OC_PulseWidth16BitSet(OC_ID_2, (TIMER2_MOTOR_DC/MOTOR_DC_BAND*pData->absSpeed));
    PLIB_OC_PulseWidth16BitSet(OC_ID_3, (pData->absAngle * ((MAX_SERVO - MIN_SERVO)/SERVO_RANGE) + MIN_SERVO));  
}

// Execution PWM software
void GPWM_ExecPWMSoft(S_pwmSettings *pData)
{
    static uint8_t PWM_Soft_4 = 0;
    
    PWM_Soft_4 = (PWM_Soft_4 + 1) % 100;
   
    if ( PWM_Soft_4 < pData -> absSpeed)
    {
        BSP_LEDOff(BSP_LED_2);
    }
    else
    {
        BSP_LEDOn(BSP_LED_2);
    }
}


uint16_t Avg_ADC_Value(uint16_t Avg_Table[])
{ 
    uint32_t Sum = 0;
    uint8_t i = 0;
    
    //Filling up table to get average
    for(i = 0; i < AVERAGE_SIZE; i++)
    {
        Sum += Avg_Table[9-i];
        if(i < 9)
        {
          Avg_Table[9-i] = Avg_Table[8-i];
        }
    }
    return(Sum/AVERAGE_SIZE);
}
