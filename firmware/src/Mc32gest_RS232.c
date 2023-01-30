// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'émission et de réception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoyé réponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "gestPWM.h"
#include "Mc32gest_RS232.h"
#include "Mc32CalCrc16.h"


typedef union 
{
    uint16_t val;
    struct{
        uint8_t lsb;
        uint8_t msb;
    } shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)

// Structure décrivant le message
typedef struct {
    uint8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    uint8_t MsbCrc;
    uint8_t LsbCrc;
} StruMess;


// Struct pour émission des messages
StruMess TxMess;
// Struct pour réception des messages
StruMess RxMess;

// Declaration des FIFO pour réception et émission
#define FIFO_RX_SIZE ((4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ((4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de réception
S_fifo descrFifoRX;


int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'émission
S_fifo descrFifoTX;


// Initialisation de la communication sérielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de réception
    InitFifo (&descrFifoRX, FIFO_RX_SIZE, fifoRX, 0);
    // Initialisation du fifo d'émission
    InitFifo (&descrFifoTX, FIFO_TX_SIZE, fifoTX, 0);
    
    // Init RTS 
    RS232_RTS = 1;   // interdit émission par l'autre
   
} // InitComm

 
// Valeur de retour 0  = pas de message reçu donc local (data non modifié)
// Valeur de retour 1  = message reçu donc en remote (data mis à jour)
int GetMessage(S_pwmSettings *pData)
{
    static int commStatus = 0;
    int NbCharToRead;
    int LsbCrcControl, MsbCrcControl;
    uint16_t ctrl_CRC16 = 0xFFFF;
    uint8_t nbrCycles = 0;
    //U_manip16 RxCRC;

    // Lecture et décodage fifo réception
    NbCharToRead = GetReadSize(&descrFifoRX);

    if(NbCharToRead >= MESS_SIZE)
    {
        GetCharFromFifo(&descrFifoRX,&RxMess.Start);
        GetCharFromFifo(&descrFifoRX,&RxMess.Speed);
        GetCharFromFifo(&descrFifoRX,&RxMess.Angle);
        GetCharFromFifo(&descrFifoRX,&RxMess.MsbCrc);
        GetCharFromFifo(&descrFifoRX,&RxMess.LsbCrc);
    }
    else
    {
        commStatus = 0;
    }

    if(RxMess.Start == BYTE_BEGIN_MESSAGE)
    {
        
        ctrl_CRC16 = updateCRC16(ctrl_CRC16, RxMess.Start);
        ctrl_CRC16 = updateCRC16(ctrl_CRC16, RxMess.Speed);
        ctrl_CRC16 = updateCRC16(ctrl_CRC16, RxMess.Angle);

        MsbCrcControl = (ctrl_CRC16 >> 8)&0xFF;
        LsbCrcControl = ctrl_CRC16 &0xFF;

        if((RxMess.LsbCrc == LsbCrcControl)&&(RxMess.MsbCrc == MsbCrcControl))
        {
            pData->SpeedSetting = RxMess.Speed;
            pData->AngleSetting = RxMess.Angle;
            commStatus = 1;
            nbrCycles = 0;
        }
        else
        {
            LED6_W = !LED6_R;
            //commStatus = 0;
        }
    }
    else
    {
        //Attendre 10 cycles avant de switcher en mode local
        
        //Si 10 cycles passées 
        if (nbrCycles > 9) 
        {
            //Remet a 0 le nbr de cycle
            nbrCycles = 0; 
            
            //Indicateur de reception à 0
            commStatus = 0;
         }
        
        //Incrémentation du nbr de cycles  
        nbrCycles++;  
    }
    
    // Gestion controle de flux de la réception
    if(GetWriteSpace ( &descrFifoRX) >= (2*MESS_SIZE)) 
    {
        // autorise émission par l'autre
        RS232_RTS = 0;
    }
    return commStatus;
} // GetMessage


// Fonction d'envoi des messages, appel cyclique
void SendMessage(S_pwmSettings *pData)
{
    int8_t freeSize;
    uint16_t value_CRC16 = 0xFFFF;

    // Traitement émission à introduire ICI
    freeSize = GetWriteSpace(&descrFifoTX);
    // Formatage message et remplissage fifo émission
    if(freeSize >= MESS_SIZE)
    {
        value_CRC16 = updateCRC16(value_CRC16, BYTE_BEGIN_MESSAGE);
        value_CRC16 = updateCRC16(value_CRC16, pData->SpeedSetting);
        value_CRC16 = updateCRC16(value_CRC16, pData->AngleSetting);

        TxMess.Start = BYTE_BEGIN_MESSAGE;
        TxMess.Speed = pData->SpeedSetting;
        TxMess.Angle = pData->AngleSetting;
        TxMess.MsbCrc = (value_CRC16 >> 8)&0xFF;
        TxMess.LsbCrc = value_CRC16 &0xFF;

        PutCharInFifo (&descrFifoTX, TxMess.Start);
        PutCharInFifo (&descrFifoTX, TxMess.Speed);
        PutCharInFifo (&descrFifoTX, TxMess.Angle);
        PutCharInFifo (&descrFifoTX, TxMess.MsbCrc);
        PutCharInFifo (&descrFifoTX, TxMess.LsbCrc);
    }

    // Gestion du controle de flux
    // si on a un caractère à envoyer et que CTS = 0
    freeSize = GetReadSize(&descrFifoTX);
    if ((RS232_CTS == 0) && (freeSize > 0))
    {
        // Autorise int émission    
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
    }
}


// Interruption USART1
void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    USART_ERROR UsartStatus;   
    int ctsStatus;
    int32_t txSize;
    int txBufferState;
    int8_t rxSize;
    int8_t Car;
    int8_t ptcarLu;


    // Marque début interruption avec Led3
    LED3_W = 1;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) )
    {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur à la réception
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) )
    {

        // Oui Test si erreur parité ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if((UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0)
        {

            // Traitement RX à faire ICI
            // Lecture des caractères depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            // ...
            while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {
                Car = PLIB_USART_ReceiverByteReceive(USART_ID_1);
                PutCharInFifo (&descrFifoRX, Car);
                
            }
            LED4_W = !LED4_R; // Toggle Led4
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        }
        else
        {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN)
            {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }

        
        // Traitement controle de flux reception à faire ICI
        // Gerer sortie RS232_RTS en fonction de place dispo dans fifo reception
        rxSize = GetWriteSpace(&descrFifoRX);
        if (rxSize <= 6) // a cause d'un int pour 6 char
        {
            // Demande de ne plus émettre
            RS232_RTS = 1;
        }

        
    } // end if RX

    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) )
    {

        // Traitement TX à faire ICI
        // Envoi des caractères depuis le fifo SW -> buffer HW
            
        // Avant d'émettre, on vérifie 3 conditions :
        //  Si CTS = 0 autorisation d'émettre (entrée RS232_CTS)
        //  S'il y a un caratères à émettre dans le fifo
        //  S'il y a de la place dans le buffer d'émission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
        
        ctsStatus = RS232_CTS;
        txSize = GetReadSize (&descrFifoTX);
        txBufferState = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
       
        if ((ctsStatus == 0) && ( txSize > 0 ) && txBufferState==false )
        {
            do{
                // Traitement TX à faire ICI
                // Envoi des caractères depuis le fifo SW -> buffer HW
                GetCharFromFifo(&descrFifoTX, &ptcarLu);
                PLIB_USART_TransmitterByteSend(USART_ID_1, ptcarLu);
                LED5_W = !LED5_R; // Toggle Led5
                ctsStatus = RS232_CTS;
                txSize = GetReadSize (&descrFifoTX);
                txBufferState = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
            } while (( ctsStatus == 0) && ( txSize > 0 ) && txBufferState==false);
        }

        // ...
       
	   
        LED5_W = !LED5_R; // Toggle Led5
		
        // disable TX interrupt (pour éviter une interrupt. inutile si plus rien à transmettre)
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
        // Clear the TX interrupt Flag (Seulement apres TX) 
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    // Marque fin interruption avec Led3
    LED3_W = 0;
}




