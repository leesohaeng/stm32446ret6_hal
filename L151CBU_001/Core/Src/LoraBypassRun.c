#include "LoraBypassRun.h"
#include "Lom202aHandle.h"
#include "DebugSerial.h"
#include "Lom202aHandle.h"

/* Private define ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
#define MAX_DEBUG_LINE_LEN  (128)
#define MAX_LORA_LINE_LEN   (128)

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t DEBUG_LINE[MAX_DEBUG_LINE_LEN];
uint8_t LORA_LINE[MAX_LORA_LINE_LEN];
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void LoraBypassMode(void)
{
	uint8_t getCh;
    int32_t debugLineIndex = 0, loraLineIndex = 0;
	
  	while(1)
	{
        while(DebugSerial_RxExist()) {
            getCh = DebugSerial_GetByte();
            if(getCh == '\n') {
                DEBUG_LINE[debugLineIndex++] = getCh;
                LoraCommSerial_Send(DEBUG_LINE, debugLineIndex);
				debugLineIndex = 0;
            }
            else {
                DEBUG_LINE[debugLineIndex++] = getCh;
            }
        }

        while(LoraCommSerial_RxExist()) {
            getCh = LoraCommSerial_GetByte();
            if(getCh == '\n') {
                LORA_LINE[loraLineIndex++] = getCh;
                DebugSerial_Send(LORA_LINE, loraLineIndex);
                loraLineIndex = 0;
            }
            else {
                LORA_LINE[loraLineIndex++] = getCh;
            }
        }
    }
}

