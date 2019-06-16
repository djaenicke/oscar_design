/*
 * logging.cpp
 *
 *  Created on: Jun 16, 2019
 *      Author: Devin
 */

#include <stdio.h>
#include <string.h>
#include "fsl_sd.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_TASK_GET_SEM_BLOCK_TICKS 1U


/*******************************************************************************
* Variables
******************************************************************************/
static void SD_Card_Detect_Call_Back(bool is_inserted, void *userData);
static const sdmmchost_detect_card_t s_sdCardDetect = {
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
    .cdTimeOut_ms = (~0U),
    .cardInserted = SD_Card_Detect_Call_Back,
    .cardRemoved  = SD_Card_Detect_Call_Back,
};

static FATFS File_System;
static FIL   File_Object;

static uint32_t Task_Sleep_Ticks = portMAX_DELAY;

static volatile bool Card_Inserted     = false;
static volatile bool Card_Insert_Status = false;

static SemaphoreHandle_t File_Access_Semaphore = NULL;
static SemaphoreHandle_t Card_Detect_Semaphore = NULL;

static status_t Make_File_System(void);

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
static void SD_Card_Detect_Call_Back(bool is_inserted, void *userData)
{
   Card_Insert_Status = is_inserted;
   xSemaphoreGiveFromISR(Card_Detect_Semaphore, NULL);
}

void SD_Card_Detect_Task(void *pvParameters)
{
   File_Access_Semaphore = xSemaphoreCreateBinary();
   Card_Detect_Semaphore = xSemaphoreCreateBinary();

   g_sd.host.base           = SD_HOST_BASEADDR;
   g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
   g_sd.usrParam.cd         = &s_sdCardDetect;

   /* SD host init function */
   if (SD_HostInit(&g_sd) == kStatus_Success)
   {
     while (true)
     {
         /* take card detect semaphore */
         if (xSemaphoreTake(Card_Detect_Semaphore, portMAX_DELAY) == pdTRUE)
         {
             if (Card_Inserted != Card_Insert_Status)
             {
                 Card_Inserted = Card_Insert_Status;

                 /* power off card */
                 SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);

                 if (Card_Inserted)
                 {
                     PRINTF("\r\nCard inserted.\r\n");
                     /* power on the card */
                     SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);
                     /* make file system */
                     if (Make_File_System() != kStatus_Success)
                     {
                         continue;
                     }
                     xSemaphoreGive(File_Access_Semaphore);
                     Task_Sleep_Ticks = DEMO_TASK_GET_SEM_BLOCK_TICKS;
                 }
             }

             if (!Card_Inserted)
             {
                 PRINTF("\r\nPlease insert a card into board.\r\n");
             }
         }
     }
   }
   else
   {
     PRINTF("\r\nSD host init fail\r\n");
   }

   vTaskSuspend(NULL);
}

static status_t Make_File_System(void)
{
    FRESULT error;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};
    BYTE work[FF_MAX_SS];

    if (f_mount(&File_System, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return kStatus_Fail;
    }

#if (FF_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return kStatus_Fail;
    }
#endif

#if FF_USE_MKFS
    PRINTF("\r\nMake file system......The time may be long if the card capacity is big.\r\n");
    if (f_mkfs(driverNumberBuffer, FM_ANY, 0U, work, sizeof work))
    {
        PRINTF("Make file system failed.\r\n");
        return kStatus_Fail;
    }
#endif /* FF_USE_MKFS */

    PRINTF("\r\nCreate directory......\r\n");
    error = f_mkdir(_T("/dir_1"));
    if (error)
    {
        if (error == FR_EXIST)
        {
            PRINTF("Directory exists.\r\n");
        }
        else
        {
            PRINTF("Make directory failed.\r\n");
            return kStatus_Fail;
        }
    }

    return kStatus_Success;
}
