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
#define BUFFER_SIZE (100U)

/*******************************************************************************
* Variables
******************************************************************************/
SDK_ALIGN(uint8_t Write_Buffer[SDK_SIZEALIGN(BUFFER_SIZE, SDMMC_DATA_BUFFER_ALIGN_CACHE)],
          MAX(SDMMC_DATA_BUFFER_ALIGN_CACHE, SDMMCHOST_DMA_BUFFER_ADDR_ALIGN));

static void SD_Card_Detect_Call_Back(bool is_inserted, void *userData);
static const sdmmchost_detect_card_t s_sdCardDetect = {
    .cdType = kSDMMCHOST_DetectCardByGpioCD,
    .cdTimeOut_ms = (~0U),
    .cardInserted = SD_Card_Detect_Call_Back,
    .cardRemoved  = SD_Card_Detect_Call_Back,
};

static FATFS File_System;
static FIL   File_Object;

static volatile bool Card_Inserted      = false;
static volatile bool Card_Insert_Status = false;

static SemaphoreHandle_t Card_Detect_Semaphore = NULL;
static SemaphoreHandle_t File_Access_Semaphore = NULL;

static int Init_File_System(void);

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
void Data_Logging_Task(void *pvParameters)
{
   while(1)
   {
      if (pdTRUE == xSemaphoreTake(File_Access_Semaphore, portMAX_DELAY))
      {
         if (SD_IsCardPresent(&g_sd))
         {
            /* TODO: Add mechanism to open and close file from bluetooth input */
         }

         xSemaphoreGive(File_Access_Semaphore);
      }
      else
      {
         PRINTF("Failed to take semaphore.\r\n");
      }
   }
}

void SD_Card_Init_Task(void *pvParameters)
{
   Card_Detect_Semaphore = xSemaphoreCreateBinary();
   File_Access_Semaphore = xSemaphoreCreateBinary();

   g_sd.host.base           = SD_HOST_BASEADDR;
   g_sd.host.sourceClock_Hz = SD_HOST_CLK_FREQ;
   g_sd.usrParam.cd         = &s_sdCardDetect;

   if (SD_HostInit(&g_sd) == kStatus_Success)
   {
      while (true)
      {
         if (xSemaphoreTake(Card_Detect_Semaphore, portMAX_DELAY) == pdTRUE)
         {
            if (Card_Inserted != Card_Insert_Status)
            {
               Card_Inserted = Card_Insert_Status;

               SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);

               if (Card_Inserted)
               {
                  PRINTF("\r\nCard inserted.\r\n");

                  SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);

                  if (-1 != Init_File_System())
                  {
                     xSemaphoreGive(File_Access_Semaphore);
                  }
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

static int Init_File_System(void)
{
   FRESULT error;
   const TCHAR drive_number[3U] = {SDDISK + '0', ':', '/'};

   error = f_mount(&File_System, drive_number, 0U);
   if (error)
   {
       PRINTF("Mount volume failed.\r\n");
       return -1;
   }

   error = f_chdrive((char const *)&drive_number[0U]);
   if (error)
   {
       PRINTF("Change drive failed.\r\n");
       return -1;
   }

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
         return -1;
       }
   }

   /* TODO
   PRINTF("\r\nCreate a file......\r\n");
   error = f_open(&File_Object, _T("/dir_1/data_log.bin"), (FA_WRITE | FA_CREATE_NEW));
   if (error)
   {
        PRINTF("Open file failed.\r\n");
        return -1;
   }
   else
   {
      PRINTF("File created.\r\n");
   }
   */

   return 0;
}

static void SD_Card_Detect_Call_Back(bool is_inserted, void *userData)
{
   Card_Insert_Status = is_inserted;
   xSemaphoreGiveFromISR(Card_Detect_Semaphore, NULL);
}
