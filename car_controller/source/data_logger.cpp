/*
 * data_logger.cpp
 *
 *  Created on: Jun 16, 2019
 *      Author: Devin
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "fsl_sd.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "board.h"
#include "battery_monitor.h"
#include "wheel_speeds.h"
#include "logging_streams.h"
#include "pin_mux.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BUFFER_SIZE (100U)
#define CLOSED (0U)
#define OPEN   (1U)

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
static uint8_t File_State = CLOSED;
static bool File_System_Init_Complete = false;

static volatile bool Card_Inserted      = false;
static volatile bool Card_Insert_Status = false;

static SemaphoreHandle_t Card_Detect_Semaphore = NULL;
static SemaphoreHandle_t File_Access_Semaphore = NULL;

static Motor_Controls_Stream_T MC_Stream_Data;

static int Init_File_System(void);

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
void Log_MC_Stream_Task(void *pvParameters)
{
   size_t br = 0;
   uint32_t bw = 0;

   while(1)
   {
      br = xStreamBufferReceive(MC_Stream.handle,
                                (void *) &MC_Stream_Data,
                                sizeof(MC_Stream_Data),
                                portMAX_DELAY);

      assert(br==sizeof(MC_Stream_Data));

      if (pdTRUE == xSemaphoreTake(File_Access_Semaphore, pdMS_TO_TICKS(5)))
      {
         if (SD_IsCardPresent(&g_sd) && OPEN == File_State)
         {
            f_write(&File_Object, &MC_Stream_Data, sizeof(MC_Stream_Data), (UINT *)&bw);
            assert(bw==sizeof(MC_Stream_Data));
         }

         xSemaphoreGive(File_Access_Semaphore);
      }

      vTaskDelay(pdMS_TO_TICKS(5));
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
         if (xSemaphoreTake(Card_Detect_Semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
         {
            if (Card_Inserted != Card_Insert_Status)
            {
               Card_Inserted = Card_Insert_Status;

               SD_PowerOffCard(g_sd.host.base, g_sd.usrParam.pwr);

               if (Card_Inserted)
               {
                  SD_PowerOnCard(g_sd.host.base, g_sd.usrParam.pwr);

                  if (-1 != Init_File_System())
                  {
                     xSemaphoreGive(File_Access_Semaphore);
                  }
               }
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

   File_System_Init_Complete = true;
   return 0;
}

void Open_Log_File(void)
{
   FRESULT error;

   if (File_System_Init_Complete)
   {
      PRINTF("\r\nCreate a file......\r\n");
      error = f_open(&File_Object, _T("/dir_1/data_log.bin"), (FA_WRITE | FA_CREATE_ALWAYS));
      if (error)
      {
           PRINTF("Open file failed.\r\n");
      }
      else
      {
         PRINTF("File created.\r\n");
         File_State = OPEN;
      }
   }
}

void Close_Log_File(void)
{
   if (CLOSED != File_State)
   {
      if (pdTRUE == xSemaphoreTake(File_Access_Semaphore, portMAX_DELAY))
      {
         if (f_close(&File_Object))
         {
            PRINTF("\r\nFailed to close file.\r\n");
         }
         else
         {
            PRINTF("\r\nFile closed.\r\n");
            File_State = CLOSED;
         }
      }
   }
}

static void SD_Card_Detect_Call_Back(bool is_inserted, void *userData)
{
   Card_Insert_Status = is_inserted;
   xSemaphoreGiveFromISR(Card_Detect_Semaphore, NULL);
}
