/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Application */
#include "app_supervisor.h"
#include "data_logger.h"
#include "inertial_states.h"
#include "motor_controls.h"
#include "behaviors.h"
#include "servo.h"
#include "wheel_speeds.h"
#include "bluetooth_control.h"
#include "battery_monitor.h"
#include "logging_streams.h"
#include "object_detection.h"
#include "constants.h"
#include "ip_app_iface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SUPERVISOR_PRIO            (configMAX_PRIORITIES - 1U)
#define SD_CARD_INIT_TASK_PRIO     (configMAX_PRIORITIES - 2U)
#define MC_DATA_LOGGING_TASK_PRIO  (configMAX_PRIORITIES - 3U)
#define BEHAVIORS_TASK_PRIO        (configMAX_PRIORITIES - 4U)
#define BLUETOOTH_CMD_TASK_PRIO    (configMAX_PRIORITIES - 5U)

typedef struct Task_Cfg_Tag
{
    TaskFunction_t func;
    const char name[configMAX_TASK_NAME_LEN];
    const configSTACK_DEPTH_TYPE stack_size;
    UBaseType_t priority;
    TaskHandle_t handle;
} Task_Cfg_T;

#if USE_ETHERNET
static void Supervisor_Task(void *pvParameters);
#endif
static void Create_App_Tasks(void);

/*******************************************************************************
* Variables
******************************************************************************/


/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
void Init_App(void)
{
   Init_Battery_Monitor();
   Init_Inertial_Sensors();
   Init_Wheel_Speed_Sensors();
   Init_Motor_Controls();
   Init_Object_Detection();
   Bluetooth_Serial_Open();
   Create_Streams();
   Init_Behaviors();

   NVIC_SetPriorityGrouping(0U);

#if USE_ETHERNET
   Init_Network_If();
   xTaskCreate(Supervisor_Task,   "Supervisor",  512,  NULL, SUPERVISOR_PRIO,  NULL);
#else
   Create_App_Tasks();
#endif
   vTaskStartScheduler();
}

static void Create_App_Tasks(void)
{
   xTaskCreate(SD_Card_Init_Task,    "SD_Card_Init_Task", 1024, NULL, SD_CARD_INIT_TASK_PRIO,     NULL);
   xTaskCreate(Behaviors_Task,       "Behaviors_Task",    2048, NULL, BEHAVIORS_TASK_PRIO,        NULL);
   xTaskCreate(Bluetooth_Cmd_Task,   "Bluetooth_Control", 1024, NULL, BLUETOOTH_CMD_TASK_PRIO,    NULL);
   xTaskCreate(Log_MC_Stream_Task,   "MC_Logging_Task",   1024, NULL, MC_DATA_LOGGING_TASK_PRIO,  NULL);
}

#if USE_ETHERNET
static void Supervisor_Task(void *pvParameters)
{
   bool app_tasks_created = false;

   while(1)
   {
      Print_DHCP_State();
      if (CONNECTED == Get_Network_Status())
      {
         if (!app_tasks_created)
         {
            Create_App_Tasks();
            app_tasks_created = true;
         }
      }

      vTaskDelay(pdMS_TO_TICKS(50));
   }
}
#endif
