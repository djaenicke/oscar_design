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

extern void Init_App(void);
