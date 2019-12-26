typedef enum
{
    NOT_CONNECTED,
    CONNECTED
} Network_Status_T;

extern void Init_App(void);
extern Network_Status_T Get_Network_Status(void);
