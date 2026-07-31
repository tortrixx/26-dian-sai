#include "app.h"

#include "buttons.h"
#include "k230_uart.h"
#include "line_follow.h"
#include "menu.h"
#include "oled.h"
#include "servo.h"
#include "static_ball.h"
#include "system_time.h"
#include "ti_msp_dl_config.h"

#define APP_LOOP_DELAY_CYCLES (32000U * 1U)

static void App_DelayForUiTick(void)
{
    delay_cycles(APP_LOOP_DELAY_CYCLES);
}

void App_Init(void)
{
    SYSCFG_DL_init();
    SystemTime_Init();
    Buttons_Init();
    Oled_StartInit();
    K230Uart_Init();
    Servo_Init();
    StaticBall_Init();
    LineFollow_Init();
    Menu_Init();
}

void App_Run(void)
{
    ButtonEvent event;

    while (1) {
        K230Uart_Task();
        StaticBall_Task();
        Servo_Task();
        LineFollow_Task();
        Oled_Task();

        event = Buttons_Poll();
        if (event != BUTTON_EVENT_NONE) {
            Menu_HandleEvent(event);
        }

        Menu_Render();
        App_DelayForUiTick();
    }
}
