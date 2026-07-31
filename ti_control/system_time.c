#include "system_time.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t gSystemMillis = 0U;

void SystemTime_Init(void)
{
    gSystemMillis = 0U;
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
}

uint32_t SystemTime_Millis(void)
{
    return gSystemMillis;
}

void SysTick_Handler(void)
{
    gSystemMillis++;
}
