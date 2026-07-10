#include "watchdog.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

static uint32_t s_bootResetCause = 0;

void Watchdog_Init(void)
{
    s_bootResetCause = (uint32_t)DL_SYSCTL_getResetCause();

    DL_WWDT_reset(WWDT0);
    DL_WWDT_enablePower(WWDT0);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_WWDT_initWatchdogMode(WWDT0, DL_WWDT_CLOCK_DIVIDE_8,
        DL_WWDT_TIMER_PERIOD_12_BITS, DL_WWDT_RUN_IN_SLEEP,
        DL_WWDT_WINDOW_PERIOD_0, DL_WWDT_WINDOW_PERIOD_0);
    DL_WWDT_setActiveWindow(WWDT0, DL_WWDT_WINDOW0);
    DL_WWDT_setCoreHaltBehavior(WWDT0, DL_WWDT_CORE_HALT_STOP);
}

void Watchdog_Service(unsigned long nowMs, uint32_t schedulerHeartbeat)
{
    static unsigned long s_lastServiceMs = 0;
    static uint32_t s_lastHeartbeat = 0;

    if ((unsigned long)(nowMs - s_lastServiceMs) < APP_WATCHDOG_SERVICE_MS)
        return;

    if (schedulerHeartbeat != s_lastHeartbeat)
    {
        s_lastHeartbeat = schedulerHeartbeat;
        s_lastServiceMs = nowMs;
        DL_WWDT_restart(WWDT0);
    }
}

bool Watchdog_PreviousResetWasWatchdog(void)
{
    return s_bootResetCause ==
           (uint32_t)DL_SYSCTL_RESET_CAUSE_SYSRST_WWDT0_VIOLATION;
}
