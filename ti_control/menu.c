#include "menu.h"

#include <stdio.h>

#include "line_follow.h"
#include "oled.h"
#include "servo.h"
#include "static_ball.h"
#include "system_time.h"

#define MENU_VISIBLE_ROWS 6U
#define MENU_STATUS_REFRESH_MS 200U
#define MENU_SERVO_STATUS_REFRESH_MS 250U

typedef enum {
    MENU_VIEW_HOME = 0,
    MENU_VIEW_PAGE
} MenuView;

typedef enum {
    MENU_ITEM_LINE_FOLLOW = 0,
    MENU_ITEM_STATIC_BALL,
    MENU_ITEM_AB_BALANCE,
    MENU_ITEM_FULL_LOOP,
    MENU_ITEM_TARGET_POS
} MenuItemId;

typedef struct {
    const char *title;
    const char *line1;
    const char *line2;
} MenuItem;

static const MenuItem gMenuItems[] = {
    {"Line Follow", "Req.2 line track", "TODO: wait logic"},
    {"Static Ball", "Req.3 static ball", "TODO: wait logic"},
    {"AB Balance", "Req.4 A to B", "TODO: wait logic"},
    {"Full Loop", "Req.5 one circle", "TODO: wait logic"},
    {"Target Pos", "Req.6 any point", "TODO: wait logic"},
};

#define MENU_ITEM_COUNT ((uint8_t)(sizeof(gMenuItems) / sizeof(gMenuItems[0])))

static MenuView gView = MENU_VIEW_HOME;
static uint8_t gSelectedIndex = 0;
static uint8_t gTopIndex = 0;
static bool gNeedsRefresh = true;
static uint32_t gLastStatusRefreshMs = 0U;

static void Menu_ResetUiState(void)
{
    gView = MENU_VIEW_HOME;
    gSelectedIndex = 0;
    gTopIndex = 0;
    gNeedsRefresh = true;
}

static void Menu_EnsureSelectionVisible(void)
{
    if (gSelectedIndex < gTopIndex) {
        gTopIndex = gSelectedIndex;
    } else if (gSelectedIndex >= (uint8_t)(gTopIndex + MENU_VISIBLE_ROWS)) {
        gTopIndex = (uint8_t)(gSelectedIndex - MENU_VISIBLE_ROWS + 1U);
    }
}

static void Menu_StartSelectedFunction(void)
{
    if (gSelectedIndex == MENU_ITEM_LINE_FOLLOW) {
        Servo_Detach();
        LineFollow_Start();
    } else if (gSelectedIndex == MENU_ITEM_STATIC_BALL) {
        StaticBall_Start();
    } else {
        Servo_Detach();
    }
}

static void Menu_StopSelectedFunction(void)
{
    if (gSelectedIndex == MENU_ITEM_LINE_FOLLOW) {
        LineFollow_Stop();
    } else if (gSelectedIndex == MENU_ITEM_STATIC_BALL) {
        StaticBall_Exit();
    }
}

void Menu_Init(void)
{
    Menu_ResetUiState();
}

void Menu_HandleEvent(ButtonEvent event)
{
    switch (event) {
        case BUTTON_EVENT_S1_PRESSED:
            if (gView == MENU_VIEW_HOME) {
                gSelectedIndex = (uint8_t)((gSelectedIndex + 1U) % MENU_ITEM_COUNT);
                Menu_EnsureSelectionVisible();
                gNeedsRefresh = true;
            }
            break;
        case BUTTON_EVENT_S2_PRESSED:
            if (gView == MENU_VIEW_HOME) {
                Menu_StartSelectedFunction();
                gView = MENU_VIEW_PAGE;
                gLastStatusRefreshMs = 0U;
                gNeedsRefresh = true;
            }
            break;
        case BUTTON_EVENT_S3_PRESSED:
            Menu_StopSelectedFunction();
            Menu_ResetUiState();
            break;
        case BUTTON_EVENT_S4_PRESSED:
            if (gView == MENU_VIEW_PAGE) {
                Menu_StopSelectedFunction();
                gView = MENU_VIEW_HOME;
                gNeedsRefresh = true;
            }
            break;
        case BUTTON_EVENT_NONE:
        default:
            break;
    }
}

static void Menu_RenderHome(void)
{
    uint8_t row;

    Oled_Clear();
    Oled_DrawString(0, 0, "FOLLOW MENU");
    Oled_DrawHLine(0, 9, 128);

    for (row = 0; row < MENU_VISIBLE_ROWS; row++) {
        uint8_t itemIndex = (uint8_t)(gTopIndex + row);
        uint8_t y = (uint8_t)(12U + row * 8U);

        if (itemIndex >= MENU_ITEM_COUNT) {
            break;
        }

        Oled_DrawChar(0, y, (itemIndex == gSelectedIndex) ? '>' : ' ');
        Oled_DrawString(8, y, gMenuItems[itemIndex].title);
    }

    Oled_Update();
}

static void Menu_RenderLineFollowPage(void)
{
    LineFollowStatus status;
    char line[22];

    LineFollow_GetStatus(&status);

    Oled_Clear();
    Oled_DrawString(0, 0, "Line Follow");
    Oled_DrawHLine(0, 9, 128);
    Oled_DrawString(0, 12, status.active ? "RUN" : "STOP");

    if (status.lineLost) {
        Oled_DrawString(36, 12, "LOST");
    } else if (status.allBlack) {
        Oled_DrawString(36, 12, "ALL BLACK");
    } else {
        Oled_DrawString(36, 12, "TRACK");
    }

    (void)snprintf(line, sizeof(line), "M:%02X E:%d",
        status.sensorMask, status.lineError);
    Oled_DrawString(0, 24, line);

    (void)snprintf(line, sizeof(line), "L:%d/%d",
        status.leftTarget, status.leftMeasured);
    Oled_DrawString(0, 34, line);

    (void)snprintf(line, sizeof(line), "R:%d/%d",
        status.rightTarget, status.rightMeasured);
    Oled_DrawString(0, 44, line);

    Oled_DrawString(0, 56, "S3:STOP S4:BACK");
    Oled_Update();
}

static void Menu_RenderPage(void)
{
    const MenuItem *item = &gMenuItems[gSelectedIndex];
    StaticBallStatus staticBallStatus;

    if (gSelectedIndex == MENU_ITEM_STATIC_BALL) {
        StaticBall_GetStatus(&staticBallStatus);

        Oled_Clear();
        Oled_DrawString(0, 0, "Static Ball");
        Oled_DrawHLine(0, 9, 128);
        Oled_DrawString(0, 12, staticBallStatus.active ? "RUN" : "STOP");

        {
            char line[22];
            int16_t x = staticBallStatus.ballXCmX100;
            int16_t v = staticBallStatus.velocityCmSX100;
            int16_t target = staticBallStatus.targetCmX100;

            (void)snprintf(line, sizeof(line), "R:%lu O:%lu",
                (unsigned long)staticBallStatus.rxBytes,
                (unsigned long)staticBallStatus.rxOk);
            Oled_DrawString(36, 12, line);

            if (x < 0) {
                (void)snprintf(line, sizeof(line), "X:-%d.%02d Q:%u",
                    (int)(-x / 100), (int)(-x % 100),
                    staticBallStatus.quality);
            } else {
                (void)snprintf(line, sizeof(line), "X:%d.%02d Q:%u",
                    (int)(x / 100), (int)(x % 100),
                    staticBallStatus.quality);
            }
            if (staticBallStatus.visionValid) {
                Oled_DrawString(0, 22, line);
            } else if (staticBallStatus.visionFresh) {
                (void)snprintf(line, sizeof(line), "X:QLOW Q:%u",
                    staticBallStatus.quality);
                Oled_DrawString(0, 22, line);
            } else if (staticBallStatus.rxOk == 0U) {
                Oled_DrawString(0, 22, "X:WAIT UART");
            } else {
                Oled_DrawString(0, 22, "X:LOST");
            }

            if (v < 0) {
                (void)snprintf(line, sizeof(line), "V:-%d.%02dcm/s",
                    (int)(-v / 100), (int)(-v % 100));
            } else {
                (void)snprintf(line, sizeof(line), "V:%d.%02dcm/s",
                    (int)(v / 100), (int)(v % 100));
            }
            if (staticBallStatus.rxOk == 0U) {
                (void)snprintf(line, sizeof(line), "A:%lu H:%lu L:%02X",
                    (unsigned long)staticBallStatus.rxHeadAA,
                    (unsigned long)staticBallStatus.rxHeadAA55,
                    staticBallStatus.lastByte);
            }
            Oled_DrawString(0, 32, line);

            if (target < 0) {
                (void)snprintf(line, sizeof(line), "P:%u G:-%d.%02d S:%d",
                    staticBallStatus.phase,
                    (int)(-target / 100), (int)(-target % 100),
                    staticBallStatus.servoAngleDeg);
            } else {
                (void)snprintf(line, sizeof(line), "P:%u G:%d.%02d S:%d",
                    staticBallStatus.phase,
                    (int)(target / 100), (int)(target % 100),
                    staticBallStatus.servoAngleDeg);
            }
            if (staticBallStatus.rxOk == 0U) {
                (void)snprintf(line, sizeof(line), "N:%02X T:%02X C:%02X/%02X",
                    staticBallStatus.lastLength,
                    staticBallStatus.lastType,
                    staticBallStatus.lastChecksumRx,
                    staticBallStatus.lastChecksumCalc);
            }
            Oled_DrawString(0, 42, line);

            Oled_DrawString(0, 56, "S3:STOP S4:BACK");
        }

        Oled_Update();
        return;
    }

    if (gSelectedIndex == MENU_ITEM_LINE_FOLLOW) {
        Menu_RenderLineFollowPage();
        return;
    }

    Oled_Clear();
    Oled_DrawString(0, 0, item->title);
    Oled_DrawHLine(0, 9, 128);
    Oled_DrawString(0, 18, item->line1);
    Oled_DrawString(0, 30, item->line2);
    Oled_DrawString(0, 46, "S3:Reset  S4:Back");
    Oled_Update();
}

void Menu_Render(void)
{
    if ((gView == MENU_VIEW_PAGE) &&
        (gSelectedIndex == MENU_ITEM_LINE_FOLLOW) &&
        LineFollow_IsActive()) {
        uint32_t nowMs = SystemTime_Millis();

        if ((nowMs - gLastStatusRefreshMs) >= MENU_STATUS_REFRESH_MS) {
            gLastStatusRefreshMs = nowMs;
            gNeedsRefresh = true;
        }
    }

    if ((gView == MENU_VIEW_PAGE) &&
        (gSelectedIndex == MENU_ITEM_STATIC_BALL) &&
        StaticBall_IsActive()) {
        uint32_t nowMs = SystemTime_Millis();

        if ((nowMs - gLastStatusRefreshMs) >= MENU_SERVO_STATUS_REFRESH_MS) {
            gLastStatusRefreshMs = nowMs;
            gNeedsRefresh = true;
        }
    }

    if (!gNeedsRefresh || !Oled_IsReady()) {
        return;
    }

    if (gView == MENU_VIEW_HOME) {
        Menu_RenderHome();
    } else {
        Menu_RenderPage();
    }

    gNeedsRefresh = false;
}
