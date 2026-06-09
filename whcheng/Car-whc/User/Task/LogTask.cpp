#include "LogTask.hpp"

#include "cmsis_os.h"
#include "robot_messages.hpp"
#include "robot_runtime.hpp"
#include "oled.hpp"

using namespace RobotMessages;
using namespace RobotRuntime;
using BSP::OLED::Color;

namespace
{
    /* 图表参数 */
    constexpr uint8_t kGraphX0   = 24;   // 图表左边界 (Y轴)
    constexpr uint8_t kGraphY0   = 16;   // 图表上边界
    constexpr uint8_t kGraphX1   = 127;  // 图表右边界
    constexpr uint8_t kGraphY1   = 63;   // 图表下边界 (=64-1)
    constexpr uint8_t kGraphW    = kGraphX1 - kGraphX0 + 1;  // 图表宽度 = 104
    constexpr uint8_t kGraphH    = kGraphY1 - kGraphY0 + 1;  // 图表高度 = 48

    constexpr float kMaxHeight   = 40.0f;  // Y轴最大高度 mm

    /* 高度历史环形缓冲 */
    float height_history[128] = {};
    uint8_t hist_index = 0;
    bool hist_full = false;

    /* 消息订阅数据 */
    HighFeedbackData feedback_data{};
    HighTargetData   target_data{};
    MotorOutData     motor_output{};

    void OnFeedback(const HighFeedbackData &msg) { feedback_data = msg; }
    void OnTarget(const HighTargetData &msg)     { target_data   = msg; }
    void OnMotorOut(const MotorOutData &msg)     { motor_output  = msg; }

    void InitMessageSubscribe()
    {
        SubscribeHighFeedbackData(OnFeedback);
        SubscribeHighTargetData(OnTarget);
        SubscribeMotorOutData(OnMotorOut);
    }

    /* 高度值 -> 像素Y坐标 (值越大越靠上) */
    uint8_t HeightToY(float h)
    {
        if (h < 0.0f)        h = 0.0f;
        if (h > kMaxHeight)  h = kMaxHeight;
        return kGraphY1 - static_cast<uint8_t>((h / kMaxHeight) * static_cast<float>(kGraphH - 2));
    }

    /* Bresenham 画线 (带边界裁剪) */
    void DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1)
    {
        auto &oled = Oled();
        int16_t dx  = x1 > x0 ? x1 - x0 : x0 - x1;
        int16_t dy  = y1 > y0 ? y1 - y0 : y0 - y1;
        int16_t sx = x0 < x1 ? 1 : -1;
        int16_t sy = y0 < y1 ? 1 : -1;
        int16_t err = dx - dy;

        while (true)
        {
            if (x0 >= kGraphX0 && x0 <= kGraphX1 && y0 >= kGraphY0 && y0 <= kGraphY1)
                oled.DrawPixel(static_cast<uint8_t>(x0), static_cast<uint8_t>(y0), Color::White);
            if (x0 == x1 && y0 == y1) break;
            int16_t e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }

    /* 绘制图表区域 */
    void DrawGraph()
    {
        auto &oled = Oled();

        // --- Y 轴 (左边界) ---
        for (uint8_t y = kGraphY0; y <= kGraphY1; y++)
            oled.DrawPixel(kGraphX0, y, Color::White);

        // --- X 轴 (底边) ---
        for (uint8_t x = kGraphX0; x <= kGraphX1; x++)
            oled.DrawPixel(x, kGraphY1, Color::White);

        // --- Y 轴刻度 + 标签 ---
        for (int h = 0; h <= (int)kMaxHeight; h += 10)
        {
            uint8_t y = HeightToY(static_cast<float>(h));
            oled.DrawPixel(kGraphX0,     y, Color::White);
            oled.DrawPixel(kGraphX0 + 1, y, Color::White);
            oled.DrawPixel(kGraphX0 + 2, y, Color::White);

            if (h % 20 == 0 && y >= 4)
            {
                oled.SetCursor(0, y - 4);
                oled.WriteInt(h);
            }
        }

        // --- 目标高度参考线 (虚线) ---
        float target_cm = static_cast<float>(target_data.target_distance_mm) * 0.1f;
        uint8_t ty = HeightToY(target_cm);
        for (uint8_t x = kGraphX0 + 1; x < kGraphX1; x += 3)
            oled.DrawPixel(x, ty, Color::White);

        // --- 当前高度曲线 ---
        uint8_t total = hist_full ? 128 : hist_index;
        uint8_t count = total < kGraphW ? total : kGraphW;
        if (count < 2) return;

        for (uint8_t i = 1; i < count; i++)
        {
            int idx0 = hist_index - count + i - 1;
            int idx1 = hist_index - count + i;
            if (idx0 < 0) idx0 += 128;
            if (idx1 < 0) idx1 += 128;

            int16_t x0 = kGraphX0 + i - 1;
            int16_t x1 = kGraphX0 + i;
            int16_t y0 = HeightToY(height_history[idx0]);
            int16_t y1 = HeightToY(height_history[idx1]);
            DrawLine(x0, y0, x1, y1);
        }
    }

    /* 绘制顶部文字：两行不重叠 */
    void DrawText()
    {
        auto &oled = Oled();

        // 第一行：当前高度    目标高度
        oled.SetCursor(2, 0);
        oled.WriteString("H:", Color::White);
        oled.WriteInt(static_cast<int32_t>(feedback_data.feedback_distance_mm / 10));

        oled.SetCursor(50, 0);
        oled.WriteString("T:", Color::White);
        oled.WriteInt(static_cast<int32_t>(target_data.target_distance_mm / 10));

        // 第二行：PWM输出
        oled.SetCursor(90, 0);
        oled.WriteString("P:", Color::White);
        oled.WriteInt(static_cast<int32_t>(motor_output.high_out));
    }
}

extern "C" void LogTask(void const *argument)
{
    InitMessageSubscribe();
    Oled().Init();

    for (;;)
    {
        height_history[hist_index] = static_cast<float>(feedback_data.feedback_distance_mm) * 0.1f;
        hist_index = (hist_index + 1) % 128;
        if (hist_index == 0) hist_full = true;

        Oled().Fill(Color::Black);
        DrawText();
        DrawGraph();
        Oled().Update();

        osDelay(33);
    }
}
