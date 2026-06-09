#ifndef LASER_HPP
#define LASER_HPP

#include <stdint.h>

namespace BSP::Laser
{
    /**
     * @brief MINIF(DVP0501C1)激光测距数据。
     */
    struct LaserData
    {
        /** 距离，单位毫米，有效范围20~4500。 */
        uint16_t distance_mm = 0;
        /** 置信度，范围0~62，值越大表示干扰越大。 */
        uint8_t confidence = 0;
        /** 数据是否有效（最近一次帧解析成功）。 */
        bool valid = false;
    };

    /**
     * @brief MINIF(DVP0501C1)激光测距模组封装。
     *
     * 通过UART接收ASCII格式的距离数据，默认30Hz持续输出。
     *
     * 实际数据帧格式：<距离ASCII>, <置信度ASCII><\n>
     * 示例：33 32 37 2C 20 36 31 0A -> "327, 61\n" -> 距离327mm，置信度61
     *
     * 使用方式：
     * - 构造后调用 UartManager 注册 OnUartRx 回调
     * - 解析数据后通过 LaserData 结构获取距离和置信度
     */
    class Laser
    {
    public:
        /**
         * @brief 构造激光测距对象。
         */
        Laser() = default;

        /**
         * @brief 获取最新的激光测距数据。
         *
         * @return const LaserData& 激光数据只读引用。
         */
        const LaserData &GetData() const
        {
            return data_;
        }

        /**
         * @brief 获取最近一次有效帧的距离值。
         *
         * @return uint16_t 距离，单位毫米。数据无效时返回0。
         */
        uint16_t GetDistance() const
        {
            return data_.distance_mm;
        }

        /**
         * @brief 获取最近一次有效帧的置信度。
         *
         * @return uint8_t 置信度，范围0~62，值越大表示干扰越大。数据无效时返回0。
         */
        uint8_t GetConfidence() const
        {
            return data_.confidence;
        }

        /**
         * @brief 判断最近一次解析的数据帧是否有效。
         *
         * @return true 帧解析成功。
         * @return false 帧解析失败或尚未接收到数据。
         */
        bool IsValid() const
        {
            return data_.valid;
        }

        /**
         * @brief UART接收回调，用于接收和处理激光模组的ASCII数据帧。
         *
         * 该函数需注册到DRV::UART::UartManager的对应UART通道：
         * DRV::UART::UartManager::Instance().RegisterRxCallback(uart_id, callback);
         *
         * @param data 原始UART接收数据缓冲区。
         * @param size 数据长度。
         */
        void OnUartRx(const uint8_t *data, uint16_t size)
        {
            ParseFrame(data, size);
        }

    private:
        /**
         * @brief 解析激光模组ASCII数据帧。
         *
         * 实际帧格式：<距离ASCII>,<空格><置信度ASCII><\n>
         *
         * 激光30Hz持续发送，DMA启动时机随机。
         * 以 \n 为帧边界，向前定位完整帧后再解析，
         * 天然处理了半帧/粘帧问题。
         *
         * @param data 原始数据缓冲区。
         * @param size 数据长度。
         */
        void ParseFrame(const uint8_t *data, uint16_t size)
        {
            if (data == nullptr || size < 6)
            {
                data_.valid = false;
                return;
            }

            const uint8_t *end = data + size;

            // 1. 从末尾向前查找帧尾 \n (0x0A)
            const uint8_t *tail = end - 1;
            while (tail >= data && *tail != 0x0A)
            {
                --tail;
            }
            if (tail < data)
            {
                data_.valid = false;
                return;
            }

            // 2. 从 \n 再向前跳过前一帧（或半帧），定位本帧起始字节
            const uint8_t *head = tail - 1;
            while (head >= data && *head != 0x0A)
            {
                --head;
            }
            ++head; // head 指向本帧第一个字节

            const uint8_t *p = head;

            // 3. 解析距离值（ASCII数字，1~5位）
            uint16_t distance = 0;
            uint8_t dist_digits = 0;
            while (p < tail && *p >= '0' && *p <= '9')
            {
                distance = distance * 10 + static_cast<uint16_t>(*p - '0');
                ++dist_digits;
                ++p;
            }

            if (dist_digits == 0 || dist_digits > 5)
            {
                data_.valid = false;
                return;
            }

            // 4. 验证分隔符 ", " (0x2C 0x20)
            if (p + 1 >= tail || *p != 0x2C || *(p + 1) != 0x20)
            {
                data_.valid = false;
                return;
            }
            p += 2;

            // 5. 解析置信度（ASCII数字，1~2位）
            uint8_t confidence = 0;
            uint8_t conf_digits = 0;
            while (p < tail && *p >= '0' && *p <= '9')
            {
                confidence = confidence * 10 + static_cast<uint8_t>(*p - '0');
                ++conf_digits;
                ++p;
            }

            if (conf_digits == 0 || conf_digits > 2)
            {
                data_.valid = false;
                return;
            }

            // 6. 确认解析指针刚好停在 \n 之前（即 p == tail）
            if (p != tail)
            {
                data_.valid = false;
                return;
            }

            // 7. 范围校验
            if (distance < 20 || distance > 4500 || confidence > 62)
            {
                data_.valid = false;
                return;
            }

            // 8. 更新有效数据
            data_.distance_mm = distance;
            data_.confidence = confidence;
            data_.valid = true;
        }

        LaserData data_ = {};
    };

}

#endif // !LASER_HPP
