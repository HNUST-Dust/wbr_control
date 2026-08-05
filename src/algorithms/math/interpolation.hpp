/**
 * @file interpolation.hpp
 * @brief 简单线性插值/分段输出工具（历史实现）
 *
 * 设计思路：
 * =========
 * - 用于在 `point_num` 个周期内，从 `start_point` 线性过渡到 `end_point`。
 * - `Output()` 每调用一次推进一步，并输出当前插值值。
 *
 * 注意事项：
 * =========
 * - `Output()` 内部使用了 `static uint8_t times;` 作为计数器，这会导致计数器在所有实例之间共享。
 *   因此该类并非“多实例互不影响”的严格意义对象；也不适合并发/多任务共享使用。
 * - 若需要更严格的实例隔离，可将计数器改为成员变量（但这属于逻辑改动，本次仅文档化）。
 */

#ifndef WBR_CONTROL_ALGORITHMS_MATH_INTERPOLATION_HPP_
#define WBR_CONTROL_ALGORITHMS_MATH_INTERPOLATION_HPP_

#include <cstdint>
#include <stdint.h>

enum InterpolationState
{
    INTERPOLATION_STATE_START,
    INTERPOLATION_STATE_ING,
    INTERPOLATION_STATE_END
};

class Interpolation 
{
private:
    float start_point_;
    float end_point_;
    uint8_t point_num_;
    float output_;
    InterpolationState state_ = INTERPOLATION_STATE_END;
public:
    void Start(float start_point,float end_point,uint8_t point_num)
    {
        start_point_ = start_point;
        end_point_ = end_point;
        point_num_ = point_num;
        state_ = INTERPOLATION_STATE_START;
    }
    float Output()
    {
        static uint8_t times;
        times++;
        if((times >= point_num_ )||(state_ == INTERPOLATION_STATE_END)){
            output_ = end_point_;
            times = 0;
            state_ = INTERPOLATION_STATE_END;
        }else{
            output_ = start_point_ + (((end_point_ - start_point_) / point_num_) * times);
            state_ = INTERPOLATION_STATE_ING;
        }
        return output_;
    }

};



#endif /* WBR_CONTROL_ALGORITHMS_MATH_INTERPOLATION_HPP_ */
