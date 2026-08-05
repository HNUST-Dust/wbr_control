#include <algorithms/utils/alg_fast_math.h>

namespace alg {

float Sqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    float y = x * 0.5f;
    const float maxError = x * 0.001f;

    float delta = 0.0f;
    do {
        delta = (y * y) - x;
        y -= delta / (2.0f * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

} // namespace alg
