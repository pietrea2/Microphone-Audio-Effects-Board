#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <fstream>

inline std::int32_t double_to_fixed(double input)
{
    return std::lround(input * (1 << 30));
}

#define NUM_SAMPLES 8000

int main(int, char**)
{
    std::fstream file("output.h", std::fstream::out);
    if (!file) return -1;

    file << "#ifndef SIN_LUT_H\n";
    file << "#define SIN_LUT_H\n\n";

    file << "static const int32_t sin_LUT[] = \n{";

    for (int i = 0; i < NUM_SAMPLES; ++i)
    {
        if (i % 10 == 0)
            file << "\n    ";

        auto val = double_to_fixed(sin(((double)i / NUM_SAMPLES) * 2 * M_PI));
        file << val;
        if (i != NUM_SAMPLES - 1) 
            file << ", ";
    }

    file << "\n};\n\n";
    file << "#endif";

    file.close();
}