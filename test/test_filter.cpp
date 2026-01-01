#include <gtest/gtest.h>
#include <control/ComplementaryFilter.hpp>
#include <chrono>

using namespace std::chrono_literals;
using drone::control::ComplementaryFilter;

TEST(FilterTest, InitialAngleIsZero) {
    drone::control::ComplementaryFilter<float> filter(0.98f);
    EXPECT_FLOAT_EQ(filter.getAngle(), 0.0f);
}

TEST(FilterTest, ConvergesToAccelerometer) {
    // Alpha de 0.90 significa que el 10% de la corrección viene del acelerómetro
    drone::control::ComplementaryFilter<float> filter(0.90f);
    
    // Si el dron está inclinado a 10 grados y no se mueve (gyro = 0)
    // Tras varias iteraciones, el ángulo debe acercarse a 10
    float angle = 0.0f;
    for(int i = 0; i < 100; ++i) {
        angle = filter.update(10.0f, 0.0f, 10ms);
    }
    
    EXPECT_NEAR(angle, 10.0f, 0.1f);
}

TEST(FilterTest, FiltersOutHighFrequencyNoise) {
    drone::control::ComplementaryFilter<float> filter(0.98f);
    
    // El ángulo real es 0, pero el acelerómetro tiene un pico de ruido de 45 grados
    float output = filter.update(45.0f, 0.0f, 10ms);
    
    // La salida no debería saltar a 45, debería ser un cambio muy pequeño (filtro paso-bajo)
    // 45 * (1 - 0.98) = 0.9 grados
    EXPECT_LT(output, 1.0f);
}