cmake_minimum_required(VERSION 3.16)
project(RTES_EMG CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_library(I2C_LIB i2c REQUIRED)

# ── EMG sensor library ───────────────────────────────────────────────────────
add_library(emgsensors
    src/sensors/EMGSensors.cpp
    src/EMGLogger.cpp
)

# ── Motor control library ─────────────────────────────────────────────────────
add_library(motorcontrol
    src/motor/PCA9685.cpp
    src/motor/MotorController.cpp
)

target_include_directories(motorcontrol
    PUBLIC include
)

target_link_libraries(motorcontrol
    PUBLIC
        ${I2C_LIB}
)

target_include_directories(emgsensors
    PUBLIC include
)

target_link_libraries(emgsensors
    PUBLIC
        ${I2C_LIB}
        pthread
)

# ── Main application ─────────────────────────────────────────────────────────
add_executable(main
    apps/main.cpp
)

target_link_libraries(main
    PRIVATE emgsensors
    PRIVATE motorcontrol
)

# ── Servo calibration utility ─────────────────────────────────────────────────
add_executable(servo_calibration
    apps/servo_calibration.cpp
)

target_link_libraries(servo_calibration
    PRIVATE motorcontrol
)