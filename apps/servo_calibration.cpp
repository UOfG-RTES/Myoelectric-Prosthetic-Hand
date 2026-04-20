#include "motor/PCA9685.hpp"
#include "motor/MotorController.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

/**
 * Servo calibration utility.
 *
 * Moves each servo to user-specified angles so you can identify
 * the correct open/closed positions for MotorController constants.
 *
 * Usage:
 *   sudo ./servo_calibration
 *
 * Commands (type at prompt):
 *   <channel> <angle>   e.g.  "0 90"  — move channel 0 to 90°
 *   q                   — quit
 *
 * Once you find the correct angles, update these constants in
 * include/motor/MotorController.hpp:
 *   CLOSE_SERVO_OPEN    channel 0 angle when hand is open
 *   CLOSE_SERVO_CLOSED  channel 0 angle when hand is closed
 *   OPEN_SERVO_OPEN     channel 2 angle when hand is open
 *   OPEN_SERVO_CLOSED   channel 2 angle when hand is closed
 */
int main() {
    PCA9685 pwm(1, 0x40);
    try {
        pwm.init();
    } catch (const std::exception& e) {
        fprintf(stderr, "PCA9685 init failed: %s\n", e.what());
        return 1;
    }

    printf("\n=== Servo Calibration ===\n");
    printf("Channels in use:\n");
    printf("  Channel %d — close servo (pulls fingers)\n", MotorController::CLOSE_CHANNEL);
    printf("  Channel %d — open  servo (releases fingers)\n", MotorController::OPEN_CHANNEL);
    printf("\nCurrent placeholder angles:\n");
    printf("  CLOSE_SERVO_OPEN   = %.0f°\n", MotorController::CLOSE_SERVO_OPEN);
    printf("  CLOSE_SERVO_CLOSED = %.0f°\n", MotorController::CLOSE_SERVO_CLOSED);
    printf("  OPEN_SERVO_OPEN    = %.0f°\n", MotorController::OPEN_SERVO_OPEN);
    printf("  OPEN_SERVO_CLOSED  = %.0f°\n", MotorController::OPEN_SERVO_CLOSED);
    printf("\nType: <channel> <angle>  (e.g. \"0 90\")\n");
    printf("      q  to quit\n\n");

    // Start both servos at their current open positions
    pwm.setAngle(MotorController::CLOSE_CHANNEL, MotorController::CLOSE_SERVO_OPEN);
    pwm.setAngle(MotorController::OPEN_CHANNEL,  MotorController::OPEN_SERVO_OPEN);
    printf("Servos moved to current OPEN positions.\n\n");

    char line[64];
    while (true) {
        printf("ch angle> ");
        fflush(stdout);

        if (!fgets(line, sizeof(line), stdin)) break;
        if (line[0] == 'q' || line[0] == 'Q') break;

        int   channel   = -1;
        float angle_deg = -1.0f;

        if (sscanf(line, "%d %f", &channel, &angle_deg) != 2) {
            printf("  Invalid input — enter: <channel> <angle>  e.g. \"0 90\"\n");
            continue;
        }
        if (channel != MotorController::CLOSE_CHANNEL &&
            channel != MotorController::OPEN_CHANNEL) {
            printf("  Channel must be %d or %d\n",
                   MotorController::CLOSE_CHANNEL, MotorController::OPEN_CHANNEL);
            continue;
        }
        if (angle_deg < 0.0f || angle_deg > 180.0f) {
            printf("  Angle must be 0–180°\n");
            continue;
        }

        pwm.setAngle(channel, angle_deg);
        printf("  Channel %d → %.0f°\n", channel, angle_deg);
    }

    printf("\n=== Update MotorController.hpp with your calibrated values ===\n");
    printf("  CLOSE_SERVO_OPEN   = ?°   (ch %d, hand fully open)\n",
           MotorController::CLOSE_CHANNEL);
    printf("  CLOSE_SERVO_CLOSED = ?°   (ch %d, hand fully closed)\n",
           MotorController::CLOSE_CHANNEL);
    printf("  OPEN_SERVO_OPEN    = ?°   (ch %d, hand fully open)\n",
           MotorController::OPEN_CHANNEL);
    printf("  OPEN_SERVO_CLOSED  = ?°   (ch %d, hand fully closed)\n",
           MotorController::OPEN_CHANNEL);

    return 0;
}
