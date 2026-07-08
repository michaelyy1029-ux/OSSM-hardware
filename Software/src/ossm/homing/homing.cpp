#include "homing.h"

#include "Strings.h"
#include "homing_logic.h"
#include "constants/Config.h"
#include "constants/Pins.h"
#include "constants/UserConfig.h"
#include "ossm/Events.h"
#include "ossm/state/calibration.h"
#include "ossm/state/error.h"
#include "ossm/state/state.h"
#include "services/led.h"
#include "services/stepper.h"
#include "services/tasks.h"
#include "utils/analog.h"

namespace sml = boost::sml;
using namespace sml;

namespace homing {

void clearHoming() {
    ESP_LOGD("Homing", "Homing started");

    pinMode(Pins::Driver::limitSwitchPin, INPUT_PULLUP); // 确保限位开关引脚有上拉电阻

    // Set homing active flag for LED indication
    setHomingActive(true);

    calibration.isForward = true;

    // Set acceleration and deceleration in steps/s^2
    stepper->setAcceleration(1000_mm);
    // Set speed in steps/s
    stepper->setSpeedInHz(25_mm);

    // Clear the stored values.
    calibration.measuredStrokeSteps = 0;

    /*
    // Recalibrate the current sensor offset.
    calibration.currentSensorOffset = (getAnalogAveragePercent(
        SampleOnPin{Pins::Driver::currentSensorPin, 1000}));
    */
}

static void startHomingTask(void *pvParameters) {
    TickType_t xTaskStartTime = xTaskGetTickCount();

#ifdef AJ_DEVELOPMENT_HARDWARE
    stepper->setCurrentPosition(0);
    stepper->forceStopAndNewPosition(0);
    stateMachine->process_event(Done{});
    vTaskDelete(nullptr);
    return;
#endif

    // Stroke Engine and Simple Penetration treat this differently.
    stepper->enableOutputs();
    stepper->setDirectionPin(Pins::Driver::motorDirectionPin, false);
    int16_t sign = stateMachine->is("homing.backward"_s) ? 1 : -1;

    int32_t targetPositionInSteps =
        round(sign * Config::Driver::maxStrokeSteps);

    ESP_LOGD("Homing", "Target position in steps: %d", targetPositionInSteps);
    stepper->moveTo(targetPositionInSteps, false);

    auto isInCorrectState = []() {
        // Add any states that you want to support here.
        return stateMachine->is("homing"_s) ||
               stateMachine->is("homing.forward"_s) ||
               stateMachine->is("homing.backward"_s);
    };

    // run loop for 15second or until loop exits
    while (isInCorrectState()) {
        TickType_t xCurrentTickCount = xTaskGetTickCount();
        // Calculate the time in ticks that the task has been running.
        TickType_t xTicksPassed = xCurrentTickCount - xTaskStartTime;

        // If you need the time in milliseconds, convert ticks to milliseconds.
        // 'portTICK_PERIOD_MS' is the number of milliseconds per tick.
        uint32_t msPassed = xTicksPassed * portTICK_PERIOD_MS;

        if (homing_logic::isHomingTimedOut(msPassed, 40000)) {
            ESP_LOGE("Homing", "Homing took too long. Check power and restart");
            errorState.message = ui::strings::homingTookTooLong;

            // Clear homing active flag for LED indication
            setHomingActive(false);

            stateMachine->process_event(Error{});
            break;
        }

        // 【删除/注释掉原有的电流检测】
        /*
        // measure the current analog value.
        float current = getAnalogAveragePercent(
                            SampleOnPin{Pins::Driver::currentSensorPin, 50}) -
                        calibration.currentSensorOffset;

        ESP_LOGV("Homing", "Current: %f", current);
        bool isCurrentOverLimit = homing_logic::isCurrentOverLimit(
            current, 0, Config::Driver::sensorlessCurrentLimit);

        if (!isCurrentOverLimit) {
            vTaskDelay(10);  // Increased from 1ms to 10ms to reduce CPU load
            continue;
        }

        ESP_LOGD("Homing", "Current over limit: %f", current);
        stepper->stopMove();

        stepper->setSpeedInHz(250_mm);
        */

        // 【替换为机械限位开关检测】
        // 读取引脚 12，如果是低电平(LOW)，说明碰到了限位器
        bool isLimitSwitchPressed = (digitalRead(Pins::Driver::limitSwitchPin) == LOW); 

        if (!isLimitSwitchPressed) { 
            vTaskDelay(10);  // 没碰到就继续等待
            continue;
        }

        // 【以下为触发限位后的处理逻辑】
        ESP_LOGD("Homing", "Limit switch pressed!"); 
        stepper->stopMove(); 
        stepper->setSpeedInHz(250_mm);

        // step away from the hard stop, with your hands in the air!
        int32_t currentPosition = stepper->getCurrentPosition();
        stepper->moveTo(currentPosition - sign * Config::Driver::homingOffsetMn,
                        true);

        /*
        // measure and save the current position
        calibration.measuredStrokeSteps =
            homing_logic::calculateMeasuredStroke(
                stepper->getCurrentPosition(),
                Config::Driver::maxStrokeSteps);
        */

        // 强制指定当前位置和总行程
        // 既然只有单向限位，我们直接欺骗系统，告诉它已经测满了 140mm 轨道
        calibration.measuredStrokeSteps = Config::Driver::maxStrokeSteps;

        stepper->setCurrentPosition(0);
        stepper->forceStopAndNewPosition(0);

        int32_t goToPosition = homing_logic::calculatePostHomingPosition(
            sign, calibration.measuredStrokeSteps,
            UserConfig::afterHomingPosition);

        stepper->moveTo(goToPosition,true);

        // Clear homing active flag for LED indication
        setHomingActive(false);

        stateMachine->process_event(Done{});
        break;
    };

    vTaskDelete(nullptr);
}

void startHoming() {
    int stackSize = 10 * configMINIMAL_STACK_SIZE;
    xTaskCreatePinnedToCore(startHomingTask, "startHomingTask", stackSize,
                            nullptr, configMAX_PRIORITIES - 1,
                            &Tasks::runHomingTaskH, Tasks::operationTaskCore);
}

bool isStrokeTooShort() {
    if (calibration.measuredStrokeSteps > Config::Driver::minStrokeLengthMm) {
        return false;
    }
    errorState.message = ui::strings::strokeTooShort;
    return true;
}

}  // namespace homing
