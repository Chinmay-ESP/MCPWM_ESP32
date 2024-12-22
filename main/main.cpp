#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM0A_OUT 22
#define GPIO_PWM0B_OUT 23 

class BrushedDCMotor {
public:
    BrushedDCMotor(mcpwm_unit_t mcpwm_unit, mcpwm_timer_t timer, int gpio_pwm0a, int gpio_pwm0b)
        : mcpwm_unit_(mcpwm_unit), timer_(timer), gpio_pwm0a_(gpio_pwm0a), gpio_pwm0b_(gpio_pwm0b) {}

    void init() {
        mcpwm_gpio_init(mcpwm_unit_, MCPWM0A, gpio_pwm0a_);
        mcpwm_gpio_init(mcpwm_unit_, MCPWM0B, gpio_pwm0b_);


        mcpwm_config_t pwm_config;
        pwm_config.frequency = 1000; // Frequency = 1 kHz
        pwm_config.cmpr_a = 0;       // Duty cycle of PWMxA = 0
        pwm_config.cmpr_b = 0;       // Duty cycle of PWMxB = 0
        pwm_config.counter_mode = MCPWM_UP_COUNTER;
        pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

        mcpwm_init(mcpwm_unit_, timer_, &pwm_config); 
    }

    void forward(float duty_cycle) {
        mcpwm_set_signal_low(mcpwm_unit_, timer_, MCPWM_OPR_B);
        mcpwm_set_duty(mcpwm_unit_, timer_, MCPWM_OPR_A, duty_cycle);
        mcpwm_set_duty_type(mcpwm_unit_, timer_, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }

    void backward(float duty_cycle) {
        mcpwm_set_signal_low(mcpwm_unit_, timer_, MCPWM_OPR_A);
        mcpwm_set_duty(mcpwm_unit_, timer_, MCPWM_OPR_B, duty_cycle);
        mcpwm_set_duty_type(mcpwm_unit_, timer_, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }

    void stop() {
        mcpwm_set_signal_low(mcpwm_unit_, timer_, MCPWM_OPR_A);
        mcpwm_set_signal_low(mcpwm_unit_, timer_, MCPWM_OPR_B);
    }

private:
    mcpwm_unit_t mcpwm_unit_;
    mcpwm_timer_t timer_;
    int gpio_pwm0a_;
    int gpio_pwm0b_;
};

extern "C" void app_main() {
    std::cout << "Testing brushed motor..." << std::endl;

    BrushedDCMotor motor(MCPWM_UNIT_0, MCPWM_TIMER_0, GPIO_PWM0A_OUT, GPIO_PWM0B_OUT);
    motor.init();

    while (true) {
        std::cout << "Moving forward..." << std::endl;
        motor.forward(50.0);
        vTaskDelay(pdMS_TO_TICKS(2000));

        std::cout << "Moving backward..." << std::endl;
        motor.backward(30.0);
        vTaskDelay(pdMS_TO_TICKS(2000));

        std::cout << "Stopping motor..." << std::endl;
        motor.stop();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
