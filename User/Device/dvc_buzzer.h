#ifndef DVC_BUZZER_H
#define DVC_BUZZER_H
#include "stdint.h"

#define MAX_PSC                 1000

#define MAX_BUZZER_PWM      20000
#define MIN_BUZZER_PWM      10000

void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);


#endif