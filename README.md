STM32F103C8 - Bluepill

Uni-polar Sine-wave Inverter

PWM HA 		     -> GPIO PIN 8, PWM LA 		     -> GPIO PIN 7
Fundamental HB -> GPIO PIN 9, Fundamental LB -> GPIO PIN 10

Using Complimentary Center-Aligned PWM, f = (clock_source/(pre-scaler)/period, Open tim.c to see the detail
Duty max @ period, ISR Event TIM1 call @ Top of counter and @ counter reset (twice per cycle),
ISR called function "ISR_SINE" to calculate the duty cycle for sine generation.

For output Low Pass Filter, find cut-off resonance frequency -> f_cut = 1/(2*pi*(LC)^-1)
i use L = 0.1 H with C = 47/2 uF to get ~ 100 Hz Cut-off frequency.
