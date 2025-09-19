from maix import pwm, time, pinmap

SERVO_FREQ = 50        # 50Hz 20ms
SERVO_MIN_DUTY = 2.5   # 2.5% -> 0.5ms
SERVO_MAX_DUTY = 12.5  # 12.5% -> 2.5ms



# Use PWM7
pwm_id = 6
# !! set pinmap to use PWM7
pinmap.set_pin_function("A18", "PWM6")


def angle_to_duty(angle):
    return (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 270 * angle + SERVO_MIN_DUTY


out = pwm.PWM(pwm_id, freq=SERVO_FREQ, duty=angle_to_duty(0), enable=True)

for i in range(270):
    out.duty(angle_to_duty(i))
    print(i)
    time.sleep_ms(100)

# for i in range(180):
#     out.duty(angle_to_duty(i))
#     print(i)
#     time.sleep_ms(100)