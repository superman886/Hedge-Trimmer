'''
    适配180度和270度舵机的驱动
'''

from maix import pwm, pinmap

class Servo():
    SERVO_FREQ = 50        # 50Hz 20ms
    SERVO_MIN_DUTY = 2.5   # 2.5% -> 0.5ms
    SERVO_MAX_DUTY = 12.5  # 12.5% -> 2.5ms

    def __init__(self, pwm_id:int, angle:int, max_angle:int=180) -> None:
        """
        初始化舵机
        :param pwm_id: PWM通道ID
        :param angle: 初始角度
        :param max_angle: 舵机最大角度（180或270）
        """
        # 验证最大角度合法性
        if max_angle not in (180, 270):
            raise ValueError("max_angle must be 180 or 270")
        self.max_angle = max_angle

        # 限制初始角度在有效范围
        angle = self.max_angle if angle > self.max_angle else angle
        angle = 0 if angle < 0 else angle
        self.current_angle = angle  # 记录当前角度

        # 配置PWM引脚和初始化
        if pwm_id == 7:
            pinmap.set_pin_function("A19", "PWM7")
            self.pwm = pwm.PWM(pwm_id, freq=Servo.SERVO_FREQ, duty=self._angle_to_duty(angle), enable=True)
        elif pwm_id == 6:
            pinmap.set_pin_function("A18", "PWM6")
            self.pwm = pwm.PWM(pwm_id, freq=Servo.SERVO_FREQ, duty=self._angle_to_duty(angle), enable=True)
        elif pwm_id == 5:
            pinmap.set_pin_function("A17", "PWM5")
            self.pwm = pwm.PWM(pwm_id, freq=Servo.SERVO_FREQ, duty=self._angle_to_duty(angle), enable=True)
        elif pwm_id == 4:
            pinmap.set_pin_function("A16", "PWM4")
            self.pwm = pwm.PWM(pwm_id, freq=Servo.SERVO_FREQ, duty=self._angle_to_duty(angle), enable=True)

    def __del__(self) -> None:
        self.pwm.disable()

    def _angle_to_duty(self, angle:int) -> float:
        '''将角度转换为占空比（根据最大角度动态计算）'''
        return (Servo.SERVO_MAX_DUTY - Servo.SERVO_MIN_DUTY) / self.max_angle * angle + Servo.SERVO_MIN_DUTY
    
    def angle(self, angle:int) -> None:
        '''设定舵机角度（自动限制在有效范围）'''
        angle = self.max_angle if angle > self.max_angle else angle
        angle = 0 if angle < 0 else angle  

        self.current_angle = angle  # 更新当前角度记录
        duty = self._angle_to_duty(angle)
        self.pwm.duty(duty)