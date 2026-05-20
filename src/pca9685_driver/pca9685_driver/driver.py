from adafruit_pca9685 import PCA9685
import board
import busio

class PCA9685Driver:

    def __init__(self, frequency=50):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = frequency

    def set_pwm_us(self, channel, pulse_us):
        # PCA9685 trabaja en duty cycle
        # 50Hz → 20ms period → 1 tick = 4.096µs aprox
        pulse_length = 1000000 / self.pca.frequency
        duty = int((pulse_us / pulse_length) * 65535)

        self.pca.channels[channel].duty_cycle = duty