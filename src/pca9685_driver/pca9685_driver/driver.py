from adafruit_pca9685 import PCA9685
import busio
import board


class PCA9685Driver:

    def __init__(self, frequency=50):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = frequency

    def set_pwm_us(self, channel, pulse_us):
        period_us = 1_000_000 / self.pca.frequency
        duty = int((pulse_us / period_us) * 65535)

        duty = max(0, min(65535, duty))  # safety clamp

        self.pca.channels[channel].duty_cycle = duty