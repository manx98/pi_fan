#!/usr/bin/env python3
from luma.oled.device import sh1106
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from PIL import ImageFont
import argparse
import os
import subprocess
import time
from pathlib import Path

import pigpio
import psutil


font_size = 9
offset = 0

# GPIO 使用 BCM 编号。
# GPIO12 = 物理引脚 32，支持硬件 PWM。
TEMP_CHANNEL = 12

# 温度阈值控制：
# temp >= MAX_TEMP：开启风扇，按固定占空比运行
# temp <= MIN_TEMP：关闭风扇
# MIN_TEMP < temp < MAX_TEMP：保持上一次状态，避免频繁启停
MIN_TEMP = 45
MAX_TEMP = 50

# 4 线 PWM 风扇常用 25kHz
PWM_FREQ = 25_000

# 用户设定的固定转速，占空比 0~100
DEFAULT_FAN_DUTY = 60

# 用户运行时控制文件。写入 0~100 的数字即可修改“风扇开启时”的固定转速。
# 例如：echo 70 | sudo tee /run/oled-fan-duty
CONTROL_FILE = os.environ.get("FAN_DUTY_FILE", "/run/oled-fan-duty")

BASE_DIR = Path(__file__).resolve().parent
FONT_PATH = BASE_DIR / "AlibabaPuHuiTi-2-55-Regular.ttf"

font = ImageFont.truetype(str(FONT_PATH), size=font_size)
address = i2c(address=0x3c, port=1)

# rotate=2：OLED 屏幕旋转 180 度
device = sh1106(address=address, width=128, height=64, rotate=2)


def clamp_duty(duty):
    duty = float(duty)
    if duty < 0:
        return 0.0
    if duty > 100:
        return 100.0
    return round(duty, 1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="OLED monitor with threshold-controlled fixed-speed pigpio PWM fan"
    )
    parser.add_argument(
        "--fan-duty",
        type=float,
        default=None,
        help="风扇开启时的固定占空比，范围 0~100。优先级高于 FAN_DUTY 环境变量。"
    )
    parser.add_argument(
        "--fan-duty-file",
        default=None,
        help="运行时控制文件，写入 0~100 可修改风扇开启时的固定转速，默认 /run/oled-fan-duty。"
    )
    parser.add_argument(
        "--min-temp",
        type=float,
        default=None,
        help="低于或等于该温度时关闭风扇。优先级高于 MIN_TEMP 环境变量。"
    )
    parser.add_argument(
        "--max-temp",
        type=float,
        default=None,
        help="高于或等于该温度时开启风扇。优先级高于 MAX_TEMP 环境变量。"
    )
    return parser.parse_args()


def get_float_config(arg_value, env_name, default_value):
    if arg_value is not None:
        return float(arg_value)

    env_value = os.environ.get(env_name)
    if env_value not in (None, ""):
        return float(env_value)

    return float(default_value)


def get_initial_duty(args):
    if args.fan_duty is not None:
        return clamp_duty(args.fan_duty)

    env_duty = os.environ.get("FAN_DUTY")
    if env_duty not in (None, ""):
        return clamp_duty(env_duty)

    return clamp_duty(DEFAULT_FAN_DUTY)


class PigpioThresholdPWMFan:
    """
    使用 pigpio 控制 GPIO12 硬件 PWM。

    控制逻辑：
    - 温度 >= max_temp：开启风扇，输出 fixed_duty
    - 温度 <= min_temp：关闭风扇，输出 0
    - 中间区间：保持当前开关状态，避免频繁启停

    注意：
    1. 需要先启动 pigpiod：
       sudo systemctl enable --now pigpiod

    2. GPIO 编号使用 BCM 编号。
       GPIO12 = 物理引脚 32。
    """

    def __init__(self, gpio, freq, fixed_duty, min_temp, max_temp):
        if min_temp >= max_temp:
            raise ValueError(f"MIN_TEMP 必须小于 MAX_TEMP，当前 MIN_TEMP={min_temp}, MAX_TEMP={max_temp}")

        self.gpio = gpio
        self.freq = freq
        self.fixed_duty = clamp_duty(fixed_duty)
        self.min_temp = float(min_temp)
        self.max_temp = float(max_temp)
        self.enabled = False
        self.last_output_duty = None

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError(
                "连接 pigpiod 失败。请确认 pigpiod 已启动：sudo systemctl enable --now pigpiod"
            )

        self.mode = f"pigpio hardware_PWM GPIO{gpio} {freq}Hz"
        self._write_pwm(0)

    def _write_pwm(self, output_duty):
        output_duty = clamp_duty(output_duty)
        if output_duty == self.last_output_duty:
            return output_duty

        # pigpio.hardware_PWM 的 dutycycle 范围是 0 ~ 1_000_000
        duty_million = int(output_duty * 10_000)
        rc = self.pi.hardware_PWM(self.gpio, self.freq, duty_million)
        if rc != 0:
            raise RuntimeError(
                f"pigpio.hardware_PWM(GPIO{self.gpio}, {self.freq}, {duty_million}) 失败，返回值 {rc}"
            )

        self.last_output_duty = output_duty
        print(f"fan output duty set to {output_duty}%")
        return output_duty

    def set_fixed_duty(self, duty):
        self.fixed_duty = clamp_duty(duty)
        print(f"fan fixed duty updated to {self.fixed_duty}%")
        if self.enabled:
            self._write_pwm(self.fixed_duty)
        return self.fixed_duty

    def update_by_temp(self, temp):
        temp = float(temp)

        if temp >= self.max_temp:
            if not self.enabled:
                print(f"CPU temp {temp:.2f} >= {self.max_temp:.2f}, fan ON")
            self.enabled = True
        elif temp <= self.min_temp:
            if self.enabled:
                print(f"CPU temp {temp:.2f} <= {self.min_temp:.2f}, fan OFF")
            self.enabled = False

        if self.enabled:
            return self._write_pwm(self.fixed_duty)

        return self._write_pwm(0)

    def cleanup(self):
        try:
            self._write_pwm(0)
            self.pi.stop()
        except Exception:
            pass


def read_duty_from_file(path):
    """
    从控制文件读取固定占空比。
    文件不存在时返回 None。
    文件内容非法时忽略，保留上一次设置。
    """
    p = Path(path)
    if not p.exists():
        return None

    try:
        text = p.read_text(encoding="utf-8").strip()
        if not text:
            return None
        return clamp_duty(text)
    except Exception as e:
        print(f"ignore invalid fan duty file {path}: {e}")
        return None


def get_disk_info():
    disk = psutil.disk_usage("/")
    return "{}/{}G {}%".format(
        round(disk.used / (1024 * 1024 * 1024), 2),
        round(disk.total / (1024 * 1024 * 1024), 2),
        disk.percent
    )


def get_mem_str():
    mem = psutil.virtual_memory()
    return "{}/{}M {}%".format(
        round(mem.used / 1024 / 1024),
        round(mem.total / 1024 / 1024),
        mem.percent
    )


def get_cpu_temp():
    with open("/sys/class/thermal/thermal_zone0/temp") as tempfile:
        cpu_temp = tempfile.read()
    return float(cpu_temp) / 1000


def get_ip():
    cmd = "hostname -I | cut -d' ' -f1"
    return subprocess.check_output(cmd, shell=True).decode("utf8").strip()


def filesizeformat(bytes_value, precision=1):
    for unit in ["B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB"]:
        if abs(bytes_value) < 1024.0:
            return "%s %s" % (format(bytes_value, ".%df" % precision), unit)
        bytes_value /= 1024.0
    return "%s %s" % (format(bytes_value, ".%df" % precision), "Yi")


class NetSpeedUtil:
    def __init__(self):
        self.speed_str = "loading..."
        self.last_time = None
        self.last_data = None

    def compute(self):
        lt = time.time()
        ld = psutil.net_io_counters()
        if self.last_data:
            ct = lt - self.last_time
            if ct > 0:
                cr = (ld.bytes_recv - self.last_data.bytes_recv) / ct
                cs = (ld.bytes_sent - self.last_data.bytes_sent) / ct
                self.speed_str = "↑{}/s ↓{}/s".format(filesizeformat(cs), filesizeformat(cr))
        self.last_data = ld
        self.last_time = lt

    def get_speed(self):
        self.compute()
        return self.speed_str


def show_state(netSpeed, fan, control_file, show=False):
    duty_from_file = read_duty_from_file(control_file)
    if duty_from_file is not None and duty_from_file != fan.fixed_duty:
        fan.set_fixed_duty(duty_from_file)

    ca = canvas(device)
    with ca as draw:
        if offset > 0:
            draw.rectangle(device.bounding_box, outline="white", fill="black")

        temp = get_cpu_temp()
        output_duty = fan.update_by_temp(temp)

        draw.text((offset, 0), "IP:%s" % get_ip(), fill="white", font=font)
        draw.text(
            (offset, font_size),
            "CPU {}% {:.2f}℃".format(psutil.cpu_percent(interval=1), temp),
            fill="white",
            font=font
        )
        draw.text((offset, font_size * 2), "Mem:%s" % get_mem_str(), fill="white", font=font)
        draw.text((offset, font_size * 3), "Disk:%s" % get_disk_info(), fill="white", font=font)
        draw.text((offset, font_size * 4), "%s" % netSpeed.get_speed(), fill="white", font=font)
        draw.text(
            (offset, font_size * 5),
            "Fan:{} set:{}% out:{}%".format("ON" if fan.enabled else "OFF", fan.fixed_duty, output_duty),
            fill="white",
            font=font
        )

        if show:
            try:
                from IPython.display import display
                display(ca.image)
            except Exception:
                pass


if __name__ == "__main__":
    args = parse_args()
    if args.fan_duty_file:
        CONTROL_FILE = args.fan_duty_file

    min_temp = get_float_config(args.min_temp, "MIN_TEMP", MIN_TEMP)
    max_temp = get_float_config(args.max_temp, "MAX_TEMP", MAX_TEMP)
    initial_duty = get_initial_duty(args)

    fan = PigpioThresholdPWMFan(
        gpio=TEMP_CHANNEL,
        freq=PWM_FREQ,
        fixed_duty=initial_duty,
        min_temp=min_temp,
        max_temp=max_temp,
    )
    netSpeed = NetSpeedUtil()

    print("running...")
    print("fan mode:", fan.mode)
    print("fan fixed duty:", initial_duty)
    print("fan min temp:", min_temp)
    print("fan max temp:", max_temp)
    print("fan duty control file:", CONTROL_FILE)

    try:
        while True:
            show_state(netSpeed, fan, CONTROL_FILE)
            time.sleep(0.5)
    finally:
        fan.cleanup()
