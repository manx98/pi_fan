#!/usr/bin/env python3
from luma.oled.device import sh1106
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from PIL import ImageFont
import argparse
import mmap
import os
import signal
import struct
import subprocess
import time
from pathlib import Path

import psutil


font_size = 9
offset = 0

# GPIO 使用 BCM 编号。
# GPIO12 = 物理引脚 32，支持 PWM0 硬件输出，ALT0 功能。
TEMP_CHANNEL = 12

MIN_TEMP = 45
MAX_TEMP = 50

# 4 线 PWM 风扇常用 25kHz
PWM_FREQ = 25_000
PWM_RANGE = 1000

# 用户设定的固定转速，占空比 0~100
DEFAULT_FAN_DUTY = 60

# 用户运行时控制文件。写入 0~100 的数字即可修改“风扇开启时”的固定转速。
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
        description="OLED monitor with threshold-controlled fixed-speed direct-register PWM fan"
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
    parser.add_argument(
        "--pwm-gpio",
        type=int,
        default=None,
        help="硬件 PWM GPIO，默认 GPIO12。支持 GPIO12/13/18/19。"
    )
    parser.add_argument(
        "--pwm-range",
        type=int,
        default=None,
        help="PWM range，默认 1000。占空比会映射到 0~range。"
    )
    return parser.parse_args()


def get_float_config(arg_value, env_name, default_value):
    if arg_value is not None:
        return float(arg_value)

    env_value = os.environ.get(env_name)
    if env_value not in (None, ""):
        return float(env_value)

    return float(default_value)


def get_int_config(arg_value, env_name, default_value):
    if arg_value is not None:
        return int(arg_value)

    env_value = os.environ.get(env_name)
    if env_value not in (None, ""):
        return int(env_value)

    return int(default_value)


def get_initial_duty(args):
    if args.fan_duty is not None:
        return clamp_duty(args.fan_duty)

    env_duty = os.environ.get("FAN_DUTY")
    if env_duty not in (None, ""):
        return clamp_duty(env_duty)

    return clamp_duty(DEFAULT_FAN_DUTY)


class RegisterBlock:
    def __init__(self, fd, phys_addr, size):
        page_size = mmap.PAGESIZE
        page_mask = page_size - 1
        self.page_base = phys_addr & ~page_mask
        self.page_offset = phys_addr - self.page_base
        self.map_size = ((self.page_offset + size + page_mask) // page_size) * page_size
        self.mem = mmap.mmap(
            fd,
            self.map_size,
            flags=mmap.MAP_SHARED,
            prot=mmap.PROT_READ | mmap.PROT_WRITE,
            offset=self.page_base,
        )

    def read32(self, offset):
        return struct.unpack_from("<I", self.mem, self.page_offset + offset)[0]

    def write32(self, offset, value):
        struct.pack_into("<I", self.mem, self.page_offset + offset, value & 0xFFFFFFFF)

    def close(self):
        self.mem.close()


class DirectRegisterThresholdPWMFan:
    """
    使用 /dev/mem 直接配置树莓派 GPIO/PWM/Clock 寄存器。

    这个版本不依赖 pigpio/pigpiod，也不依赖 /sys/class/pwm。
    适用于 BCM2835/BCM2836/BCM2837/BCM2711 这类传统树莓派 SoC。
    Raspberry Pi 5 使用 RP1 IO 控制器，寄存器布局不同，本类不支持。
    """

    GPIO_BASE_OFFSET = 0x00200000
    PWM_BASE_OFFSET = 0x0020C000
    CLK_BASE_OFFSET = 0x00101000

    BLOCK_SIZE = 0x1000

    GPFSEL0 = 0x00

    PWM_CTL = 0x00
    PWM_STA = 0x04
    PWM_RNG1 = 0x10
    PWM_DAT1 = 0x14
    PWM_RNG2 = 0x20
    PWM_DAT2 = 0x24

    PWM_CTL_PWEN1 = 1 << 0
    PWM_CTL_MSEN1 = 1 << 7
    PWM_CTL_PWEN2 = 1 << 8
    PWM_CTL_MSEN2 = 1 << 15

    CLK_PWMCTL = 0xA0
    CLK_PWMDIV = 0xA4

    BCM_PASSWORD = 0x5A000000
    CLK_CTL_BUSY = 1 << 7
    CLK_CTL_KILL = 1 << 5
    CLK_CTL_ENAB = 1 << 4
    CLK_CTL_SRC_PLLD = 6

    GPIO_PWM_MAP = {
        12: (0, 4),  # GPIO12 ALT0 -> PWM0
        13: (1, 4),  # GPIO13 ALT0 -> PWM1
        18: (0, 2),  # GPIO18 ALT5 -> PWM0
        19: (1, 2),  # GPIO19 ALT5 -> PWM1
    }

    def __init__(self, gpio, freq, pwm_range, fixed_duty, min_temp, max_temp):
        if min_temp >= max_temp:
            raise ValueError(f"MIN_TEMP 必须小于 MAX_TEMP，当前 MIN_TEMP={min_temp}, MAX_TEMP={max_temp}")

        if gpio not in self.GPIO_PWM_MAP:
            raise ValueError("硬件 PWM 仅支持 GPIO12、GPIO13、GPIO18、GPIO19")

        if int(freq) <= 0:
            raise ValueError("PWM_FREQ 必须大于 0")

        if int(pwm_range) <= 0:
            raise ValueError("PWM_RANGE 必须大于 0")

        self.gpio = int(gpio)
        self.freq = int(freq)
        self.pwm_range = int(pwm_range)
        self.pwm_channel, self.gpio_alt = self.GPIO_PWM_MAP[self.gpio]
        self.fixed_duty = clamp_duty(fixed_duty)
        self.min_temp = float(min_temp)
        self.max_temp = float(max_temp)
        self.enabled = False
        self.last_output_duty = None

        self.compatible = self._read_compatible()
        if "bcm2712" in self.compatible or "rp1" in self.compatible:
            raise RuntimeError("当前系统看起来是 Raspberry Pi 5/RP1，寄存器布局不同，当前直控实现不支持")

        self.peripheral_base = self._detect_peripheral_base()
        self.plld_freq = self._detect_plld_frequency()
        self.clock_divider = self.plld_freq / (self.freq * self.pwm_range)
        if self.clock_divider < 1.0 or self.clock_divider >= 4096.0:
            raise RuntimeError(
                f"无法生成 {self.freq}Hz PWM：PLLD={self.plld_freq}, range={self.pwm_range}, divider={self.clock_divider:.4f}"
            )

        try:
            self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        except PermissionError as e:
            raise RuntimeError("打开 /dev/mem 失败：权限不足，请用 root 运行服务") from e
        except OSError as e:
            raise RuntimeError(f"打开 /dev/mem 失败：{e}") from e

        self.gpio_regs = None
        self.pwm_regs = None
        self.clk_regs = None

        try:
            self.gpio_regs = RegisterBlock(
                self.fd,
                self.peripheral_base + self.GPIO_BASE_OFFSET,
                self.BLOCK_SIZE,
            )
            self.pwm_regs = RegisterBlock(
                self.fd,
                self.peripheral_base + self.PWM_BASE_OFFSET,
                self.BLOCK_SIZE,
            )
            self.clk_regs = RegisterBlock(
                self.fd,
                self.peripheral_base + self.CLK_BASE_OFFSET,
                self.BLOCK_SIZE,
            )
            self.mode = (
                f"direct register PWM GPIO{self.gpio} PWM{self.pwm_channel} "
                f"{self.freq}Hz range={self.pwm_range} base=0x{self.peripheral_base:X}"
            )
            self._init_pwm_output()
        except Exception:
            self.close()
            raise

    def _read_compatible(self):
        try:
            data = Path("/proc/device-tree/compatible").read_bytes()
            return data.replace(b"\x00", b" ").decode("ascii", errors="ignore").lower()
        except OSError:
            return ""

    def _detect_peripheral_base(self):
        try:
            data = Path("/proc/device-tree/soc/ranges").read_bytes()
            cells = [int.from_bytes(data[i:i + 4], "big") for i in range(0, len(data) - 3, 4)]
            for i in range(0, len(cells) - 2):
                if cells[i] == 0x7E000000:
                    if i + 3 < len(cells) and cells[i + 1] == 0:
                        return cells[i + 2]
                    return cells[i + 1]
        except OSError:
            pass

        if "bcm2711" in self.compatible:
            return 0xFE000000
        if "bcm2836" in self.compatible or "bcm2837" in self.compatible or "bcm2709" in self.compatible:
            return 0x3F000000
        return 0x20000000

    def _detect_plld_frequency(self):
        if "bcm2711" in self.compatible:
            return 750_000_000
        return 500_000_000

    def _sleep_short(self):
        time.sleep(0.001)

    def _set_gpio_alt(self):
        fsel_offset = self.GPFSEL0 + (self.gpio // 10) * 4
        shift = (self.gpio % 10) * 3
        value = self.gpio_regs.read32(fsel_offset)
        value &= ~(0b111 << shift)
        value |= (self.gpio_alt << shift)
        self.gpio_regs.write32(fsel_offset, value)
        self._sleep_short()

    def _stop_pwm_clock(self):
        self.clk_regs.write32(self.CLK_PWMCTL, self.BCM_PASSWORD | self.CLK_CTL_KILL)
        for _ in range(1000):
            if not (self.clk_regs.read32(self.CLK_PWMCTL) & self.CLK_CTL_BUSY):
                return
            time.sleep(0.001)
        raise RuntimeError("等待 PWM clock 停止超时")

    def _start_pwm_clock(self):
        divi = int(self.clock_divider)
        divf = int(round((self.clock_divider - divi) * 4096))
        if divf >= 4096:
            divi += 1
            divf = 0

        self.clk_regs.write32(self.CLK_PWMDIV, self.BCM_PASSWORD | (divi << 12) | divf)
        self.clk_regs.write32(self.CLK_PWMCTL, self.BCM_PASSWORD | self.CLK_CTL_SRC_PLLD)
        self._sleep_short()
        self.clk_regs.write32(
            self.CLK_PWMCTL,
            self.BCM_PASSWORD | self.CLK_CTL_ENAB | self.CLK_CTL_SRC_PLLD,
        )
        for _ in range(1000):
            if self.clk_regs.read32(self.CLK_PWMCTL) & self.CLK_CTL_BUSY:
                return
            time.sleep(0.001)
        raise RuntimeError("等待 PWM clock 启动超时")

    def _channel_ctl_bits(self):
        if self.pwm_channel == 0:
            return self.PWM_CTL_MSEN1 | self.PWM_CTL_PWEN1
        return self.PWM_CTL_MSEN2 | self.PWM_CTL_PWEN2

    def _range_offset(self):
        return self.PWM_RNG1 if self.pwm_channel == 0 else self.PWM_RNG2

    def _data_offset(self):
        return self.PWM_DAT1 if self.pwm_channel == 0 else self.PWM_DAT2

    def _duty_to_data(self, duty):
        duty = clamp_duty(duty)
        return int(round(self.pwm_range * duty / 100.0))

    def _init_pwm_output(self):
        self._set_gpio_alt()

        self.pwm_regs.write32(self.PWM_CTL, 0)
        self._sleep_short()
        self._stop_pwm_clock()
        self._start_pwm_clock()

        self.pwm_regs.write32(self._range_offset(), self.pwm_range)
        self.pwm_regs.write32(self._data_offset(), 0)
        self._sleep_short()
        self.pwm_regs.write32(self.PWM_CTL, self._channel_ctl_bits())

        self.last_output_duty = 0.0
        actual_freq = self.plld_freq / (self.clock_divider * self.pwm_range)
        print(
            f"fan PWM initialized: GPIO{self.gpio}, channel={self.pwm_channel}, "
            f"freq={actual_freq:.2f}Hz, range={self.pwm_range}, divider={self.clock_divider:.4f}"
        )

    def _write_pwm(self, output_duty):
        output_duty = clamp_duty(output_duty)
        data = self._duty_to_data(output_duty)
        self.pwm_regs.write32(self._data_offset(), data)
        if output_duty == self.last_output_duty:
            return output_duty

        self.last_output_duty = output_duty
        print(f"fan output duty set to {output_duty}% ({data}/{self.pwm_range})")
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
        except Exception:
            pass

    def close(self):
        for block in (self.gpio_regs, self.pwm_regs, self.clk_regs):
            if block is not None:
                try:
                    block.close()
                except Exception:
                    pass
        if hasattr(self, "fd"):
            try:
                os.close(self.fd)
            except Exception:
                pass


def read_duty_from_file(path):
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
    pwm_gpio = get_int_config(args.pwm_gpio, "PWM_GPIO", TEMP_CHANNEL)
    pwm_range = get_int_config(args.pwm_range, "PWM_RANGE", PWM_RANGE)

    fan = DirectRegisterThresholdPWMFan(
        gpio=pwm_gpio,
        freq=PWM_FREQ,
        pwm_range=pwm_range,
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
    print("fan pwm gpio:", pwm_gpio)
    print("fan pwm range:", pwm_range)

    def handle_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)

    try:
        while True:
            show_state(netSpeed, fan, CONTROL_FILE)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("stopping...")
    finally:
        fan.cleanup()
        fan.close()
