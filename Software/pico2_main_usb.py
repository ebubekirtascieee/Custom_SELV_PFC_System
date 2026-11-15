# Pico 2 W - 50Hz DFT Metrology

from machine import Pin, SPI
import time
import math
import sys
import uselect

# ==============================================================================
# 1. HARDWARE & METROLOGY CALIBRATION PARAMETERS
# ==============================================================================
VREF_A          = 3.312       # +3V3_A rail voltage (Put calibrated value here)
ADC_MAX         = 4095.0

R_TOP           = 300000.0
R_BOT           = 4700.0
V_GAIN_CAL      = 1.0000
V_OFF_CAL       = 0.0000

R_SHUNT         = 0.100
AMP_GAIN        = 3.300
C24_ATTEN_COMP  = 1.00525
I_GAIN_CAL      = 1.0000 * C24_ATTEN_COMP
AUTO_ZERO_I     = True
I_ZERO_TRIM_V   = -0.0010

PHASE_CAL_DEG   = 0.7         # Locked purely resistive phase calibration
P_PSU_TARE_W    = 0.000
Q_PSU_TARE_VAR  = 0.000

# ==============================================================================
# 2. SAFE BOOT & GPIO INITIALIZATION (CAP0..CAP3 on GP0..GP7)
# ==============================================================================
# Pin mapping:
# CAP0 (4.7uF): EN = GP0, RST = GP1
# CAP1 (10uF) : EN = GP2, RST = GP3
# CAP2 (22uF) : EN = GP4, RST = GP5
# CAP3 (44uF) : EN = GP6, RST = GP7

CAP_EN_PINS  = [
    Pin(0, Pin.OUT, value=0),
    Pin(2, Pin.OUT, value=0),
    Pin(4, Pin.OUT, value=0),
    Pin(6, Pin.OUT, value=0)
]
CAP_RST_PINS = [
    Pin(1, Pin.OUT, value=0),
    Pin(3, Pin.OUT, value=0),
    Pin(5, Pin.OUT, value=0),
    Pin(7, Pin.OUT, value=0)
]
CAP_UF_VALS  = [4.7, 10.0, 22.0, 44.0]

K_V_BUF         = (R_TOP + R_BOT) / R_BOT
K_V_REF         = R_TOP / R_BOT
K_I_SENS        = 1.0 / (R_SHUNT * AMP_GAIN)
LSB_VOLTS       = VREF_A / ADC_MAX

NUM_SAMPLES     = 400
SAMPLE_US       = 500
F_GRID          = 50.0
F_SAMPLE        = 1_000_000.0 / SAMPLE_US

COS_LUT = [math.cos(2.0 * math.pi * F_GRID * n / F_SAMPLE) for n in range(NUM_SAMPLES)]
SIN_LUT = [math.sin(2.0 * math.pi * F_GRID * n / F_SAMPLE) for n in range(NUM_SAMPLES)]

theta_rad = math.radians(PHASE_CAL_DEG)
ROT_COS   = math.cos(theta_rad)
ROT_SIN   = math.sin(theta_rad)

# ==============================================================================
# 3. SPI0 INITIALIZATION
# ==============================================================================
cs = Pin(17, Pin.OUT, value=1)
spi = SPI(0, baudrate=500_000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))

TX_CH0 = bytearray([0x06, 0x00, 0x00])
TX_CH1 = bytearray([0x06, 0x40, 0x00])
TX_CH2 = bytearray([0x06, 0x80, 0x00])
rx_buf = bytearray(3)

raw_ch0 = [0] * NUM_SAMPLES
raw_ch1 = [0] * NUM_SAMPLES
raw_ch2 = [0] * NUM_SAMPLES
dt_us   = [0] * NUM_SAMPLES
v_arr   = [0.0] * NUM_SAMPLES
i_arr   = [0.0] * NUM_SAMPLES

@micropython.native
def capture_burst():
    t_start = time.ticks_us()
    next_t = t_start
    for i in range(NUM_SAMPLES):
        while time.ticks_diff(time.ticks_us(), next_t) < 0:
            pass
        t_now = time.ticks_us()
        dt_us[i] = time.ticks_diff(t_now, t_start)
        next_t = time.ticks_add(next_t, SAMPLE_US)

        cs(0); spi.write_readinto(TX_CH0, rx_buf); cs(1)
        c0 = ((rx_buf[1] & 0x0F) << 8) | rx_buf[2]

        cs(0); spi.write_readinto(TX_CH2, rx_buf); cs(1)
        c2 = ((rx_buf[1] & 0x0F) << 8) | rx_buf[2]

        cs(0); spi.write_readinto(TX_CH1, rx_buf); cs(1)
        c1_a = ((rx_buf[1] & 0x0F) << 8) | rx_buf[2]
        cs(0); spi.write_readinto(TX_CH1, rx_buf); cs(1)
        c1_b = ((rx_buf[1] & 0x0F) << 8) | rx_buf[2]

        raw_ch0[i] = c0
        raw_ch1[i] = (c1_a + c1_b) >> 1
        raw_ch2[i] = c2

def process_and_send_frame():
    capture_burst()
    mean_ch2_v = (sum(raw_ch2) / NUM_SAMPLES) * LSB_VOLTS
    mean_ch1_v = (sum(raw_ch1) / NUM_SAMPLES) * LSB_VOLTS
    i_zero_ref = mean_ch1_v if AUTO_ZERO_I else (mean_ch2_v + I_ZERO_TRIM_V)

    v_re_acc, v_im_acc, i_re_acc, i_im_acc = 0.0, 0.0, 0.0, 0.0

    for n in range(NUM_SAMPLES):
        v_buf = raw_ch0[n] * LSB_VOLTS
        i_buf = raw_ch1[n] * LSB_VOLTS

        v_ac = ((v_buf * K_V_BUF) - (mean_ch2_v * K_V_REF)) * V_GAIN_CAL + V_OFF_CAL
        i_ac = ((i_buf - i_zero_ref) * K_I_SENS) * I_GAIN_CAL

        v_arr[n] = v_ac
        i_arr[n] = i_ac

        v_re_acc += v_ac * COS_LUT[n]; v_im_acc -= v_ac * SIN_LUT[n]
        i_re_acc += i_ac * COS_LUT[n]; i_im_acc -= i_ac * SIN_LUT[n]

    scale = 2.0 / NUM_SAMPLES
    v_re, v_im = v_re_acc * scale, v_im_acc * scale
    i_re_raw, i_im_raw = i_re_acc * scale, i_im_acc * scale

    i_re_cal = (i_re_raw * ROT_COS) - (i_im_raw * ROT_SIN)
    i_im_cal = (i_re_raw * ROT_SIN) + (i_im_raw * ROT_COS)

    v1_rms = math.sqrt(v_re**2 + v_im**2) * 0.70710678
    i1_rms = math.sqrt(i_re_cal**2 + i_im_cal**2) * 0.70710678

    p_shunt = 0.5 * (v_re * i_re_cal + v_im * i_im_cal)
    q_shunt = 0.5 * (v_im * i_re_cal - v_re * i_im_cal)
    s_shunt = math.sqrt(p_shunt**2 + q_shunt**2)
    dpf_shunt = (p_shunt / s_shunt) if s_shunt > 1e-5 else 1.0

    sys.stdout.write("FRAME_START\n")
    sys.stdout.write(f"DFT,{v1_rms:.3f},{i1_rms*1000.0:.2f},{p_shunt:.3f},{q_shunt:.3f},{s_shunt:.3f},{dpf_shunt:.4f}\n")
    for n in range(NUM_SAMPLES):
        sys.stdout.write(f"{dt_us[n]*0.001:.2f},{v_arr[n]:.3f},{i_arr[n]:.5f}\n")
    sys.stdout.write("FRAME_END\n")
    

# Main Loop
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)


while True:
    if poll.poll(10):
        if sys.stdin.readline().strip() == "Q":
            for p in CAP_EN_PINS + CAP_RST_PINS:
                p.value(0)
            break
    process_and_send_frame()
    time.sleep_ms(10)