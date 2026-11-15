# Pico 2 W - Closed-Loop PID PFC Controller (Industrial SCADA HMI Edition)

from machine import Pin, SPI
import network
import socket
import time
import math
import gc

try:
    import errno
    _EAGAIN = errno.EAGAIN
except ImportError:
    _EAGAIN = 11

# ==============================================================================
# 0. OPERATING MODE SELECTOR (SINGLE BOOLEAN SWITCH)
# ==============================================================================
REAL_WORLD_MOTOR_MODE = False

# ==============================================================================
# 1. WI-FI ACCESS POINT CONFIGURATION
# ==============================================================================
AP_SSID     = "Pico2W_PFC_Analyzer"
AP_PASSWORD = "pfc12345" # <-- Change this to a secure password for your Wi-Fi network

# ==============================================================================
# 2. HARDWARE & METROLOGY CALIBRATION PARAMETERS
# ==============================================================================
VREF_A          = 3.312        # Calibrated +3V3_A rail voltage
ADC_MAX         = 4095.0

R_TOP           = 300000.0     # R1 + R4 + R6 (Ohms)
R_BOT           = 4700.0       # R7 (Ohms)
V_GAIN_CAL      = 1.0000
V_OFF_CAL       = 0.0000

R_SHUNT         = 0.100        # R2 100mR Shunt
AMP_GAIN        = 3.300        # R9 / R8 (33k / 10k)
C24_ATTEN_COMP  = 1.00525      # 50Hz RC filter roll-off compensation
I_GAIN_CAL      = 1.0000 * C24_ATTEN_COMP
AUTO_ZERO_I     = True
I_ZERO_TRIM_V   = -0.0010

# Stage 1: Complex Phase Rotation (Locked from pure resistive calibration)
PHASE_CAL_DEG   = 0.7

# Stage 2: Pre-Shunt PSU Tare at J1 (Pico 2 W + Analog Rail consumption before R2)
P_PSU_TARE_W    = 0.750        # Active power consumed by onboard PSU before R2 (W)
Q_PSU_TARE_VAR  = -0.180       # Reactive power of onboard PSU half-wave rectifier (VAr)

# Per-Stage Capacitor Nominal & Effective Values (uF) + Active Loss (W)
CAP_UF_VALS     = (4.7, 10.0, 22.0, 44.0)
CAP_EFF_UF      = (3.80, 9.70, 21.3, 42.7)
CAP_LOSS_W      = (0.04, 0.09, 0.19, 0.39)

MASK_EFF_UF = tuple(
    sum(CAP_EFF_UF[b] for b in range(4) if (m & (1 << b))) for m in range(16)
)
MASK_NOM_UF = tuple(
    sum(CAP_UF_VALS[b] for b in range(4) if (m & (1 << b))) for m in range(16)
)
MAX_BANK_EFF_UF = MASK_EFF_UF[15]

# ==============================================================================
# 3. MODE-DEPENDENT PID & PROTECTION PARAMETERS
# ==============================================================================
if REAL_WORLD_MOTOR_MODE:
    MODE_NAME        = "REAL-WORLD AC MOTOR MODE"
    PID_KP           = 0.50
    PID_KI           = 0.85
    PID_KD           = 0.02
    EMA_ALPHA        = 0.20
    HYST_UF          = 1.30
    SETTLE_FRAMES    = 4
    Q_SETPOINT_VAR   = 0.30
    INRUSH_FREEZE_EN = True
    UNLOAD_DUMP_EN   = True
else:
    MODE_NAME        = "FAST LAB BENCH MODE"
    PID_KP           = 0.65
    PID_KI           = 1.80
    PID_KD           = 0.04
    EMA_ALPHA        = 0.45
    HYST_UF          = 0.95
    SETTLE_FRAMES    = 2
    Q_SETPOINT_VAR   = 0.00
    INRUSH_FREEZE_EN = False
    UNLOAD_DUMP_EN   = False

pid_integral_uf = 0.0
pid_prev_err_uf = 0.0
pid_output_uf   = 0.0
pid_last_ms     = time.ticks_ms()
blank_count     = 0
prev_j2_i_rms   = 0.0

filt_init       = False
q_j1_filt       = 0.0
p_j1_filt       = 0.0
phi_j1_filt     = 0.0

# ==============================================================================
# 4. SAFE BOOT GPIO & ONBOARD TX LED INITIALIZATION
# ==============================================================================
CAP_EN_PINS  = (Pin(0, Pin.OUT, value=0), Pin(2, Pin.OUT, value=0),
                Pin(4, Pin.OUT, value=0), Pin(6, Pin.OUT, value=0))
CAP_RST_PINS = (Pin(1, Pin.OUT, value=0), Pin(3, Pin.OUT, value=0),
                Pin(5, Pin.OUT, value=0), Pin(7, Pin.OUT, value=0))

tx_led = Pin("LED", Pin.OUT, value=0)

# ==============================================================================
# 5. PRE-COMPUTED DFT TABLES & SPI0 SETUP
# ==============================================================================
K_V_BUF         = (R_TOP + R_BOT) / R_BOT
K_V_REF         = R_TOP / R_BOT
K_I_SENS        = 1.0 / (R_SHUNT * AMP_GAIN)
LSB_VOLTS       = VREF_A / ADC_MAX

NUM_SAMPLES     = 400          # 10 cycles of 50Hz @ 2000Hz (200ms window)
SAMPLE_US       = 500
F_GRID          = 50.0
F_SAMPLE        = 1_000_000.0 / SAMPLE_US

COS_LUT = [math.cos(2.0 * math.pi * F_GRID * n / F_SAMPLE) for n in range(NUM_SAMPLES)]
SIN_LUT = [math.sin(2.0 * math.pi * F_GRID * n / F_SAMPLE) for n in range(NUM_SAMPLES)]

theta_rad = math.radians(PHASE_CAL_DEG)
ROT_COS   = math.cos(theta_rad)
ROT_SIN   = math.sin(theta_rad)

cs = Pin(17, Pin.OUT, value=1)
spi = SPI(0, baudrate=500_000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))

TX_CH0 = bytearray([0x06, 0x00, 0x00])  # V_BUF
TX_CH1 = bytearray([0x06, 0x40, 0x00])  # I_BUF
TX_CH2 = bytearray([0x06, 0x80, 0x00])  # +1V65
rx_buf = bytearray(3)

raw_ch0 = [0] * NUM_SAMPLES
raw_ch1 = [0] * NUM_SAMPLES
raw_ch2 = [0] * NUM_SAMPLES

# ==============================================================================
# 6. DETERMINISTIC TRIAC SWITCHING ENGINE & 16-COMBINATION MAPPER
# ==============================================================================
DEADTIME_EN_OFF   = 25
DEADTIME_POST_RST = 15
CAP_DISCHARGE_MS  = (30, 35, 55, 80)

current_mask      = 0
target_mask       = 0
pending_mask      = 0
off_mask          = 0
on_mask           = 0
discharge_ms      = 30
seq_state         = 0          # 0: Idle/Locked, 1: DeadTime(25ms), 2: Discharge, 3: Post-RST(15ms)
seq_timer         = time.ticks_ms()

def uf_to_best_mask(req_uf, cur_m):
    if req_uf <= 0.0:
        clamped_uf = 0.0
    elif req_uf >= MAX_BANK_EFF_UF:
        clamped_uf = MAX_BANK_EFF_UF
    else:
        clamped_uf = req_uf

    best_m = 0
    best_diff = 9999.0
    for m in range(16):
        d = abs(MASK_EFF_UF[m] - clamped_uf)
        if d < best_diff:
            best_diff = d
            best_m = m

    if best_m != cur_m:
        cur_diff = abs(MASK_EFF_UF[cur_m] - clamped_uf)
        if (cur_diff - best_diff) < HYST_UF:
            return cur_m

    return best_m

def request_cap_mask(new_mask):
    global pending_mask
    pending_mask = new_mask & 0x0F

@micropython.native
def update_cap_state_machine():
    global current_mask, target_mask, off_mask, on_mask, discharge_ms, seq_state, seq_timer
    now = time.ticks_ms()

    if seq_state == 0:
        if pending_mask != current_mask:
            target_mask = pending_mask
            off_mask    = current_mask & (~target_mask)
            on_mask     = target_mask & (~current_mask)

            if off_mask:
                max_dis_ms = 30
                for b in range(4):
                    if off_mask & (1 << b):
                        CAP_EN_PINS[b].value(0)
                        if CAP_DISCHARGE_MS[b] > max_dis_ms:
                            max_dis_ms = CAP_DISCHARGE_MS[b]
                discharge_ms = max_dis_ms
                seq_state = 1
                seq_timer = now
            else:
                for b in range(4):
                    if on_mask & (1 << b):
                        CAP_RST_PINS[b].value(0)
                seq_state = 3
                seq_timer = now

    elif seq_state == 1:
        if time.ticks_diff(now, seq_timer) >= DEADTIME_EN_OFF:
            for b in range(4):
                if off_mask & (1 << b):
                    CAP_RST_PINS[b].value(1)
            seq_state = 2
            seq_timer = now

    elif seq_state == 2:
        if time.ticks_diff(now, seq_timer) >= discharge_ms:
            for b in range(4):
                if off_mask & (1 << b):
                    CAP_RST_PINS[b].value(0)
            seq_state = 3
            seq_timer = now

    elif seq_state == 3:
        if time.ticks_diff(now, seq_timer) >= DEADTIME_POST_RST:
            if on_mask:
                for b in range(4):
                    if on_mask & (1 << b):
                        CAP_RST_PINS[b].value(0)
                        CAP_EN_PINS[b].value(1)
            current_mask = target_mask
            seq_state = 0
            seq_timer = now

# ==============================================================================
# 7. FAST NATIVE BURST SAMPLER, 50Hz DFT + WIDEBAND ENGINE & PID CONTROLLER
# ==============================================================================
fast_http_payload = (
    b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
    b"Content-Length: 2\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n[]"
)

@micropython.native
def capture_burst():
    t_start = time.ticks_us()
    next_t = t_start
    for i in range(NUM_SAMPLES):
        while time.ticks_diff(time.ticks_us(), next_t) < 0:
            pass
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

        if seq_state != 0 or (i & 7) == 0:
            update_cap_state_machine()

@micropython.native
def dft_accumulate():
    sum_c0 = 0
    sum_c1 = 0
    sum_c0_sq = 0
    sum_c1_sq = 0
    sum_c0_c1 = 0
    v_re_raw = 0.0
    v_im_raw = 0.0
    i_re_raw = 0.0
    i_im_raw = 0.0

    for n in range(NUM_SAMPLES):
        c0 = raw_ch0[n]
        c1 = raw_ch1[n]
        sum_c0 += c0
        sum_c1 += c1
        sum_c0_sq += c0 * c0
        sum_c1_sq += c1 * c1
        sum_c0_c1 += c0 * c1

        c_val = COS_LUT[n]
        s_val = SIN_LUT[n]
        v_re_raw += c0 * c_val
        v_im_raw -= c0 * s_val
        i_re_raw += c1 * c_val
        i_im_raw -= c1 * s_val

    return sum_c0, sum_c1, sum_c0_sq, sum_c1_sq, sum_c0_c1, v_re_raw, v_im_raw, i_re_raw, i_im_raw

def compute_metrology_and_pid():
    global fast_http_payload, filt_init, q_j1_filt, p_j1_filt, phi_j1_filt
    global pid_integral_uf, pid_prev_err_uf, pid_output_uf, pid_last_ms, blank_count, prev_j2_i_rms

    mask_before = current_mask
    state_before = seq_state

    capture_burst()
    update_cap_state_machine()
    (sum_c0, sum_c1, sum_c0_sq, sum_c1_sq, sum_c0_c1,
     v_re_raw, v_im_raw, i_re_raw, i_im_raw) = dft_accumulate()
    update_cap_state_machine()

    snap_mask = current_mask
    switched_during_frame = (snap_mask != mask_before) or (state_before != 0) or (seq_state != 0)

    inv_n = 1.0 / NUM_SAMPLES
    v_lsb = LSB_VOLTS * K_V_BUF * V_GAIN_CAL
    i_lsb = LSB_VOLTS * K_I_SENS * I_GAIN_CAL

    v_scale = (2.0 * inv_n) * v_lsb
    i_scale = (2.0 * inv_n) * i_lsb

    v_re = v_re_raw * v_scale
    v_im = v_im_raw * v_scale
    i_re_u = i_re_raw * i_scale
    i_im_u = i_im_raw * i_scale

    # Stage 1: Complex Phase Rotation e^(j * PHASE_CAL_DEG)
    i_re_cal = (i_re_u * ROT_COS) - (i_im_u * ROT_SIN)
    i_im_cal = (i_re_u * ROT_SIN) + (i_im_u * ROT_COS)

    v1_rms = math.sqrt(v_re * v_re + v_im * v_im) * 0.70710678
    i1_shunt_rms = math.sqrt(i_re_cal * i_re_cal + i_im_cal * i_im_cal) * 0.70710678

    p_shunt = 0.5 * (v_re * i_re_cal + v_im * i_im_cal)
    q_shunt = 0.5 * (v_im * i_re_cal - v_re * i_im_cal)

    # Wideband (0..1000Hz) Time-Domain True-RMS Variance Calculation
    mean_c0 = sum_c0 * inv_n
    mean_c1 = sum_c1 * inv_n
    var_c0 = max(0.0, (sum_c0_sq * inv_n) - (mean_c0 * mean_c0))
    var_c1 = max(0.0, (sum_c1_sq * inv_n) - (mean_c1 * mean_c1))
    cov_c0_c1 = (sum_c0_c1 * inv_n) - (mean_c0 * mean_c1)

    v_wb_rms = max(v1_rms, math.sqrt(var_c0) * v_lsb)
    i_shunt_wb_rms = math.sqrt(var_c1) * i_lsb
    i_harm_sq = max(0.0, (i_shunt_wb_rms * i_shunt_wb_rms) - (i1_shunt_rms * i1_shunt_rms))
    p_shunt_raw_50 = 0.5 * (v_re * i_re_u + v_im * i_im_u)
    p_harm = (cov_c0_c1 * v_lsb * i_lsb) - p_shunt_raw_50

    # --------------------------------------------------------------------------
    # BAY 01: J2 CONNECTOR FUNDAMENTAL (50Hz Isolated External Load)
    # --------------------------------------------------------------------------
    active_eff_uf = MASK_EFF_UF[snap_mask]
    nom_uf        = MASK_NOM_UF[snap_mask]
    p_cap_est     = 0.0
    for b in range(4):
        if snap_mask & (1 << b):
            p_cap_est += CAP_LOSS_W[b]

    v_sq = v1_rms * v1_rms
    omega_v_sq_uf = 2.0 * math.pi * F_GRID * v_sq * 1e-6
    q_cap_est = -(omega_v_sq_uf * active_eff_uf)

    p_j2 = max(0.0, p_shunt - p_cap_est) if snap_mask != 0 else p_shunt
    q_j2 = (q_shunt - q_cap_est) if snap_mask != 0 else q_shunt
    s_j2 = math.sqrt(p_j2 * p_j2 + q_j2 * q_j2)
    i1_j2_rms = (s_j2 / v1_rms) if v1_rms > 1.0 else 0.0
    dpf_j2 = (p_j2 / s_j2) if s_j2 > 1e-4 else 1.0
    phi_j2 = math.degrees(math.atan2(q_j2, p_j2)) if s_j2 > 1e-4 else 0.0

    # --------------------------------------------------------------------------
    # BAY 02: J1 CONNECTOR FUNDAMENTAL (50Hz Total Input + PID Feedback Loop)
    # --------------------------------------------------------------------------
    p_j1_raw = p_shunt + P_PSU_TARE_W
    q_j1_raw = q_shunt + Q_PSU_TARE_VAR
    s_j1_raw = math.sqrt(p_j1_raw * p_j1_raw + q_j1_raw * q_j1_raw)
    phi_j1_raw = math.degrees(math.atan2(q_j1_raw, p_j1_raw)) if s_j1_raw > 1e-4 else 0.0

    now_ms = time.ticks_ms()
    dt_s = time.ticks_diff(now_ms, pid_last_ms) * 0.001
    if dt_s <= 0.05 or dt_s > 2.0:
        dt_s = 0.22
    pid_last_ms = now_ms

    inrush_detected = False
    if INRUSH_FREEZE_EN:
        if i1_j2_rms > 3.50 or (prev_j2_i_rms > 0.15 and i1_j2_rms > (prev_j2_i_rms * 2.2)):
            inrush_detected = True
    prev_j2_i_rms = i1_j2_rms

    if v1_rms < 5.0:
        pid_integral_uf = 0.0
        pid_prev_err_uf = 0.0
        pid_output_uf   = 0.0
        filt_init       = False
        q_j1_filt       = q_j1_raw
        p_j1_filt       = p_j1_raw
        phi_j1_filt     = 0.0
        request_cap_mask(0)

    elif UNLOAD_DUMP_EN and snap_mask != 0 and p_j2 < 0.25 and s_j2 < 0.40:
        pid_integral_uf = 0.0
        pid_prev_err_uf = 0.0
        pid_output_uf   = 0.0
        request_cap_mask(0)
        blank_count     = SETTLE_FRAMES

    elif inrush_detected:
        blank_count = 5

    elif switched_during_frame:
        blank_count = SETTLE_FRAMES

    elif blank_count > 0:
        blank_count -= 1
        if blank_count == 0:
            q_j1_filt   = q_j1_raw
            p_j1_filt   = p_j1_raw
            phi_j1_filt = phi_j1_raw

    else:
        if not filt_init:
            q_j1_filt   = q_j1_raw
            p_j1_filt   = p_j1_raw
            phi_j1_filt = phi_j1_raw
            filt_init   = True
        else:
            q_j1_filt   = EMA_ALPHA * q_j1_raw   + (1.0 - EMA_ALPHA) * q_j1_filt
            p_j1_filt   = EMA_ALPHA * p_j1_raw   + (1.0 - EMA_ALPHA) * p_j1_filt
            phi_j1_filt = EMA_ALPHA * phi_j1_raw + (1.0 - EMA_ALPHA) * phi_j1_filt

        q_err_var = q_j1_filt - Q_SETPOINT_VAR
        err_uf = (q_err_var / omega_v_sq_uf) if omega_v_sq_uf > 1e-4 else 0.0

        p_term = PID_KP * err_uf
        pid_integral_uf += (PID_KI * err_uf * dt_s)
        if pid_integral_uf < 0.0:
            pid_integral_uf = 0.0
        elif pid_integral_uf > MAX_BANK_EFF_UF:
            pid_integral_uf = MAX_BANK_EFF_UF

        d_term = PID_KD * ((err_uf - pid_prev_err_uf) / dt_s)
        pid_prev_err_uf = err_uf

        raw_out_uf = p_term + pid_integral_uf + d_term
        if raw_out_uf < 0.0:
            pid_output_uf = 0.0
        elif raw_out_uf > MAX_BANK_EFF_UF:
            pid_output_uf = MAX_BANK_EFF_UF
        else:
            pid_output_uf = raw_out_uf

        best_mask = uf_to_best_mask(pid_output_uf, snap_mask)
        if best_mask != snap_mask:
            request_cap_mask(best_mask)
            pid_integral_uf = MASK_EFF_UF[best_mask]
            pid_prev_err_uf = 0.0
            blank_count     = SETTLE_FRAMES

    s_j1 = math.sqrt(p_j1_filt * p_j1_filt + q_j1_filt * q_j1_filt)
    i1_j1_rms = (s_j1 / v1_rms) if v1_rms > 1.0 else 0.0
    dpf_j1 = (p_j1_filt / s_j1) if s_j1 > 1e-4 else 1.0

    # --------------------------------------------------------------------------
    # BAY 03: J1 CONNECTOR WIDEBAND (True-RMS 0..1000Hz Scope/DMM Equivalent)
    # --------------------------------------------------------------------------
    i_j1_wb_rms = math.sqrt(i1_j1_rms * i1_j1_rms + i_harm_sq)
    p_j1_wb     = max(0.0, p_j1_filt + p_harm)
    s_j1_wb     = v_wb_rms * i_j1_wb_rms
    pf_j1_wb    = min(1.0, p_j1_wb / s_j1_wb) if s_j1_wb > 1e-4 else 1.0
    thd_i_pct   = (math.sqrt(i_harm_sq) / i1_j1_rms * 100.0) if i1_j1_rms > 0.005 else 0.0

    # Compact 27-element telemetry array:
    # 0..4:   v1_rms, snap_mask, nom_uf, q_cap_est, pid_output_uf
    # 5..8:   EN0, EN1, EN2, EN3
    # 9..14:  J2 Fundamental 50Hz (i_mA, dpf, p, q, s, phi)
    # 15..20: J1 Fundamental 50Hz (i_mA, dpf, p, q, s, phi)
    # 21..26: J1 Wideband True-RMS (v_wb, i_wb_mA, pf_wb, p_wb, s_wb, thd_i_pct)
    
    body = (
        f"[{v1_rms:.2f},{snap_mask},{nom_uf:.1f},{q_cap_est:.2f},{pid_output_uf:.1f},"
        f"{CAP_EN_PINS[0].value()},{CAP_EN_PINS[1].value()},{CAP_EN_PINS[2].value()},{CAP_EN_PINS[3].value()},"
        f"{i1_j2_rms*1000.0:.1f},{dpf_j2:.3f},{p_j2:.2f},{q_j2:.2f},{s_j2:.2f},{phi_j2:.1f},"
        f"{i1_j1_rms*1000.0:.1f},{dpf_j1:.3f},{p_j1_filt:.2f},{q_j1_filt:.2f},{s_j1:.2f},{phi_j1_filt:.1f},"
        f"{v_wb_rms:.2f},{i_j1_wb_rms*1000.0:.1f},{pf_j1_wb:.3f},{p_j1_wb:.2f},{s_j1_wb:.2f},{thd_i_pct:.1f}]"
    )
    fast_http_payload = (
        f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
        f"Content-Length: {len(body)}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n{body}"
    ).encode("utf-8")

# ==============================================================================
# 8. INDUSTRIAL SCADA / SUBSTATION RTU HMI (ISA-101 / WINCC STANDARD)
# ==============================================================================
_HTML_BODY = ("""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>REACTIVE POWER COMPENSATION & METROLOGY RTU</title>
<style>
  :root {
    --bg: #12161F;
    --panel: #1A202C;
    --hdr: #222B3A;
    --cell: #151A24;
    --border: #2D3748;
    --border-hi: #3E4C63;
    --text: #E2E8F0;
    --muted: #94A3B8;
    --accent: #0EA5E9;
    --accent-hi: #38BDF8;
    --green: #10B981;
    --green-bg: rgba(16, 185, 129, 0.14);
    --amber: #F59E0B;
    --amber-bg: rgba(245, 158, 11, 0.14);
    --blue-bg: rgba(14, 165, 233, 0.14);
    --mono: Consolas, 'JetBrains Mono', 'Courier New', monospace;
    --sans: 'Segoe UI', -apple-system, BlinkMacSystemFont, Roboto, sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; border-radius: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: var(--sans);
    font-size: 12px;
    padding: 14px;
    max-width: 1380px;
    margin: 0 auto;
    line-height: 1.4;
  }
  .scada-frame {
    border: 1px solid var(--border);
    background: var(--panel);
    margin-bottom: 12px;
  }
  .scada-topbar {
    display: flex;
    flex-wrap: wrap;
    justify-content: space-between;
    align-items: center;
    background: var(--hdr);
    border-bottom: 2px solid var(--accent);
    padding: 10px 14px;
    gap: 12px;
  }
  .title-block h1 {
    font-size: 14px;
    font-weight: 700;
    letter-spacing: 0.7px;
    color: var(--text);
    text-transform: uppercase;
  }
  .title-sub {
    font-family: var(--mono);
    font-size: 11px;
    color: var(--muted);
    margin-top: 2px;
  }
  .telemetry-bar {
    display: flex;
    align-items: center;
    gap: 10px;
    font-family: var(--mono);
    font-size: 11px;
  }
  .status-box {
    display: inline-flex;
    align-items: center;
    gap: 7px;
    padding: 4px 10px;
    border: 1px solid var(--border-hi);
    background: var(--cell);
    color: var(--muted);
    font-weight: 600;
  }
  .status-box strong { color: var(--text); }
  .lamp-rect {
    width: 12px;
    height: 8px;
    background: #2D3748;
    border: 1px solid #4A5568;
    display: inline-block;
  }
  .lamp-active {
    background: var(--green);
    border-color: #34D399;
  }
  .lamp-warn {
    background: var(--amber);
    border-color: #FBBF24;
  }

  .kpi-strip {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    background: var(--cell);
  }
  @media (max-width: 860px) {
    .kpi-strip { grid-template-columns: 1fr; }
  }
  .kpi-block {
    padding: 12px 16px;
    border-right: 1px solid var(--border);
    display: flex;
    justify-content: space-between;
    align-items: baseline;
  }
  .kpi-block:last-child { border-right: none; }
  .kpi-lbl {
    font-size: 11px;
    font-weight: 700;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }
  .kpi-val {
    font-family: var(--mono);
    font-size: 17px;
    font-weight: 700;
    color: var(--text);
    font-variant-numeric: tabular-nums;
  }
  .kpi-sep {
    color: var(--border-hi);
    margin: 0 6px;
    font-weight: 400;
  }
  .kpi-unit {
    font-family: var(--mono);
    font-size: 12px;
    color: var(--muted);
    margin-left: 4px;
    font-weight: 600;
  }

  .bays-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 12px;
    margin-bottom: 12px;
  }
  @media (max-width: 1080px) {
    .bays-grid { grid-template-columns: 1fr; }
  }
  .bay-panel {
    background: var(--panel);
    border: 1px solid var(--border);
    display: flex;
    flex-direction: column;
  }
  .bay-hdr {
    background: var(--hdr);
    border-bottom: 1px solid var(--border);
    padding: 8px 12px;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .bay-title {
    font-weight: 700;
    font-size: 11px;
    letter-spacing: 0.6px;
    text-transform: uppercase;
    color: var(--text);
  }
  .state-badge {
    padding: 2px 8px;
    font-family: var(--mono);
    font-size: 10px;
    font-weight: 700;
    border: 1px solid var(--border-hi);
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }
  .st-unity { background: var(--green-bg); color: var(--green); border-color: var(--green); }
  .st-ind   { background: var(--amber-bg); color: var(--amber); border-color: var(--amber); }
  .st-cap   { background: var(--blue-bg);  color: var(--accent-hi); border-color: var(--accent); }
  .st-wb    { background: var(--cell);     color: var(--muted); border-color: var(--border-hi); }

  table.scada-table {
    width: 100%;
    border-collapse: collapse;
    font-variant-numeric: tabular-nums;
  }
  table.scada-table thead th {
    background: var(--cell);
    color: var(--muted);
    font-size: 10px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    padding: 6px 12px;
    border-bottom: 1px solid var(--border);
    text-align: left;
  }
  table.scada-table thead th.num-col { text-align: right; }
  table.scada-table tbody tr { border-bottom: 1px solid #232B3A; }
  table.scada-table tbody tr:last-child { border-bottom: none; }
  table.scada-table td { padding: 8px 12px; }
  .td-param { color: var(--text); font-size: 12px; width: 46%; }
  .td-sym   { font-family: var(--mono); color: var(--muted); font-size: 11px; width: 18%; }
  .td-val   { font-family: var(--mono); font-size: 14px; font-weight: 700; color: var(--text); text-align: right; width: 24%; }
  .td-unit  { font-family: var(--mono); color: var(--muted); font-size: 11px; width: 12%; padding-left: 6px; }

  .matrix-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 10px;
    padding: 12px;
    background: var(--cell);
  }
  @media (max-width: 900px) {
    .matrix-grid { grid-template-columns: repeat(2, 1fr); }
  }
  .stage-unit {
    border: 1px solid var(--border);
    background: var(--panel);
    padding: 10px 12px;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .stage-name {
    font-family: var(--mono);
    font-size: 12px;
    font-weight: 700;
    color: var(--text);
  }
  .stage-sub {
    font-family: var(--mono);
    font-size: 10px;
    color: var(--muted);
    margin-top: 2px;
  }
</style>
</head>
<body>

<div class="scada-frame">
  <div class="scada-topbar">
    <div class="title-block">
      <h1>REACTIVE POWER COMPENSATION &amp; METROLOGY RTU</h1>
      <div class="title-sub">50 Hz FUNDAMENTAL DFT &nbsp;|&nbsp; CLOSED-LOOP PID CONTROLLER</div>
    </div>
    <div class="telemetry-bar">
      <div class="status-box">
        <span>SYSTEM MODE:</span>
        <strong>""" + MODE_NAME + """</strong>
      </div>
      <div class="status-box">
        <span>POLL RATE:</span>
        <strong id="fps_txt">-- Hz</strong>
      </div>
      <div class="status-box">
        <span id="tx_dot" class="lamp-rect"></span>
        <span>COMM LINK</span>
      </div>
    </div>
  </div>

  <div class="kpi-strip">
    <div class="kpi-block">
      <span class="kpi-lbl">BUS VOLTAGE (50 Hz V1)</span>
      <div><span class="kpi-val" id="v1_rms">--</span><span class="kpi-unit">V</span></div>
    </div>
    <div class="kpi-block">
      <span class="kpi-lbl">COMPENSATION BANK STATE</span>
      <div>
        <span class="kpi-val" id="step_bits" style="color:var(--accent-hi)">0000</span>
        <span class="kpi-sep">|</span>
        <span class="kpi-val" id="nom_uf_txt">0.0</span><span class="kpi-unit">&micro;F</span>
      </div>
    </div>
    <div class="kpi-block">
      <span class="kpi-lbl">CONTROLLER DEMAND / Q_BANK</span>
      <div>
        <span class="kpi-val" id="seq_timer_txt">0.0</span><span class="kpi-unit">&micro;F</span>
        <span class="kpi-sep">|</span>
        <span class="kpi-val" id="q_cap_est">0.00</span><span class="kpi-unit">VAr</span>
      </div>
    </div>
  </div>
</div>

<div class="bays-grid">
  <div class="bay-panel">
    <div class="bay-hdr">
      <span class="bay-title">BAY 01 &mdash; J2 LOAD (50 Hz FUNDAMENTAL)</span>
      <span id="j2_tag" class="state-badge st-unity">UNITY</span>
    </div>
    <table class="scada-table">
      <thead>
        <tr><th>Parameter</th><th>Sym</th><th class="num-col">Value</th><th>Unit</th></tr>
      </thead>
      <tbody>
        <tr><td class="td-param">Load Current</td><td class="td-sym">I1_J2</td><td class="td-val" id="j2_i">--</td><td class="td-unit">mA</td></tr>
        <tr><td class="td-param">Displacement PF</td><td class="td-sym">DPF_J2</td><td class="td-val" id="j2_dpf">--</td><td class="td-unit">pu</td></tr>
        <tr><td class="td-param">Active Power</td><td class="td-sym">P1_J2</td><td class="td-val" id="j2_p">--</td><td class="td-unit">W</td></tr>
        <tr><td class="td-param">Reactive Power</td><td class="td-sym">Q1_J2</td><td class="td-val" id="j2_q">--</td><td class="td-unit">VAr</td></tr>
        <tr><td class="td-param">Apparent Power</td><td class="td-sym">S1_J2</td><td class="td-val" id="j2_s">--</td><td class="td-unit">VA</td></tr>
        <tr><td class="td-param">Phase Angle</td><td class="td-sym">PHI_J2</td><td class="td-val" id="j2_phi">--</td><td class="td-unit">deg</td></tr>
      </tbody>
    </table>
  </div>

  <div class="bay-panel">
    <div class="bay-hdr">
      <span class="bay-title">BAY 02 &mdash; J1 INPUT (50 Hz FUNDAMENTAL)</span>
      <span id="j1_tag" class="state-badge st-unity">UNITY</span>
    </div>
    <table class="scada-table">
      <thead>
        <tr><th>Parameter</th><th>Sym</th><th class="num-col">Value</th><th>Unit</th></tr>
      </thead>
      <tbody>
        <tr><td class="td-param">Input Current</td><td class="td-sym">I1_J1</td><td class="td-val" id="j1_i">--</td><td class="td-unit">mA</td></tr>
        <tr><td class="td-param">Displacement PF</td><td class="td-sym">DPF_J1</td><td class="td-val" id="j1_dpf" style="color:var(--green)">--</td><td class="td-unit">pu</td></tr>
        <tr><td class="td-param">Active Power</td><td class="td-sym">P1_J1</td><td class="td-val" id="j1_p">--</td><td class="td-unit">W</td></tr>
        <tr><td class="td-param">Reactive Power</td><td class="td-sym">Q1_J1</td><td class="td-val" id="j1_q">--</td><td class="td-unit">VAr</td></tr>
        <tr><td class="td-param">Apparent Power</td><td class="td-sym">S1_J1</td><td class="td-val" id="j1_s">--</td><td class="td-unit">VA</td></tr>
        <tr><td class="td-param">Phase Angle</td><td class="td-sym">PHI_J1</td><td class="td-val" id="j1_phi" style="color:var(--accent-hi)">--</td><td class="td-unit">deg</td></tr>
      </tbody>
    </table>
  </div>

  <div class="bay-panel">
    <div class="bay-hdr">
      <span class="bay-title">BAY 03 &mdash; J1 INPUT (0&ndash;1 kHz TRUE-RMS)</span>
      <span class="state-badge st-wb">WIDEBAND RMS</span>
    </div>
    <table class="scada-table">
      <thead>
        <tr><th>Parameter</th><th>Sym</th><th class="num-col">Value</th><th>Unit</th></tr>
      </thead>
      <tbody>
        <tr><td class="td-param">Wideband Current</td><td class="td-sym">I_RMS</td><td class="td-val" id="wb_i">--</td><td class="td-unit">mA</td></tr>
        <tr><td class="td-param">Total Power Factor</td><td class="td-sym">PF_TOT</td><td class="td-val" id="wb_pf">--</td><td class="td-unit">pu</td></tr>
        <tr><td class="td-param">Active Power</td><td class="td-sym">P_WB</td><td class="td-val" id="wb_p">--</td><td class="td-unit">W</td></tr>
        <tr><td class="td-param">Apparent Power</td><td class="td-sym">S_WB</td><td class="td-val" id="wb_s">--</td><td class="td-unit">VA</td></tr>
        <tr><td class="td-param">Wideband Voltage</td><td class="td-sym">V_RMS</td><td class="td-val" id="wb_v">--</td><td class="td-unit">V</td></tr>
        <tr><td class="td-param">Current THD</td><td class="td-sym">THD_I</td><td class="td-val" id="wb_thd">--</td><td class="td-unit">%</td></tr>
      </tbody>
    </table>
  </div>
</div>

<div class="scada-frame">
  <div class="bay-hdr">
    <span class="bay-title">CAPACITOR BANK SWITCHING MATRIX</span>
    <span class="title-sub">4-STAGE BINARY STEPPED COMPENSATION</span>
  </div>
  <div class="matrix-grid">
    <div class="stage-unit">
      <div>
        <div class="stage-name">STAGE 1 (4.7 &micro;F)</div>
        <div class="stage-sub">STEP WEIGHT: 1</div>
      </div>
      <div class="status-box"><span id="en_0" class="lamp-rect"></span><strong id="st_0">STANDBY</strong></div>
    </div>
    <div class="stage-unit">
      <div>
        <div class="stage-name">STAGE 2 (10.0 &micro;F)</div>
        <div class="stage-sub">STEP WEIGHT: 2</div>
      </div>
      <div class="status-box"><span id="en_1" class="lamp-rect"></span><strong id="st_1">STANDBY</strong></div>
    </div>
    <div class="stage-unit">
      <div>
        <div class="stage-name">STAGE 3 (22.0 &micro;F)</div>
        <div class="stage-sub">STEP WEIGHT: 4</div>
      </div>
      <div class="status-box"><span id="en_2" class="lamp-rect"></span><strong id="st_2">STANDBY</strong></div>
    </div>
    <div class="stage-unit">
      <div>
        <div class="stage-name">STAGE 4 (44.0 &micro;F)</div>
        <div class="stage-sub">STEP WEIGHT: 8</div>
      </div>
      <div class="status-box"><span id="en_3" class="lamp-rect"></span><strong id="st_3">STANDBY</strong></div>
    </div>
  </div>
</div>

<script>
let lastT = performance.now();
function setTag(el, q) {
  if (q > 0.04) { el.textContent = "INDUCTIVE"; el.className = "state-badge st-ind"; }
  else if (q < -0.04) { el.textContent = "CAPACITIVE"; el.className = "state-badge st-cap"; }
  else { el.textContent = "UNITY"; el.className = "state-badge st-unity"; }
}
async function pollLoop() {
  const txDot = document.getElementById('tx_dot');
  const fpsTxt = document.getElementById('fps_txt');
  const ac = new AbortController();
  const tid = setTimeout(() => ac.abort(), 2000);
  try {
    txDot.className = 'lamp-rect lamp-active';
    const r = await fetch('/d', {cache: 'no-store', signal: ac.signal});
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const a = await r.json();
    clearTimeout(tid);
    setTimeout(() => { txDot.className = 'lamp-rect'; }, 45);

    const now = performance.now();
    const hz = 1000.0 / Math.max(1, now - lastT);
    lastT = now;
    fpsTxt.textContent = hz.toFixed(1) + ' Hz';

    const bits = (a[1] >>> 0).toString(2).padStart(4, '0');
    document.getElementById('v1_rms').textContent = a[0].toFixed(2);
    document.getElementById('step_bits').textContent = 'BINARY [' + bits + ']';
    document.getElementById('nom_uf_txt').textContent = a[2].toFixed(1);
    document.getElementById('q_cap_est').textContent = a[3].toFixed(2);
    document.getElementById('seq_timer_txt').textContent = a[4].toFixed(1);

    for (let i = 0; i < 4; i++) {
      const on = a[5 + i] === 1;
      document.getElementById('en_' + i).className = 'lamp-rect ' + (on ? 'lamp-active' : '');
      const st = document.getElementById('st_' + i);
      st.textContent = on ? 'ENGAGED' : 'STANDBY';
      st.style.color = on ? '#10B981' : '#94A3B8';
    }

    document.getElementById('j2_i').textContent = a[9].toFixed(1);
    document.getElementById('j2_dpf').textContent = a[10].toFixed(3);
    document.getElementById('j2_p').textContent = a[11].toFixed(2);
    document.getElementById('j2_q').textContent = (a[12] >= 0 ? '+' : '') + a[12].toFixed(2);
    document.getElementById('j2_s').textContent = a[13].toFixed(2);
    document.getElementById('j2_phi').textContent = (a[14] >= 0 ? '+' : '') + a[14].toFixed(1);
    setTag(document.getElementById('j2_tag'), a[12]);

    document.getElementById('j1_i').textContent = a[15].toFixed(1);
    document.getElementById('j1_dpf').textContent = a[16].toFixed(3);
    document.getElementById('j1_p').textContent = a[17].toFixed(2);
    document.getElementById('j1_q').textContent = (a[18] >= 0 ? '+' : '') + a[18].toFixed(2);
    document.getElementById('j1_s').textContent = a[19].toFixed(2);
    document.getElementById('j1_phi').textContent = (a[20] >= 0 ? '+' : '') + a[20].toFixed(1);
    setTag(document.getElementById('j1_tag'), a[18]);

    document.getElementById('wb_v').textContent = a[21].toFixed(2);
    document.getElementById('wb_i').textContent = a[22].toFixed(1);
    document.getElementById('wb_pf').textContent = a[23].toFixed(3);
    document.getElementById('wb_p').textContent = a[24].toFixed(2);
    document.getElementById('wb_s').textContent = a[25].toFixed(2);
    document.getElementById('wb_thd').textContent = a[26].toFixed(1);
  } catch (e) {
    clearTimeout(tid);
    txDot.className = 'lamp-rect lamp-warn';
    fpsTxt.textContent = 'WAIT';
  }
  setTimeout(pollLoop, 100);
}
pollLoop();
</script>
</body>
</html>""").encode("utf-8")

HTML_RESPONSE = (
    f"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
    f"Content-Length: {len(_HTML_BODY)}\r\nConnection: close\r\n\r\n"
).encode("utf-8") + _HTML_BODY

RESP_204 = b"HTTP/1.1 204 No Content\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"

# ==============================================================================
# 9. WI-FI AP & NON-BLOCKING SOCKET ENGINE
# ==============================================================================
ap = network.WLAN(network.AP_IF)
ap.config(essid=AP_SSID, password=AP_PASSWORD)
ap.active(True)
while not ap.active():
    time.sleep_ms(50)

try:
    ap.ifconfig(("192.168.4.1", "255.255.255.0", "192.168.4.1", "192.168.4.1"))
    ap.config(pm=0xA11140)
except Exception:
    pass

print("AP Ready (" + MODE_NAME + ") -> Open http://" + ap.ifconfig()[0])

addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(addr)
srv.listen(8)
srv.setblocking(False)

clients = []
MAX_CLIENTS  = 8
TX_STROBE_MS = 35
tx_led_off_at = None

def _close_client(c):
    try:
        c[0].close()
    except Exception:
        pass

def _accept_new():
    while True:
        try:
            cl, _ = srv.accept()
        except OSError:
            break
        if len(clients) >= MAX_CLIENTS:
            evicted = False
            for idx, c in enumerate(clients):
                if c[1] == 0:
                    _close_client(c)
                    clients.pop(idx)
                    evicted = True
                    break
            if not evicted:
                _close_client(clients.pop(0))
        cl.setblocking(False)
        clients.append([cl, 0, bytearray(), None, 0, time.ticks_add(time.ticks_ms(), 1500)])

def _pump_clients():
    global tx_led_off_at
    now = time.ticks_ms()
    i = 0
    while i < len(clients):
        update_cap_state_machine()
        c = clients[i]
        sock = c[0]

        if time.ticks_diff(now, c[5]) >= 0:
            _close_client(c)
            clients.pop(i)
            continue

        if c[1] == 0:
            closed = False
            while True:
                try:
                    chunk = sock.recv(512)
                    if chunk == b"":
                        closed = True
                        break
                    if chunk is None:
                        break
                    if len(c[2]) < 2048:
                        c[2].extend(chunk)
                except OSError as e:
                    if e.args and e.args[0] == _EAGAIN:
                        break
                    closed = True
                    break

            if closed and len(c[2]) == 0:
                _close_client(c)
                clients.pop(i)
                continue

            if b"\r\n\r\n" in c[2] or (closed and len(c[2]) > 0) or len(c[2]) >= 2048:
                raw = bytes(c[2])
                eol = raw.find(b"\r\n")
                line = raw[:eol] if eol > 0 else raw
                parts = line.split(b" ")
                path = parts[1] if len(parts) >= 2 else b"/"

                if path == b"/d" or path.startswith(b"/d?"):
                    resp = fast_http_payload
                    tx_led.value(1)
                    tx_led_off_at = time.ticks_add(time.ticks_ms(), TX_STROBE_MS)
                elif path == b"/" or path == b"/index.html" or path.startswith(b"/?"):
                    resp = HTML_RESPONSE
                    tx_led.value(1)
                    tx_led_off_at = time.ticks_add(time.ticks_ms(), TX_STROBE_MS)
                else:
                    resp = RESP_204

                c[1] = 1
                c[2] = None
                c[3] = memoryview(resp)
                c[4] = 0
                c[5] = time.ticks_add(time.ticks_ms(), 2500)
            else:
                i += 1
                continue

        if c[1] == 1:
            mv = c[3]
            off = c[4]
            total = len(mv)
            drop = False
            while off < total:
                try:
                    sent = sock.send(mv[off:])
                    if sent is None or sent == 0:
                        break
                    off += sent
                except OSError as e:
                    if e.args and e.args[0] == _EAGAIN:
                        break
                    drop = True
                    break

            c[4] = off
            if drop or off >= total:
                _close_client(c)
                clients.pop(i)
                continue

        i += 1

def serve_pending_clients():
    global tx_led_off_at
    pump_start = time.ticks_ms()
    _accept_new()
    _pump_clients()

    while True:
        update_cap_state_machine()
        now = time.ticks_ms()
        if tx_led_off_at is not None and time.ticks_diff(now, tx_led_off_at) >= 0:
            tx_led.value(0)
            tx_led_off_at = None

        has_writer = False
        for c in clients:
            if c[1] == 1:
                has_writer = True
                break
        led_active = tx_led_off_at is not None

        if not has_writer and not led_active:
            break
        if time.ticks_diff(now, pump_start) >= 150:
            break

        time.sleep_ms(1)
        _accept_new()
        _pump_clients()

    tx_led.value(0)
    tx_led_off_at = None

# ==============================================================================
# 10. MAIN REAL-TIME EXECUTION LOOP
# ==============================================================================
gc_counter = 0

while True:
    compute_metrology_and_pid()
    serve_pending_clients()

    gc_counter += 1
    if gc_counter >= 10:
        gc.collect()
        gc_counter = 0