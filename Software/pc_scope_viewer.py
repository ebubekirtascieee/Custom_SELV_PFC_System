import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SERIAL_PORT = None  
BAUD_RATE   = 115200

def find_pico_port():
    if SERIAL_PORT: return SERIAL_PORT
    for p in serial.tools.list_ports.comports():
        if "2E8A" in p.hwid.upper() or "USB" in p.description.upper() or "SERIAL" in p.description.upper():
            return p.device
    raise RuntimeError("Pico COM port not found!")

ser = serial.Serial(find_pico_port(), BAUD_RATE, timeout=1.0)
ser.reset_input_buffer()

# ==============================================================================
# GUI SETUP: 3 Rows (Voltage, Current, Terminal)
# ==============================================================================

plt.style.use("dark_background")
fig, (ax_v, ax_i, ax_log) = plt.subplots(3, 1, gridspec_kw={'height_ratios': [3, 3, 2]}, figsize=(12, 9), sharex=False)
fig.canvas.manager.set_window_title("Pico 2 W - Active PFC Hardware Automation Test")

# 1. Voltage Plot
line_v_raw, = ax_v.plot([], [], color="#FFD700", lw=1.8)
ax_v.set_ylabel("AC Line Voltage (V)", color="#FFD700")
ax_v.set_ylim(-42, 42)
ax_v.grid(True, alpha=0.25, linestyle="--")
title_v = ax_v.set_title("Waiting for data...", fontsize=11, pad=10)

# 2. Current Plot
line_i_raw, = ax_i.plot([], [], color="#00E5FF", lw=1.2, alpha=0.55, label="Raw (mA)")
line_i_dft, = ax_i.plot([], [], color="#00FF66", lw=2.3, label="50Hz DFT (mA)")
ax_i.set_ylabel("AC Load Current (mA)", color="#00FF66")
ax_i.set_xlabel("Time from Zero-Cross Trigger (ms)", fontsize=10)
ax_i.set_ylim(-150, 150)
ax_i.grid(True, alpha=0.25, linestyle="--")
ax_i.legend(loc="upper right")

# 3. Terminal Log View
ax_log.set_facecolor('#0f0f0f')
ax_log.set_xticks([])
ax_log.set_yticks([])
ax_log.set_title("Pico 2 W Live Hardware Events Terminal", fontsize=10, color="white")
log_text = ax_log.text(0.02, 0.90, "", color="#00FF00", family="monospace", fontsize=10, va="top", ha="left")

DISPLAY_WINDOW_MS = 80.0
log_history = ["--- System Booted. Waiting for COM stream... ---"]

def read_one_frame():
    new_logs = []
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("LOG,"):
            new_logs.append(line[4:])
            continue
        if line == "FRAME_START":
            break
        if not line:
            return None, None, []

    dft_metrics = None
    rows = []
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("LOG,"):
            new_logs.append(line[4:])
            continue
        if line == "FRAME_END" or not line:
            break
        if line.startswith("DFT,"):
            try: dft_metrics = [float(x) for x in line[4:].split(",")]
            except ValueError: pass
        else:
            parts = line.split(",")
            if len(parts) == 3:
                try: rows.append([float(x) for x in parts])
                except ValueError: pass
                
    if len(rows) > 50 and dft_metrics and len(dft_metrics) == 6:
        return dft_metrics, np.array(rows), new_logs
    return None, None, new_logs

def update(frame_idx):
    global log_history
    if ser.in_waiting > 20000:
        ser.reset_input_buffer()

    dft, data, new_logs = read_one_frame()
    
    # Update Terminal Window First
    if new_logs:
        log_history.extend(new_logs)
        if len(log_history) > 6:
            log_history = log_history[-6:]
        log_text.set_text("\n".join(log_history))

    if data is None or dft is None:
        return line_v_raw, line_i_raw, line_i_dft, log_text

    (v1_rms, i1_rms_mA, p_shunt, q_shunt, s_shunt, dpf_shunt) = dft
    t_ms, v_ac, i_mA = data[:, 0], data[:, 1], data[:, 2] * 1000.0

    # Software Trigger
    trig_idx = 0
    for k in range(5, len(v_ac) // 2):
        if v_ac[k - 1] < 0.0 and v_ac[k] >= 0.0 and v_ac[k + 2] > v_ac[k]:
            trig_idx = k
            break

    t_trig = t_ms[trig_idx:] - t_ms[trig_idx]
    v_trig, i_trig = v_ac[trig_idx:], i_mA[trig_idx:]
    mask = t_trig <= DISPLAY_WINDOW_MS
    t_win = t_trig[mask]

    # Reconstruct pure 50Hz wave precisely from the P and Q vectors
    i1_peak_mA = i1_rms_mA * np.sqrt(2.0)
    
    # Calculate exact continuous phase angle (phi = atan2(Q, P))
    # Current waveform equation is I(t) = I_peak * sin(w*t - phi)
    phase_rad = -np.arctan2(q_shunt, p_shunt)
    
    i_dft_wave = i1_peak_mA * np.sin(2.0 * np.pi * 0.050 * t_win + phase_rad)
    
    line_v_raw.set_data(t_win, v_trig[mask])
    line_i_raw.set_data(t_win, i_trig[mask])
    line_i_dft.set_data(t_win, i_dft_wave)
    
    ax_v.set_xlim(0, DISPLAY_WINDOW_MS)
    ax_i.set_xlim(0, DISPLAY_WINDOW_MS)
    ax_i.set_ylim(-max(100.0, i1_peak_mA * 1.5), max(100.0, i1_peak_mA * 1.5))

    q_type = "IND" if q_shunt > 0.02 else ("CAP" if q_shunt < -0.02 else "RES")
    title_v.set_text(
        f"SHUNT (R2) | V1_RMS: {v1_rms:.2f}V | I1_RMS: {i1_rms_mA:.1f}mA | "
        f"P: {p_shunt:.2f}W | Q: {q_shunt:+.2f}VAr ({q_type}) | DPF: {dpf_shunt:.3f}"
    )
    
    return line_v_raw, line_i_raw, line_i_dft, log_text

ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()