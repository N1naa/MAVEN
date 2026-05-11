# === ESP32-C3 and ESP32 Bluetooth PWM Controller + IMU Pitch (MicroPython) ===
# Updated:
# - IMU parse/update target set to 100 Hz
# - BLE pitch stream target set to 100 Hz
# - Removed pitch deadband/threshold so pitch is sent every stream tick
# Commands:
#   PWM,<freq>,<duty>   -> start/update PWM on PIN_NUM
#   STOP                -> stop PWM (drive LOW)
#   CAL[,ms]            -> calibrate zero by averaging for ms (default 3000)
#   STREAM,ON|OFF       -> stream control (default ON)
# Stream:
#   PITCH,<deg> at PRINT_HZ when enabled
#   BATT,<volts>,<pct> every BATT_DT
#
from machine import Pin, PWM, UART, ADC
from micropython import const
import ubluetooth as bt
import time, struct

time.sleep(3) # otherwise sometimes stuck when opening thonny

# ---------- USER CONFIG ----------
NAME     = "NameBLEIMU8" # change here
IMU_ID   = 8 # change here
PIN_NUM  = 3
# BNO08x RVC
IMU_UID  = 1 

# change here
# ESP32-C3 - SEEED
#IMU_RX   = 6   # IMU TX -> ESP RX
#IMU_TX   = 6   # UART needs a TX pin even if not used

# ESP32 - pico- D4
#IMU_RX   = 4   # IMU TX -> ESP RX
#IMU_TX   = 4

# ESP32-C6 - SEEED
IMU_RX   = 22   # IMU TX -> ESP RX
IMU_TX   = 22

# continue
BAUD     = 115200

# IMU processing + streaming
PRINT_HZ   = 100         # BLE pitch send rate
EMA_A      = 0.85        # exponential smoothing factor 0..1
IMU_HZ     = 100         # IMU parse/update cadence

# Battery ADC config (100k/100k divider)
VBAT_PIN  = 2
DIVIDER   = 2.0
ADC_VMAX  = 3.60
CAL_K     = 0.871
_ADC_MAX  = 65535.0
BATT_DT   = 60000        # send every 60 s

# Advertising interval (microseconds). Higher = less power when not connected.
ADV_US    = 500_000      # 500 ms

# Debug
DEBUG_RATE = False
RATE_DT    = 5000        # ms, if DEBUG_RATE True

# Hold PWM pin LOW immediately on boot to avoid chirp
Pin(PIN_NUM, Pin.OUT, value=0)

# Battery ADC setup
try:
    _adc = ADC(Pin(VBAT_PIN))
    try:
        _adc.atten(ADC.ATTN_11DB)
    except:
        pass
except Exception:
    _adc = None

_vbat_ema = None
_batt_last = time.ticks_ms()

# ---------- BLE setup ----------
ble = bt.BLE()
ble.active(True)
ble.config(mtu=128)

_UART_UUID = bt.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_TX_UUID   = bt.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")  # Notify
_RX_UUID   = bt.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")  # Write

_UART_TX  = (_TX_UUID, bt.FLAG_NOTIFY)
_UART_RX  = (_RX_UUID, bt.FLAG_WRITE | bt.FLAG_WRITE_NO_RESPONSE)
_UART_SVC = (_UART_UUID, (_UART_TX, _UART_RX))

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

services  = ble.gatts_register_services((_UART_SVC,))
svc       = services[0]
tx_raw    = svc[0]
rx_raw    = svc[1]
tx_handle = tx_raw[0] if isinstance(tx_raw, (tuple, list)) else tx_raw
rx_handle = rx_raw[0] if isinstance(rx_raw, (tuple, list)) else rx_raw

conn_handle = None
pwm = None

# --- meters ---
imu_frames = 0
ble_msgs   = 0
rate_last  = time.ticks_ms()

# streaming + calibration state
stream_enabled = True
cal_active = False
cal_deadline = 0
cal_sum = 0.0
cal_n = 0
pitch_offset = 0.0
_last_yaw = 0.0; _last_roll = 0.0
_last_ax  = 0.0; _last_ay  = 0.0; _last_az = 0.0

def _notify(msg: str):
    try:
        if conn_handle is not None:
            ble.gatts_notify(conn_handle, tx_handle, (msg + "\n").encode())
    except:
        pass

def _start_pwm(freq, duty):
    global pwm
    f = int(freq)
    d = max(0, min(100, int(duty)))
    if pwm is None:
        pwm = PWM(Pin(PIN_NUM))
        pwm.freq(f)
        pwm.duty_u16(0)
    pwm.freq(f)
    pwm.duty_u16(int(d * 65535 // 100))
    _notify(f"ACK PWM {f}Hz {d}%")

def _stop_pwm():
    global pwm
    try:
        if pwm:
            try:
                pwm.duty_u16(0)
                time.sleep_ms(5)
            except:
                pass
            pwm.deinit()
            pwm = None
    finally:
        Pin(PIN_NUM, Pin.OUT).value(0)
    _notify("ACK STOP")

def _start_calibration(duration_ms=3000):
    global cal_active, cal_deadline, cal_sum, cal_n
    cal_active = True
    cal_deadline = time.ticks_add(time.ticks_ms(), int(duration_ms))
    cal_sum = 0.0
    cal_n = 0
    _notify(f"ACK CAL START {int(duration_ms)}ms")

def _update_calibration(raw_pitch_deg):
    global cal_active, pitch_offset, cal_sum, cal_n
    if not cal_active:
        return
    cal_sum += raw_pitch_deg
    cal_n += 1
    if time.ticks_diff(cal_deadline, time.ticks_ms()) <= 0:
        if cal_n > 0:
            pitch_offset = cal_sum / cal_n
            _notify(f"ACK CAL DONE offset={pitch_offset:.2f}")
        else:
            _notify("ERR CAL no-data")
        cal_active = False

def on_ble(event, data):
    global conn_handle, stream_enabled, cal_active
    if event == _IRQ_CENTRAL_CONNECT:
        conn_handle, _, _ = data
        _notify("CONNECTED")
    elif event == _IRQ_CENTRAL_DISCONNECT:
        conn_handle = None
        cal_active = False
        _stop_pwm()
        ble.gap_advertise(ADV_US, adv_data=adv, resp_data=resp)
    elif event == _IRQ_GATTS_WRITE:
        handle = data[1]
        if handle == rx_handle:
            try:
                msg = ble.gatts_read(rx_handle).decode().strip()
                if not msg:
                    return
                up = msg.upper()

                if up.startswith("PWM"):
                    parts = msg.split(",")
                    if len(parts) != 3:
                        raise ValueError("PWM expects PWM,<freq>,<duty>")
                    _, f, d = parts
                    _start_pwm(f.strip(), d.strip())

                elif up.startswith("STOP"):
                    _stop_pwm()

                elif up.startswith("CAL"):
                    dur_ms = 3000
                    parts = msg.split(',')
                    if len(parts) >= 2:
                        try:
                            dur_ms = max(500, int(parts[1]))
                        except:
                            pass
                    _start_calibration(dur_ms)

                elif up.startswith("STREAM"):
                    parts = up.split(',')
                    if len(parts) >= 2 and parts[1].strip() in ("ON", "OFF"):
                        stream_enabled = (parts[1].strip() == "ON")
                        _notify("ACK STREAM " + ("ON" if stream_enabled else "OFF"))
                    else:
                        raise ValueError("STREAM expects STREAM,ON or STREAM,OFF")
                elif up == "ID?":
                    _notify("ID,%d" % IMU_ID) # added this
                else:
                    raise ValueError("Unknown cmd")

            except Exception as e:
                _notify("ERR " + str(e))

def _adc_read_u16_avg(n=8, settle_us=250):
    if _adc is None:
        return 0
    _ = _adc.read_u16()
    s = 0
    for _i in range(n):
        time.sleep_us(settle_us)
        s += _adc.read_u16()
    return s // max(1, n)

def _read_vbat_volts():
    if _adc is None:
        return 0.0
    raw   = _adc_read_u16_avg()
    v_pin = (raw / _ADC_MAX) * ADC_VMAX
    return (v_pin * DIVIDER) * CAL_K

def _vbat_tick(alpha=0.25):
    global _vbat_ema
    v = _read_vbat_volts()
    _vbat_ema = v if _vbat_ema is None else (_vbat_ema + alpha * (v - _vbat_ema))
    return _vbat_ema

def _vbat_percent(v):
    if v >= 4.20: return 100
    if v <= 3.00: return 0
    if v < 3.70:
        return int((v - 3.00) * (50.0 / 0.70))
    else:
        return int(50 + (v - 3.70) * (50.0 / 0.50))

def adv_payload(name=None):
    p = bytearray()
    p += bytes((2, 0x01, 0x06))
    if name:
        n = name.encode()
        p += bytes((len(n) + 1, 0x09)) + n
    return bytes(p)

NUS_UUID_LE = bytes((
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
))
adv  = adv_payload(NAME)
resp = bytes((len(NUS_UUID_LE) + 1, 0x07)) + NUS_UUID_LE
#adv = bytes((2, 0x01, 0x06)) + bytes((len(NUS_UUID_LE)+1, 0x07)) + NUS_UUID_LE
#_n = NAME.encode()[:29]
#resp = bytes((len(_n)+1, 0x09)) + _n

ble.irq(on_ble)
ble.gap_advertise(ADV_US, adv_data=adv, resp_data=resp)
print("Advertising as", NAME)

# ---------- IMU (UART-RVC) ----------
imu = UART(IMU_UID, BAUD, rx=Pin(IMU_RX), tx=Pin(IMU_TX), timeout=0, timeout_char=0)
rvc_buf = bytearray()

def _parse_rvc_latest():
    """Return latest (yaw, pitch, roll) in deg from RVC stream, or None."""
    global rvc_buf, imu_frames
    out = None
    while True:
        i = rvc_buf.find(b'\xAA\xAA')
        if i < 0:
            rvc_buf = rvc_buf[-1:]
            break
        if len(rvc_buf) - i < 19:
            if i > 0:
                rvc_buf = rvc_buf[i:]
            break
        frame = rvc_buf[i+2:i+19]
        rvc_buf = rvc_buf[i+19:]
        if (sum(frame[0:16]) & 0xFF) != frame[16]:
            continue
        #y_i, p_i, r_i, _, _, _ = struct.unpack('<hhhhhh', frame[1:13])
        #out = (y_i/100.0, p_i/100.0, r_i/100.0)
        y_i, p_i, r_i, ax_i, ay_i, az_i = struct.unpack('<hhhhhh', frame[1:13]) # changé ça
        out = (y_i/100.0, p_i/100.0, r_i/100.0, ax_i/100.0, ay_i/100.0, az_i/100.0) # changé ça
        imu_frames += 1
    return out

def _wrap180(x):
    if x <= -180 or x > 180:
        x = ((x + 180) % 360) - 180
    return x

pitch_f = None
last_tx = time.ticks_ms()
TX_DT   = max(1, int(1000 / PRINT_HZ))

# pace IMU work
IMU_DT = max(1, int(1000 / IMU_HZ))
last_imu = time.ticks_ms()

# ---------- Main loop ----------
while True:
    now = time.ticks_ms()

    # --- IMU read/parse/update at IMU_HZ ---
    if time.ticks_diff(now, last_imu) >= IMU_DT:
        last_imu = now

        b = imu.read()
        if b:
            rvc_buf.extend(b)

        v = _parse_rvc_latest()
        if v:
            _last_yaw, p_raw_raw, _last_roll, _last_ax, _last_ay, _last_az = v
            p_raw = _wrap180(p_raw_raw)
            #p_raw = _wrap180(v[1]) # changé ça
            _update_calibration(p_raw)
            p = _wrap180(p_raw - pitch_offset)

            if pitch_f is None:
                pitch_f = p
            else:
                pitch_f += EMA_A * (p - pitch_f)

    # --- transmit at PRINT_HZ with no deadband ---
    if (conn_handle is not None) and stream_enabled and (pitch_f is not None) and (time.ticks_diff(now, last_tx) >= TX_DT):
        #_notify(f"PITCH,{pitch_f:.2f}") # changé ça
        #_notify(f"PITCH,{pitch_f:.2f},{now}")
        #_notify(f"PITCH,{pitch_f:.2f},{now},{_last_yaw:.2f},{_last_roll:.2f},{_last_ax:.2f},{_last_ay:.2f},{_last_az:.2f}")
        _notify(f"PITCH,{pitch_f:.2f},{now},{_last_yaw:.1f},{_last_roll:.1f},{_last_ax:.1f},{_last_ay:.1f},{_last_az:.1f}")
        ble_msgs += 1
        last_tx = now

    # --- battery every BATT_DT ---
    if time.ticks_diff(now, _batt_last) >= BATT_DT:
        vb = _vbat_tick()
        pc = _vbat_percent(vb)
        _notify(f"BATT,{vb:.3f},{pc}")
        _batt_last = now

    # --- optional debug meter ---
    if DEBUG_RATE and (time.ticks_diff(now, rate_last) >= RATE_DT):
        dt_ms = max(1, time.ticks_diff(now, rate_last))
        imu_hz = (imu_frames * 1000.0) / dt_ms
        ble_hz = (ble_msgs * 1000.0) / dt_ms
        _notify(f"RATE IMU~{imu_hz:.1f}Hz, BLE~{ble_hz:.1f}Hz, PRINT_HZ={PRINT_HZ}, IMU_HZ={IMU_HZ}")
        imu_frames = 0
        ble_msgs   = 0
        rate_last  = now

    # --- short sleep to support 100 Hz loop timing ---
    time.sleep_ms(1)

