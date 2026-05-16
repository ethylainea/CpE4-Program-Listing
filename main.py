# main.py - Raspberry Pi Pico W
# Receives sensor data from ESP32 via UART,
# runs ML prediction, sends result back via UART
#
# Wiring:
#   Pico W GP0 (TX) --> ESP32 GPIO16 (RX2)
#   Pico W GP1 (RX) <-- ESP32 GPIO17 (TX2)
#   Pico W GND      --- ESP32 GND
#
# UART in  (from ESP32): moisture,ec_us_cm,ph,tilt\n
# UART out (to ESP32)  : soil_result,slope_result,slips\n

from machine import UART, Pin
from combined_model import soil_model_predict, slope_model_predict, slip_model_predict
import math
import time

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

prev_tilt = None

def get_delta_tilt(current_tilt):
    global prev_tilt
    if prev_tilt is None:
        prev_tilt = current_tilt
        return 0.0
    delta     = abs(current_tilt - prev_tilt)
    prev_tilt = current_tilt
    return delta

def compute_slips(moisture, delta_tilt, soil_result, slope_result):
    if soil_result == "not_suitable" or slope_result == "stable":
        return 0
    return max(0, int(round(slip_model_predict({
        "Moisture_%": moisture,
        "delta_tilt": delta_tilt
    }))))

# ---------------------------------------------------------------------------
# SHEAR STRENGTH DEFICIT (ΔS) — Wu-Waldron moisture- and slope-dependent formula
# ΔS = ((moisture - 50) / 30) * (sin(θ) / sin(θ_max)) * 20000  [Pa]
# θ     = current tilt angle (degrees)
# θ_max = 2° (reference maximum slope angle)
# Only applies when moisture > 50%; otherwise ΔS = 0.0 (no intervention needed)
# ---------------------------------------------------------------------------
_SIN_TILT_MAX = math.sin(math.radians(2.0))  # sin(2°), computed once at startup

def compute_delta_s(moisture, tilt):
    if moisture <= 50.0:
        return 0.0
    sin_tilt = math.sin(math.radians(abs(tilt)))
    return ((moisture - 50.0) / 30.0) * (sin_tilt / _SIN_TILT_MAX) * 20000.0

VALID_SOIL  = {"suitable", "not_suitable"}
VALID_SLOPE = {"stable", "pre_failure", "failure_imminent"}

SOIL_LABELS  = {0: "suitable",  1: "not_suitable",
                "0": "suitable", "1": "not_suitable"}
SLOPE_LABELS = {0: "stable", 1: "pre_failure", 2: "failure_imminent",
                "0": "stable", "1": "pre_failure", "2": "failure_imminent"}

def decode_soil(pred):
    if isinstance(pred, str):
        if pred in VALID_SOIL:  return pred
        if pred in SOIL_LABELS: return SOIL_LABELS[pred]
        return "unknown_" + pred
    try:
        return SOIL_LABELS.get(int(pred), "unknown_" + str(pred))
    except Exception:
        return "unknown_" + str(pred)

def decode_slope(pred):
    if isinstance(pred, str):
        if pred in VALID_SLOPE:  return pred
        if pred in SLOPE_LABELS: return SLOPE_LABELS[pred]
        return "unknown_" + pred
    try:
        return SLOPE_LABELS.get(int(pred), "unknown_" + str(pred))
    except Exception:
        return "unknown_" + str(pred)

led = Pin("LED", Pin.OUT)

def blink(n=1, fast=False):
    pause = 0.05 if fast else 0.1
    for _ in range(n):
        led.on();  time.sleep(pause)
        led.off(); time.sleep(pause)

print("VetiverTrack ML Node - Pico W")
blink(3)

buf = b""

while True:
    if uart.any():
        buf += uart.read(uart.any())

        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)

            try:
                line = line.strip().decode("utf-8")
            except Exception:
                continue

            if not line:
                continue

            try:
                parts = line.split(",")
                if len(parts) != 4:
                    print("[WARN] bad format: " + line)
                    continue

                moisture = float(parts[0])
                ec_us_cm = float(parts[1])
                ph       = float(parts[2])
                tilt     = float(parts[3])
                ec_ds_cm = ec_us_cm / 1000.0
                delta_tilt = get_delta_tilt(tilt)

                soil_result  = decode_soil( soil_model_predict({"EC_dScm": ec_ds_cm, "pH": ph}))
                slope_result = decode_slope(slope_model_predict({"delta_tilt": delta_tilt, "Moisture_%": moisture}))
                slips        = compute_slips(moisture, delta_tilt, soil_result, slope_result)
                delta_s      = compute_delta_s(moisture, tilt)

                uart.write((soil_result + "," + slope_result + "," + str(slips) + "," + "{:.4f}".format(delta_s) + "\n").encode("utf-8"))
                uart.flush()

                print("IN  moisture={:.1f}% EC={:.0f} pH={:.1f} tilt={:.2f} delta={:.3f}".format(
                    moisture, ec_us_cm, ph, tilt, delta_tilt))
                print("OUT soil={} slope={} slips={} delta_s={:.4f} kPa".format(soil_result, slope_result, slips, delta_s))

                blink(1)

            except Exception as e:
                print("[ERROR] " + str(e))
                blink(3, fast=True)

    time.sleep(0.01)
