#!/usr/bin/env python3

"""
Use following code if GPIO on version 1
"""


# import time
# import gpiodq
# from aiohttp import web

# # ===== Hardware config (match your board) =====
# CHIP_NAME = "/dev/gpiochip3"  # same as your original
# PIN_CLK   = 6                 # A6
# PIN_DIN   = 7                 # A7
# N_LEDS    = 6         # adjust to your strip length
# BRIGHT    = 31               # 0..31 (global brightness)

# # Bit-bang clock ~5 kHz (adjustable)
# CLK_HZ   = 500000
# _DELAY_S = 1.0 / (2.0 * CLK_HZ)

# # ===== gpiod setup =====
# chip = gpiod.Chip(CHIP_NAME)
# clk  = chip.get_line(PIN_CLK)
# din  = chip.get_line(PIN_DIN)
# clk.request(consumer="led-clk", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
# din.request(consumer="led-din", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

# def _clk_hi(): clk.set_value(1)
# def _clk_lo(): clk.set_value(0)
# def _mosi(v): din.set_value(1 if v else 0)

# def _tick():
#     _clk_hi(); time.sleep(_DELAY_S)
#     _clk_lo(); time.sleep(_DELAY_S)

# def _send_byte(b: int):
#     # MSB first (APA102)
#     for i in range(7, -1, -1):
#         _mosi((b >> i) & 1)
#         _tick()

# def _send_bytes(buf: bytes | list[int]):
#     for b in buf:
#         _send_byte(b)

# def _frame_all(r: int, g: int, b: int) -> list[int]:
#     # APA102: start(4x 0x00), per-LED [0b111BBBBB, B, G, R], end(ceil(N/16) x 0xFF)
#     start = [0x00, 0x00, 0x00, 0x00]
#     hdr   = 0b11100000 | (BRIGHT & 0x1F)
#     led   = [hdr, b & 0xFF, g & 0xFF, r & 0xFF]
#     leds  = led * N_LEDS
#     end   = [0xFF] * ((N_LEDS + 15) // 16)
#     return start + leds + end

# def _write(frame: list[int]):
#     _send_bytes(frame)

# # ===== HTTP handlers (match led_server.py) =====
# async def led_on(_request):
#     _write(_frame_all(255, 255, 255))
#     print("led on")
#     return web.json_response({"ok": True, "state": "on"})

# async def led_off(_request):
#     # Send black (brightness field stays as-is; colors = 0)
#     _write(_frame_all(0, 0, 0))
#     # Some clones need an extra flush to fully clear:
#     _write(_frame_all(0, 0, 0))
#     print("led off")
#     return web.json_response({"ok": True, "state": "off"})

# # ===== App wiring =====
# app = web.Application()
# app.router.add_post("/on",  led_on)
# app.router.add_post("/off", led_off)

# async def on_shutdown(_app):
#     try:
#         _write(_frame_all(0, 0, 0))
#     except Exception:
#         pass
#     try:
#         clk.set_value(0); din.set_value(0)
#         clk.release();    din.release()
#         chip.close()
#     except Exception:
#         pass

# app.on_shutdown.append(on_shutdown)

# if __name__ == "__main__":
#     web.run_app(app, host="0.0.0.0", port=8080)


"""
Use following code if GPIO on version 2
"""

#!/usr/bin/env python3
import time
import gpiod
from gpiod.line import Direction, Value
from aiohttp import web

# ===== Hardware config (match your board) =====
CHIP_NAME = "/dev/gpiochip3"  # verify with `gpiodetect` / `gpioinfo`
PIN_CLK   = 6                 # line offsets on gpiochip
PIN_DIN   = 7
N_LEDS    = 6                 # adjust to your strip length
BRIGHT    = 31                # 0..31 (global brightness)

# Bit-bang clock ~5 kHz (adjustable)
CLK_HZ   = 5_000_000              # 5 kHz (your old code had 500_000 by mistake)
_DELAY_S = 1.0 / (2.0 * CLK_HZ)

# ===== gpiod v2.x setup =====
chip = gpiod.Chip(CHIP_NAME)
out_cfg = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)

# Keep separate handles for clk and din
clk = chip.request_lines(consumer="led-clk", config={PIN_CLK: out_cfg})
din = chip.request_lines(consumer="led-din", config={PIN_DIN: out_cfg})

def _clk_hi(): clk.set_value(PIN_CLK, Value.ACTIVE)
def _clk_lo(): clk.set_value(PIN_CLK, Value.INACTIVE)
def _mosi(v): din.set_value(PIN_DIN, Value.ACTIVE if v else Value.INACTIVE)

def _tick():
    _clk_hi(); time.sleep(_DELAY_S)
    _clk_lo(); time.sleep(_DELAY_S)

def _send_byte(b: int):
    # MSB first (APA102/SK9822)
    for i in range(7, -1, -1):
        _mosi((b >> i) & 1)
        _tick()

def _send_bytes(buf):
    for b in buf:
        _send_byte(int(b) & 0xFF)

def _frame_all(r: int, g: int, b: int) -> list[int]:
    """
    APA102 format:
      - Start frame: 4 x 0x00
      - Per-LED frame: [0b111BBBBB, B, G, R]
      - End frame: ceil(N/16) x 0xFF  (often +1 extra for safety)
    """
    start = [0x00, 0x00, 0x00, 0x00]
    hdr   = 0b11100000 | (BRIGHT & 0x1F)
    led   = [hdr, b & 0xFF, g & 0xFF, r & 0xFF]
    leds  = led * N_LEDS
    end   = [0xFF] * (((N_LEDS + 15) // 16) + 1)  # extra flush for robustness
    return start + leds + end

def _write(frame: list[int]):
    _send_bytes(frame)

# ===== HTTP handlers =====
async def led_on(_request):
    _write(_frame_all(255, 255, 255))
    print("led on")
    return web.json_response({"ok": True, "state": "on"})

async def led_off(_request):
    # Send black (brightness field stays as-is; colors = 0)
    _write(_frame_all(0, 0, 0))
    # Some clones need an extra flush to fully clear:
    _write(_frame_all(0, 0, 0))
    print("led off")
    return web.json_response({"ok": True, "state": "off"})

# ===== App wiring =====
app = web.Application()
app.router.add_post("/on",  led_on)
app.router.add_post("/off", led_off)

async def on_shutdown(_app):
    try:
        _write(_frame_all(0, 0, 0))
    except Exception:
        pass
    try:
        clk.set_value(PIN_CLK, Value.INACTIVE)
        din.set_value(PIN_DIN, Value.INACTIVE)
        clk.release(); din.release()
        chip.close()
    except Exception:
        pass

app.on_shutdown.append(on_shutdown)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)
