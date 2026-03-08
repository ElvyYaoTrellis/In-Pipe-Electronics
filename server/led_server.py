#!/usr/bin/env python3
import asyncio
import time
import spidev
from aiohttp import web

# --- Hardware config ---
N_LEDS = 12
BUS, DEV = 3, 0        # /dev/spidev3.0  (SPI3 on Radxa Zero 3W)
HZ      = 2_400_000    # 2.4 MHz → 417 ns/bit; 3 SPI bits ≈ WS2812 NZR timing
#   logic 0 → 0b100  (T0H≈417 ns, T0L≈833 ns)
#   logic 1 → 0b110  (T1H≈833 ns, T1L≈417 ns)
SYMBOL_0 = 0b100
SYMBOL_1 = 0b110

# Zero bytes needed for a ≥300 µs reset at 2.4 MHz (8 bits × 417 ns ≈ 3.3 µs/byte → ~91 bytes).
# This is sent before AND after each frame so the strip always latches cleanly.
_RESET_BYTES = bytes(91)


def _encode_grb(grb: bytes) -> bytearray:
    """Convert raw GRB bytes to the SPI bitstream WS2812 needs."""
    out = bytearray()
    acc, acc_bits = 0, 0
    for byte in grb:
        for bit in range(7, -1, -1):
            sym = SYMBOL_1 if (byte >> bit) & 1 else SYMBOL_0
            acc = (acc << 3) | sym
            acc_bits += 3
            while acc_bits >= 8:
                shift = acc_bits - 8
                out.append((acc >> shift) & 0xFF)
                acc_bits -= 8
                acc &= (1 << acc_bits) - 1
    if acc_bits:
        out.append((acc << (8 - acc_bits)) & 0xFF)
    return out


def _frame_all(r: int, g: int, b: int, brightness: float = 1.0) -> bytes:
    br = max(0.0, min(1.0, float(brightness)))
    r = int((r & 255) * br)
    g = int((g & 255) * br)
    b = int((b & 255) * br)
    return bytes([g, r, b]) * N_LEDS   # WS2812 wire order is GRB


def _write_ws2812(grb_frame: bytes, clear_twice: bool = False) -> None:
    spi = spidev.SpiDev()
    spi.open(BUS, DEV)
    spi.max_speed_hz = HZ
    spi.mode    = 0
    spi.no_cs   = True   # WS2812 has no chip-select; suppress CS toggling

    payload = _encode_grb(grb_frame)

    # Pre-reset: flush any mid-frame state left from a previous write
    spi.writebytes2(_RESET_BYTES)

    spi.writebytes2(payload)

    # Post-reset: latch the frame (WS2812 needs >50 µs low; _RESET_BYTES ≈ 300 µs)
    spi.writebytes2(_RESET_BYTES)

    if clear_twice:                     # extra pass for stubborn "1 LED stays on" issue
        spi.writebytes2(payload)
        spi.writebytes2(_RESET_BYTES)

    spi.close()


async def _write_async(grb_frame: bytes, clear_twice: bool = False) -> None:
    """Run the blocking SPI write in a thread so the event loop stays free."""
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, _write_ws2812, grb_frame, clear_twice)


# --- HTTP handlers ---

async def led_on(_):
    await _write_async(_frame_all(255, 255, 255, brightness=0.20))
    print("led on (white)")
    return web.json_response({"ok": True, "state": "on"})


async def led_off(_):
    await _write_async(_frame_all(0, 0, 0), clear_twice=True)
    print("led off")
    return web.json_response({"ok": True, "state": "off"})


# POST /set  {"r":255,"g":0,"b":0,"brightness":0.5}
async def led_set(request):
    data = await request.json()
    r          = int(data.get("r", 0))
    g          = int(data.get("g", 0))
    b          = int(data.get("b", 0))
    brightness = float(data.get("brightness", 1.0))
    await _write_async(_frame_all(r, g, b, brightness=brightness))
    return web.json_response({"ok": True, "state": "set",
                               "r": r, "g": g, "b": b, "brightness": brightness})


async def on_shutdown(_app):
    """Turn off all LEDs when the server stops."""
    try:
        _write_ws2812(_frame_all(0, 0, 0), clear_twice=True)
    except Exception:
        pass


app = web.Application()
app.router.add_post("/on",  led_on)
app.router.add_post("/off", led_off)
app.router.add_post("/set", led_set)
app.on_shutdown.append(on_shutdown)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)
