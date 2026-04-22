#!/usr/bin/env python3
import asyncio
import spidev
from aiohttp import web

# --- Hardware config ---
N_RING = 12       # front ring LEDs (indices 0-11)
N_LEDS = 13       # ring + 1 rear LED (index 12)
BUS, DEV = 3, 0   # /dev/spidev3.0  (SPI3 on Radxa Zero 3W)
HZ      = 2_400_000
SYMBOL_0 = 0b100
SYMBOL_1 = 0b110
_RESET_BYTES = bytes(91)

_spi = spidev.SpiDev()
_spi.open(BUS, DEV)
_spi.max_speed_hz = HZ
_spi.mode = 0

# Shared LED state (r, g, b, brightness)
_ring = (0, 0, 0, 0.0)
_rear = (0, 0, 0, 0.0)


def _encode_grb(grb: bytes) -> bytearray:
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


def _pixel(r, g, b, brightness):
    br = max(0.0, min(1.0, float(brightness)))
    return bytes([int(g * br), int(r * br), int(b * br)])  # GRB wire order


def _build_frame():
    ring_px = _pixel(*_ring) * N_RING
    rear_px = _pixel(*_rear)
    return ring_px + rear_px


def _write_ws2812(frame: bytes, clear_twice: bool = False) -> None:
    payload = _encode_grb(frame)
    combined = _RESET_BYTES + payload + _RESET_BYTES
    _spi.writebytes2(combined)
    if clear_twice:
        _spi.writebytes2(_RESET_BYTES + payload + _RESET_BYTES)


async def _write_async(frame: bytes, clear_twice: bool = False) -> None:
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, _write_ws2812, frame, clear_twice)


# ── Ring endpoints ────────────────────────────────────────────────────────────

async def led_on(_):
    global _ring
    _ring = (255, 255, 255, 0.20)
    await _write_async(_build_frame())
    return web.json_response({"ok": True, "state": "on"})


async def led_off(_):
    global _ring
    _ring = (0, 0, 0, 0.0)
    await _write_async(_build_frame(), clear_twice=True)
    return web.json_response({"ok": True, "state": "off"})


async def led_set(request):
    global _ring
    data = await request.json()
    r  = int(data.get("r", 0))
    g  = int(data.get("g", 0))
    b  = int(data.get("b", 0))
    br = float(data.get("brightness", 1.0))
    _ring = (r, g, b, br)
    await _write_async(_build_frame())
    return web.json_response({"ok": True, "r": r, "g": g, "b": b, "brightness": br})


# ── Rear LED endpoints ────────────────────────────────────────────────────────

async def rear_on(_):
    global _rear
    _rear = (255, 255, 255, 0.20)
    await _write_async(_build_frame())
    return web.json_response({"ok": True, "state": "on"})


async def rear_off(_):
    global _rear
    _rear = (0, 0, 0, 0.0)
    await _write_async(_build_frame(), clear_twice=True)
    return web.json_response({"ok": True, "state": "off"})


async def rear_set(request):
    global _rear
    data = await request.json()
    r  = int(data.get("r", 0))
    g  = int(data.get("g", 0))
    b  = int(data.get("b", 0))
    br = float(data.get("brightness", 1.0))
    _rear = (r, g, b, br)
    await _write_async(_build_frame())
    return web.json_response({"ok": True, "r": r, "g": g, "b": b, "brightness": br})


async def led_status(_):
    return web.json_response({
        "ok": True,
        "ring": {"r": _ring[0], "g": _ring[1], "b": _ring[2], "brightness": _ring[3]},
        "rear": {"r": _rear[0], "g": _rear[1], "b": _rear[2], "brightness": _rear[3]},
    })


async def on_shutdown(_app):
    try:
        _write_ws2812(_pixel(0, 0, 0, 0.0) * N_LEDS, clear_twice=True)
    except Exception:
        pass


app = web.Application()
app.router.add_post("/on",      led_on)
app.router.add_post("/off",     led_off)
app.router.add_post("/set",     led_set)
app.router.add_post("/rear/on",  rear_on)
app.router.add_post("/rear/off", rear_off)
app.router.add_post("/rear/set", rear_set)
app.router.add_get( "/status",  led_status)
app.on_shutdown.append(on_shutdown)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)
