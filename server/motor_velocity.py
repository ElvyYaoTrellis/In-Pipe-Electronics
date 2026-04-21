#!/usr/bin/env python3
import argparse
import sys
import time
import errno
import os
import json
import asyncio
import threading
from dataclasses import dataclass
from typing import Optional, Dict, Any

from smbus2 import SMBus, i2c_msg

try:
    from aiohttp import web
except Exception:
    web = None

# --- DRV8234 Registers ---
REG_FAULT      = 0x00
REG_CONFIG0    = 0x09
REG_CONFIG3    = 0x0C  # IMODE[7:6], SMODE[5], INT_VREF[4]
REG_CONFIG4    = 0x0D
REG_REG_CTRL0  = 0x0E  # REG_CTRL[4:3]

# --- Bit masks ---
CFG0_EN_OUT    = 0x80
CFG0_CLR_FLT   = 0x02

CFG4_PMODE     = 0x08  # writable only when EN_OUT=0
CFG4_I2C_BC    = 0x04  # writable only when EN_OUT=0
CFG4_I2C_EN1   = 0x02  # used as EN in PH/EN
CFG4_I2C_PH2   = 0x01  # used as PH in PH/EN

# ------------------ Fault bits (from existing print_status mapping) ------------------
FLT_ACTIVE = 0x80
FLT_STALL  = 0x20
FLT_OCP    = 0x10
FLT_OVP    = 0x08
FLT_TSD    = 0x04
FLT_NPOR   = 0x02

# ======================================================
# DAC
# ======================================================
class MCP4725:
    """
    MCP4725 fast mode (2-byte) write:
      [00 PD1 PD0 D11..D8] [D7..D0]
    """
    def __init__(self, bus: SMBus, addr: int, vdd: float = 3.3, name: str = "DAC"):
        self.bus = bus
        self.addr = addr
        self.vdd = float(vdd)
        self.name = name
        self.last_code = None

    def probe(self) -> bool:
        try:
            self.write_code(0)
            return True
        except OSError:
            return False

    def write_code(self, code: int, pd: int = 0):
        code = int(max(0, min(4095, code)))
        pd = int(max(0, min(3, pd)))
        b0 = ((pd & 0x3) << 4) | ((code >> 8) & 0x0F)
        b1 = code & 0xFF
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, [b0, b1]))
        self.last_code = code

    def set_voltage(self, vout: float) -> float:
        if self.vdd <= 0:
            raise ValueError("DAC vdd must be > 0")
        v = max(0.0, min(self.vdd, float(vout)))
        code = int(round((v / self.vdd) * 4095.0))
        self.write_code(code)
        return (code / 4095.0) * self.vdd

    def set_current_limit(self, amps: float, ripropi_ohm: float, aipropi_uA_per_A: float):
        # VVREF = ITRIP * AIPROPI * RIPROPI
        amps = float(amps)
        vref_cmd = amps * (aipropi_uA_per_A * 1e-6) * ripropi_ohm
        vref_act = self.set_voltage(vref_cmd)
        itrip_act = (vref_act / ripropi_ohm) / (aipropi_uA_per_A * 1e-6)
        return vref_act, itrip_act

# ======================================================
# DRV8234
# ======================================================
class DRV8234:
    def __init__(self, bus: SMBus, addr: int, name: str):
        self.bus = bus
        self.addr = addr
        self.name = name
        self.last_dir = None  # bool forward?

    def read_reg(self, reg: int) -> int:
        w = i2c_msg.write(self.addr, [reg & 0xFF])
        r = i2c_msg.read(self.addr, 1)
        self.bus.i2c_rdwr(w, r)
        return list(r)[0] & 0xFF

    def read_reg_stop(self, reg: int) -> int:
        for attempt in range(20):
            try:
                self.bus.i2c_rdwr(i2c_msg.write(self.addr, [reg & 0xFF]))
                time.sleep(0.0005)
                r = i2c_msg.read(self.addr, 1)
                self.bus.i2c_rdwr(r)
                return list(r)[0] & 0xFF
            except OSError as e:
                if e.errno == errno.ENXIO:
                    time.sleep(0.003 * (attempt + 1))
                    continue
                raise
        raise OSError(errno.ENXIO, "NACK while reading register with STOP")

    def write_reg(self, reg: int, val: int):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, [reg & 0xFF, val & 0xFF]))

    def probe(self) -> bool:
        try:
            _ = self.read_reg(REG_FAULT)
            return True
        except OSError:
            return False

    def enable_output(self, enable: bool):
        c0 = self.read_reg(REG_CONFIG0)
        c0 = (c0 | CFG0_EN_OUT) if enable else (c0 & ~CFG0_EN_OUT)
        self.write_reg(REG_CONFIG0, c0)

    def set_bridge_control_i2c(self, use_i2c: bool):
        c4 = self.read_reg(REG_CONFIG4)
        c4 = (c4 | CFG4_I2C_BC) if use_i2c else (c4 & ~CFG4_I2C_BC)
        self.write_reg(REG_CONFIG4, c4)

    def set_pwm_mode(self, pwm_mode: bool):
        c4 = self.read_reg(REG_CONFIG4)
        c4 = (c4 | CFG4_PMODE) if pwm_mode else (c4 & ~CFG4_PMODE)
        self.write_reg(REG_CONFIG4, c4)

    def set_control_mode_phen(self):
        # I2C bridge control + PH/EN mode
        self.enable_output(False)
        time.sleep(0.002)
        self.set_bridge_control_i2c(True)
        self.set_pwm_mode(False)   # PH/EN
        time.sleep(0.002)
        self.enable_output(True)
        time.sleep(0.005)

    def clear_faults(self):
        c0 = self.read_reg(REG_CONFIG0)
        self.write_reg(REG_CONFIG0, c0 | CFG0_CLR_FLT)
        time.sleep(0.001)

    def set_ovp_enabled(self, enable: bool):
        c0 = self.read_reg(REG_CONFIG0)
        if enable:
            c0 |= (1 << 6)   # EN_OVP
        else:
            c0 &= ~(1 << 6)
        self.write_reg(REG_CONFIG0, c0)

    def get_fault(self) -> int:
        return self.read_reg_stop(REG_FAULT)

    def configure_current_regulation(self, imode: int = 0b10, use_int_vref: bool = False, reg_ctrl: int = 0b00):
        # IMODE=10b => current regulation enabled at all times.
        was_enabled = bool(self.read_reg(REG_CONFIG0) & CFG0_EN_OUT)

        self.enable_output(False)
        time.sleep(0.002)

        c3 = self.read_reg(REG_CONFIG3)
        c3 = (c3 & ~0xC0) | ((imode & 0x3) << 6)
        if use_int_vref:
            c3 |= (1 << 4)
        else:
            c3 &= ~(1 << 4)
        c3 |= (1 << 5)  # SMODE=1 (indication only)
        self.write_reg(REG_CONFIG3, c3)

        rc0 = self.read_reg(REG_REG_CTRL0)
        rc0 = (rc0 & ~(0x3 << 3)) | ((reg_ctrl & 0x3) << 3)
        self.write_reg(REG_REG_CTRL0, rc0)

        if was_enabled:
            self.enable_output(True)
            time.sleep(0.005)

    # PH/EN (I2C) control
    def _write_phen(self, en: int, ph: int):
        c4 = self.read_reg(REG_CONFIG4)
        c4 &= ~(CFG4_I2C_EN1 | CFG4_I2C_PH2)
        if en: c4 |= CFG4_I2C_EN1
        if ph: c4 |= CFG4_I2C_PH2
        self.write_reg(REG_CONFIG4, c4)

    def drive(self, forward: bool, enable: bool):
        self._write_phen(1 if enable else 0, 1 if forward else 0)

    def brake(self):
        # EN low => outputs Hi-Z (coast)
        self._write_phen(0, 0)

# ======================================================
# Runtime state
# ======================================================
@dataclass
class MotorChan:
    dev: DRV8234
    dac: Optional[MCP4725]
    have_dac: bool
    run_ilimit_a: float
    stall_ilimit_a: float

    vel_cmd: float = 0.0          # [-1..+1]
    last_cmd_ts: float = 0.0
    stall_mode: bool = False

@dataclass
class HWState:
    bus: SMBus
    i2c_lock: threading.Lock
    motor_lock: Dict[int, threading.Lock]
    m: Dict[int, MotorChan]
    args: argparse.Namespace

def _fault_summary(f: int) -> Dict[str, bool]:
    return {
        "fault_active": bool(f & FLT_ACTIVE),
        "stall": bool(f & FLT_STALL),
        "ocp": bool(f & FLT_OCP),
        "ovp": bool(f & FLT_OVP),
        "tsd": bool(f & FLT_TSD),
        "npor": bool(f & FLT_NPOR),
    }

def _set_limit(m: MotorChan, amps: float, ripropi: float, aipropi: float) -> Dict[str, Any]:
    if not m.have_dac or not m.dac:
        return {"ok": False, "error": "no DAC present"}
    vref, itrip = m.dac.set_current_limit(amps, ripropi, aipropi)
    return {"ok": True, "vref_v": vref, "itrip_a": itrip, "dac_code": m.dac.last_code}

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def _parse_vel(val) -> float:
    """
    Accept:
      - float in [-1..1]
      - int/float in [-100..100] meaning percent
    """
    v = float(val)
    if abs(v) > 1.5:
        v = v / 100.0
    return float(_clamp(v, -1.0, 1.0))

def set_velocity(state: HWState, motor_n: int, vel: float):
    if motor_n not in state.m:
        raise ValueError(f"invalid motor {motor_n}")
    vel = _parse_vel(vel)
    mc = state.m[motor_n]
    with state.motor_lock[motor_n]:
        mc.vel_cmd = vel
        mc.last_cmd_ts = time.monotonic()

def brake_motor(state: HWState, motor_n: Optional[int] = None):
    with state.i2c_lock:
        if motor_n is None:
            for n in (1, 2):
                state.m[n].dev.brake()
                state.m[n].vel_cmd = 0.0
        else:
            state.m[motor_n].dev.brake()
            state.m[motor_n].vel_cmd = 0.0

# ======================================================
# Motor control threads (software PWM over I2C PH/EN bits)
# ======================================================
def motor_pwm_loop(state: HWState, motor_n: int):
    mc = state.m[motor_n]
    dev = mc.dev

    period_s = 1.0 / float(state.args.pwm_hz)
    min_on_s = 0.0008  # avoid ultra-short sleeps that just jitter on Linux
    fault_poll_s = float(state.args.fault_poll_s)
    last_fault_poll = 0.0

    while True:
        now = time.monotonic()

        with state.motor_lock[motor_n]:
            v = float(mc.vel_cmd)
            age = now - float(mc.last_cmd_ts)

        # deadman: if base stops updating, brake
        if age > float(state.args.cmd_timeout_s):
            v = 0.0

        # deadzone
        if abs(v) < float(state.args.deadzone):
            with state.i2c_lock:
                dev.brake()
            time.sleep(period_s)
            continue

        forward = (v >= 0.0)
        duty = float(_clamp(abs(v), 0.0, float(state.args.max_duty)))

        # fault polling + optional stall current-limit bump
        if (now - last_fault_poll) >= fault_poll_s:
            last_fault_poll = now
            with state.i2c_lock:
                f = dev.get_fault()

                # bump current limit when stall bit asserts (optional)
                if state.args.use_stall_limit and (f & FLT_STALL):
                    if mc.have_dac and mc.dac and not mc.stall_mode:
                        _set_limit(mc, mc.stall_ilimit_a, state.args.ripropi, state.args.aipropi)
                        mc.stall_mode = True
                    dev.clear_faults()
                else:
                    if mc.have_dac and mc.dac and mc.stall_mode:
                        _set_limit(mc, mc.run_ilimit_a, state.args.ripropi, state.args.aipropi)
                        mc.stall_mode = False

                # recoverable faults: clear and keep going
                if (f & (FLT_OCP | FLT_OVP)) != 0:
                    dev.clear_faults()

                # hard faults: brake until next command
                if (f & (FLT_TSD | FLT_NPOR)) != 0:
                    dev.brake()
                    with state.motor_lock[motor_n]:
                        mc.vel_cmd = 0.0
                    time.sleep(period_s)
                    continue

        on_s = max(min_on_s, duty * period_s)
        off_s = max(0.0, period_s - on_s)

        with state.i2c_lock:
            dev.drive(forward=forward, enable=True)
            dev.last_dir = forward
        time.sleep(on_s)

        # Off-time: disable outputs (Hi-Z / coast)
        with state.i2c_lock:
            dev.drive(forward=forward, enable=False)
        if off_s > 0:
            time.sleep(off_s)

# ======================================================
# Commands (/cmd + CLI)
# ======================================================
def exec_text_command(state: HWState, line: str) -> Dict[str, Any]:
    try:
        tokens = (line or "").strip().split()
        if not tokens:
            return {"ok": False, "error": "empty command"}
        cmd = tokens[0].lower()

        if cmd in ("help", "?"):
            return {"ok": True, "help": True}
        if cmd in ("quit", "exit"):
            return {"ok": True, "exit": True}

        # velocity commands
        if cmd in ("v1", "v2"):
            if len(tokens) < 2:
                return {"ok": False, "error": f"usage: {cmd} <vel [-1..1] or % [-100..100]>"}
            motor_n = 1 if cmd == "v1" else 2
            set_velocity(state, motor_n, float(tokens[1]))
            return {"ok": True, "cmd": cmd, "motor": motor_n, "vel": _parse_vel(tokens[1])}

        if cmd in ("brake1", "brake2", "brakeall", "stop1", "stop2", "stopall", "brake"):
            with state.i2c_lock:
                if cmd.endswith("all") or cmd == "brake":
                    brake_motor(state, None)
                elif cmd.endswith("1"):
                    brake_motor(state, 1)
                else:
                    brake_motor(state, 2)
            return {"ok": True, "cmd": cmd}

        if cmd in ("status1", "status2", "statusall"):
            out = {"ok": True, "motors": {}}
            with state.i2c_lock:
                for n in (1, 2):
                    if cmd != "statusall" and ((cmd.endswith("1") and n != 1) or (cmd.endswith("2") and n != 2)):
                        continue
                    dev = state.m[n].dev
                    f = dev.get_fault()
                    out["motors"][f"m{n}"] = {
                        "addr": dev.addr,
                        "fault": int(f),
                        "fault_flags": _fault_summary(int(f)),
                    }
            return out

        if cmd in ("ilimit1", "ilimit2", "istall1", "istall2"):
            if len(tokens) < 2:
                return {"ok": False, "error": f"usage: {cmd} <amps>"}
            amps = float(tokens[1])
            motor_n = 1 if cmd.endswith("1") else 2
            mc = state.m[motor_n]
            if cmd.startswith("ilimit"):
                mc.run_ilimit_a = amps
            else:
                mc.stall_ilimit_a = amps
            with state.i2c_lock:
                applied = _set_limit(mc, amps, state.args.ripropi, state.args.aipropi)
            return {"ok": True, "cmd": cmd, "motor": motor_n, "amps": amps, "applied": applied}

        return {"ok": False, "error": f"unknown command: {cmd}"}
    except Exception as e:
        return {"ok": False, "error": str(e)}

def print_help():
    print("\nCommands:")
    print("  v1 <vel> | v2 <vel>          -> set motor velocity (vel in [-1..1] or % [-100..100])")
    print("  brake1 | brake2 | brakeall   -> brake/coast")
    print("  status1 | status2 | statusall")
    print("  ilimit1 <A> | ilimit2 <A>    -> set RUN current limit")
    print("  istall1 <A> | istall2 <A>    -> set STALL current limit")
    print("  help | quit")

# ======================================================
# HTTP server
# ======================================================
def build_app(state: HWState) -> "web.Application":
    if web is None:
        raise RuntimeError("aiohttp is not installed; cannot run in --server mode")

    async def health(_request: web.Request):
        out = {"ok": True, "bus": state.args.bus, "motors": {}}
        now = time.monotonic()
        with state.i2c_lock:
            for n in (1, 2):
                mc = state.m[n]
                f = mc.dev.get_fault()
                out["motors"][f"m{n}"] = {
                    "drv_addr": mc.dev.addr,
                    "dac_addr": (mc.dac.addr if mc.dac else None),
                    "have_dac": bool(mc.have_dac),
                    "run_ilimit_a": float(mc.run_ilimit_a),
                    "stall_ilimit_a": float(mc.stall_ilimit_a),
                    "vel_cmd": float(mc.vel_cmd),
                    "cmd_age_s": float(now - mc.last_cmd_ts) if mc.last_cmd_ts else None,
                    "fault": int(f),
                    "fault_flags": _fault_summary(int(f)),
                }
        return web.json_response(out)

    async def handle_cmd(request: web.Request):
        try:
            data = await request.json()
        except Exception:
            data = {}
        cmd = (data.get("cmd") or "").strip()
        if not cmd:
            return web.json_response({"ok": False, "error": "missing 'cmd' in JSON"}, status=200)
        res = await asyncio.to_thread(exec_text_command, state, cmd)
        return web.json_response(res, status=200)

    async def handle_motor(request: web.Request):
        """
        Velocity endpoint (kept as /motor for compatibility):
          POST /motor  {"m1": 0.25, "m2": -0.10}
          POST /motor  {"m1": 25, "m2": -10}     # percent
        """
        try:
            data = await request.json()
        except Exception:
            data = {}

        changed = {}
        if "m1" in data:
            set_velocity(state, 1, data["m1"])
            changed["m1"] = _parse_vel(data["m1"])
        if "m2" in data:
            set_velocity(state, 2, data["m2"])
            changed["m2"] = _parse_vel(data["m2"])

        if not changed:
            return web.json_response({"ok": False, "error": "send JSON like {'m1':0.2} or {'m2':-0.5} or both"}, status=200)

        return web.json_response({"ok": True, "set": changed}, status=200)

    async def handle_brake(request: web.Request):
        try:
            data = await request.json()
        except Exception:
            data = {}
        motor = data.get("motor", None)
        with state.i2c_lock:
            if motor in (1, "1"):
                brake_motor(state, 1)
            elif motor in (2, "2"):
                brake_motor(state, 2)
            else:
                brake_motor(state, None)
        return web.json_response({"ok": True, "braked": motor or "all"}, status=200)

    async def on_cleanup(_app: web.Application):
        try:
            with state.i2c_lock:
                brake_motor(state, None)
        except Exception:
            pass
        try:
            state.bus.close()
        except Exception:
            pass

    app = web.Application()
    app.router.add_get("/health", health)
    app.router.add_post("/cmd", handle_cmd)
    app.router.add_post("/motor", handle_motor)
    app.router.add_post("/brake", handle_brake)
    app.on_cleanup.append(on_cleanup)
    return app

# ======================================================
# Init + main
# ======================================================
def setup_driver(dev: DRV8234, reg_ctrl: int, disable_ovp: bool = True):
    dev.set_control_mode_phen()
    dev.clear_faults()
    if disable_ovp:
        dev.set_ovp_enabled(False)
        dev.clear_faults()
    dev.configure_current_regulation(imode=0b10, use_int_vref=False, reg_ctrl=reg_ctrl)
    dev.brake()

def init_state(args: argparse.Namespace) -> HWState:
    bus = SMBus(args.bus)
    i2c_lock = threading.Lock()
    motor_lock = {1: threading.Lock(), 2: threading.Lock()}

    dev1 = DRV8234(bus, args.addr1, "M1")
    dev2 = DRV8234(bus, args.addr2, "M2")
    dac1 = MCP4725(bus, args.dac1_addr, vdd=args.dac_vdd, name="DAC1")
    dac2 = MCP4725(bus, args.dac2_addr, vdd=args.dac_vdd, name="DAC2")

    with i2c_lock:
        if not dev1.probe():
            raise RuntimeError("No ACK from M1 DRV8234")
        if not dev2.probe():
            raise RuntimeError("No ACK from M2 DRV8234")

        have_dac1 = dac1.probe()
        have_dac2 = dac2.probe()

        reg_ctrl = 1 if args.cbc else 0
        setup_driver(dev1, reg_ctrl=reg_ctrl, disable_ovp=True)
        setup_driver(dev2, reg_ctrl=reg_ctrl, disable_ovp=True)

    m = {
        1: MotorChan(dev=dev1, dac=dac1 if have_dac1 else None, have_dac=have_dac1,
                    run_ilimit_a=float(args.ilimit1), stall_ilimit_a=float(args.stall_ilimit1)),
        2: MotorChan(dev=dev2, dac=dac2 if have_dac2 else None, have_dac=have_dac2,
                    run_ilimit_a=float(args.ilimit2), stall_ilimit_a=float(args.stall_ilimit2)),
    }

    state = HWState(bus=bus, i2c_lock=i2c_lock, motor_lock=motor_lock, m=m, args=args)

    with i2c_lock:
        for n in (1, 2):
            if state.m[n].have_dac and state.m[n].dac:
                _set_limit(state.m[n], state.m[n].run_ilimit_a, args.ripropi, args.aipropi)

    # start PWM loops
    threading.Thread(target=motor_pwm_loop, args=(state, 1), daemon=True).start()
    threading.Thread(target=motor_pwm_loop, args=(state, 2), daemon=True).start()

    return state

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=5)

    ap.add_argument("--addr1", type=lambda x: int(x, 0), default=0x30, help="Motor1 DRV8234 addr")
    ap.add_argument("--addr2", type=lambda x: int(x, 0), default=0x31, help="Motor2 DRV8234 addr")

    ap.add_argument("--dac1-addr", dest="dac1_addr", type=lambda x: int(x, 0), default=0x62, help="DAC1 addr (motor1 VREF)")
    ap.add_argument("--dac2-addr", dest="dac2_addr", type=lambda x: int(x, 0), default=0x63, help="DAC2 addr (motor2 VREF)")
    ap.add_argument("--dac-vdd", type=float, default=3.3)

    ap.add_argument("--ripropi", type=float, default=4700.0)
    ap.add_argument("--aipropi", type=float, default=1500.0)

    ap.add_argument("--ilimit1", type=float, default=0.15, help="RUN current limit motor1 (A)")
    ap.add_argument("--ilimit2", type=float, default=0.15, help="RUN current limit motor2 (A)")
    ap.add_argument("--stall-ilimit1", type=float, default=None, help="STALL current limit motor1 (A) (default=ilimit1)")
    ap.add_argument("--stall-ilimit2", type=float, default=None, help="STALL current limit motor2 (A) (default=ilimit2)")

    ap.add_argument("--cbc", action="store_true", help="Cycle-by-cycle current regulation (REG_CTRL=01b)")

    # velocity control tuning
    ap.add_argument("--pwm-hz", type=float, default=120.0, help="software PWM frequency (Hz)")
    ap.add_argument("--deadzone", type=float, default=0.08, help="velocity deadzone (0..1)")
    ap.add_argument("--max-duty", type=float, default=1.0, help="cap duty (0..1)")
    ap.add_argument("--cmd-timeout-s", type=float, default=0.35, help="deadman timeout; brake if no command")
    ap.add_argument("--fault-poll-s", type=float, default=0.10, help="fault poll period")
    ap.add_argument("--use-stall-limit", action="store_true", help="if set, raise DAC current limit on STALL bit")

    ap.add_argument("--server", action="store_true", help="Run HTTP server instead of CLI")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=int(os.getenv("PORT", "8000")))

    args = ap.parse_args()

    if args.stall_ilimit1 is None:
        args.stall_ilimit1 = args.ilimit1
    if args.stall_ilimit2 is None:
        args.stall_ilimit2 = args.ilimit2

    print("\n=== Dual DRV8234 Velocity Server (software PWM over I2C PH/EN) ===")
    print(f"/dev/i2c-{args.bus}")
    print(f"M1 DRV=0x{args.addr1:02X} DAC1=0x{args.dac1_addr:02X} | M2 DRV=0x{args.addr2:02X} DAC2=0x{args.dac2_addr:02X}")
    print(f"RUN limits:   M1={args.ilimit1}A  M2={args.ilimit2}A")
    print(f"STALL limits: M1={args.stall_ilimit1}A  M2={args.stall_ilimit2}A")
    print(f"pwm_hz={args.pwm_hz}  deadzone={args.deadzone}  cmd_timeout_s={args.cmd_timeout_s}\n")

    try:
        state = init_state(args)
    except Exception as e:
        print(f"[FATAL] Hardware init failed: {e}", file=sys.stderr)
        sys.exit(1)

    if args.server:
        if web is None:
            print("[FATAL] aiohttp is required for --server mode.", file=sys.stderr)
            sys.exit(1)
        app = build_app(state)
        print(f"[HTTP] Listening on http://{args.host}:{args.port}  endpoints: /health /motor /cmd /brake")
        web.run_app(app, host=args.host, port=args.port)
        return

    # CLI mode
    print_help()
    try:
        while True:
            line = input("> ")
            res = exec_text_command(state, line)
            if res.get("exit"):
                break
            if res.get("help"):
                print_help()
                continue
            print(json.dumps(res, indent=2))
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try:
            with state.i2c_lock:
                brake_motor(state, None)
        except Exception:
            pass
        try:
            state.bus.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()