#!/usr/bin/env python3
import time
import errno
import os
import json
import asyncio
import threading
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

REG_RC_STATUS1 = 0x01  # SPEED[7:0]
REG_RC_STATUS2 = 0x02  # RC_CNT[7:0]
REG_RC_STATUS3 = 0x03  # RC_CNT[15:8]

REG_RC_CTRL0   = 0x11  # EN_RC (bit7)
REG_RC_CTRL1   = 0x12
REG_RC_CTRL2   = 0x13
REG_RC_CTRL3   = 0x14

# --- Bit masks ---
CFG0_EN_OUT    = 0x80
CFG0_CLR_FLT   = 0x02
CFG0_CLR_CNT   = 0x04

CFG4_PMODE     = 0x08  # writable only when EN_OUT=0
CFG4_I2C_BC    = 0x04  # writable only when EN_OUT=0
CFG4_I2C_EN1   = 0x02
CFG4_I2C_PH2   = 0x01

DEFAULT_COUNTS = 2000
MOVE_TIMEOUT_S = 8.0

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

    def set_current_limit(self, amps: float, ripropi_ohm: float, aipropi_uA_per_A: float) -> tuple[float, float]:
        # VVREF = ITRIP * AIPROPI * RIPROPI
        amps = float(amps)
        vref_cmd = amps * (aipropi_uA_per_A * 1e-6) * ripropi_ohm
        vref_act = self.set_voltage(vref_cmd)
        itrip_act = (vref_act / ripropi_ohm) / (aipropi_uA_per_A * 1e-6)
        return vref_act, itrip_act

class DRV8234:
    def __init__(self, bus: SMBus, addr: int, name: str):
        self.bus = bus
        self.addr = addr
        self.name = name
        self.last_dir = None

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

    # Starred bits: must write with EN_OUT=0
    def set_bridge_control_i2c(self, use_i2c: bool):
        c4 = self.read_reg(REG_CONFIG4)
        c4 = (c4 | CFG4_I2C_BC) if use_i2c else (c4 & ~CFG4_I2C_BC)
        self.write_reg(REG_CONFIG4, c4)

    def set_pwm_mode(self, pwm_mode: bool):
        c4 = self.read_reg(REG_CONFIG4)
        c4 = (c4 | CFG4_PMODE) if pwm_mode else (c4 & ~CFG4_PMODE)
        self.write_reg(REG_CONFIG4, c4)

    def set_control_mode_phen(self):
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

    def print_status(self):
        f = self.get_fault()
        print(f"[{self.name}] Fault Register: 0x{f:02X}")
        if f & 0x80: print("  FAULT active")
        if f & 0x20: print("  Motor STALL detected")
        if f & 0x10: print("  OCP - Overcurrent")
        if f & 0x08: print("  OVP - Overvoltage")
        if f & 0x04: print("  TSD - Thermal shutdown")
        if f & 0x02: print("  NPOR - Power-on reset")

    # PH/EN commands
    def _write_phen(self, en: int, ph: int):
        c4 = self.read_reg(REG_CONFIG4)
        c4 &= ~(CFG4_I2C_EN1 | CFG4_I2C_PH2)
        if en: c4 |= CFG4_I2C_EN1
        if ph: c4 |= CFG4_I2C_PH2
        self.write_reg(REG_CONFIG4, c4)

    def motor_forward(self): self._write_phen(1, 1)
    def motor_reverse(self): self._write_phen(1, 0)
    def motor_brake(self):   self._write_phen(0, 0)

    def configure_current_regulation(self, imode: int = 0b10, use_int_vref: bool = False, reg_ctrl: int = 0b00):
        """
        IMODE=10b => current regulation enabled at all times.
        Force SMODE=1 (indication only) so it doesn't shut off outputs.
        REG_CTRL:
          00b Fixed Off-Time, 01b Cycle-By-Cycle
        """
        was_enabled = bool(self.read_reg(REG_CONFIG0) & CFG0_EN_OUT)

        self.enable_output(False)
        time.sleep(0.002)

        c3 = self.read_reg(REG_CONFIG3)
        c3 = (c3 & ~0xC0) | ((imode & 0x3) << 6)
        if use_int_vref:
            c3 |= (1 << 4)
        else:
            c3 &= ~(1 << 4)
        c3 |= (1 << 5)  # SMODE=1
        self.write_reg(REG_CONFIG3, c3)

        rc0 = self.read_reg(REG_REG_CTRL0)
        rc0 = (rc0 & ~(0x3 << 3)) | ((reg_ctrl & 0x3) << 3)
        self.write_reg(REG_REG_CTRL0, rc0)

        if was_enabled:
            self.enable_output(True)
            time.sleep(0.005)

    def enable_ripple(self, enable: bool = True, inv_r: int = 1):
        rc0 = self.read_reg(REG_RC_CTRL0)
        rc0 = (rc0 | 0x80) if enable else (rc0 & ~0x80)
        self.write_reg(REG_RC_CTRL0, rc0)
        if not enable:
            return
        self.write_reg(REG_RC_CTRL1, 0xFF)
        rc2 = self.read_reg(REG_RC_CTRL2)
        rc2 &= ~0x0F
        rc2 |= (0x03 << 2) | 0x03
        self.write_reg(REG_RC_CTRL2, rc2)
        cur_inv = self.read_reg(REG_RC_CTRL3)
        if cur_inv == 0:
            self.write_reg(REG_RC_CTRL3, inv_r if inv_r else 1)

    def read_speed(self) -> int:
        return self.read_reg(REG_RC_STATUS1)

    def read_count16(self) -> int:
        hi1 = self.read_reg(REG_RC_STATUS3)
        lo  = self.read_reg(REG_RC_STATUS2)
        hi2 = self.read_reg(REG_RC_STATUS3)
        if hi1 != hi2:
            lo  = self.read_reg(REG_RC_STATUS2)
            hi1 = hi2
        return ((hi1 << 8) | lo) & 0xFFFF

    def read_count16_stable(self) -> int:
        return self.read_count16()

    def clear_count(self):
        c0 = self.read_reg(REG_CONFIG0)
        self.write_reg(REG_CONFIG0, c0 | CFG0_CLR_CNT)
        time.sleep(0.001)


# ------------------ Fault bits (from existing print_status mapping) ------------------
FLT_ACTIVE = 0x80
FLT_STALL  = 0x20
FLT_OCP    = 0x10
FLT_OVP    = 0x08
FLT_TSD    = 0x04
FLT_NPOR   = 0x02

# Server defaults (can be overridden per-request)
MOVE_TIMEOUT_S_DEFAULT = MOVE_TIMEOUT_S
STALL_NO_PROGRESS_S_DEFAULT = 0.35
FAULT_POLL_S = 0.05
RECOVERABLE_FAULT_MAX = 30

# ======================================================
# Runtime state
# ======================================================
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

@dataclass
class MotorChan:
    dev: DRV8234
    dac: Optional[MCP4725]
    have_dac: bool
    run_ilimit_a: float
    stall_ilimit_a: float
    last_result: Optional[Dict[str, Any]] = None

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

def move_relative_counts(
    state: HWState,
    motor_n: int,
    signed_counts: int,
    *,
    timeout_s: float = MOVE_TIMEOUT_S_DEFAULT,
    no_progress_s: float = STALL_NO_PROGRESS_S_DEFAULT,
    hold_on_stall: bool = True,
    stall_hold_max_s: float = 2.0,
) -> Dict[str, Any]:
    """
    Stall-friendly blocking move.
    - Move by ripple-counter counts (RC_CNT).
    - If stall is detected (FAULT.STALL bit OR no progress for `no_progress_s`),
      set current limit to configured STALL limit and keep driving (current-regulated)
      without throwing / crashing.
    - Recoverable faults (OCP/OVP): clear + keep going up to RECOVERABLE_FAULT_MAX.
    - Hard faults (TSD/NPOR): brake and return.
    """
    if motor_n not in state.m:
        return {"ok": False, "error": f"invalid motor {motor_n}"}

    m = state.m[motor_n]
    dev = m.dev

    signed_counts = int(signed_counts)
    if signed_counts == 0:
        return {"ok": True, "motor": motor_n, "cmd_counts": 0, "note": "no-op"}

    forward = signed_counts > 0
    target = abs(signed_counts) & 0xFFFF

    faults_seen = []
    recoverable_faults = 0

    with state.i2c_lock:
        dev.set_control_mode_phen()
        dev.clear_faults()
        dev.clear_count()

        if dev.last_dir is not None and dev.last_dir != forward:
            dev.motor_brake()
            time.sleep(0.25)

        # Set RUN limit for the move
        if m.have_dac and m.dac:
            _set_limit(m, m.run_ilimit_a, state.args.ripropi, state.args.aipropi)

        dev.motor_forward() if forward else dev.motor_reverse()
        time.sleep(0.05)

    t0 = time.monotonic()
    last_fault_t = 0.0
    last_count = 0
    last_progress_t = time.monotonic()
    stalled = False
    stall_enter_t: Optional[float] = None
    max_seen = 0

    while True:
        now_t = time.monotonic()
        if now_t - t0 > float(timeout_s):
            break

        with state.i2c_lock:
            if now_t - last_fault_t >= FAULT_POLL_S:
                f = dev.get_fault()
                faults_seen.append(int(f))
                last_fault_t = now_t

                # If driver flags stall, go into stall mode (raise to stall limit)
                if (f & FLT_STALL) and not stalled:
                    stalled = True
                    stall_enter_t = now_t
                    if m.have_dac and m.dac:
                        _set_limit(m, m.stall_ilimit_a, state.args.ripropi, state.args.aipropi)
                    dev.clear_faults()

                # Recoverable faults: clear and keep going (no process-crash / no "hard error")
                if (f & (FLT_OCP | FLT_OVP)) != 0:
                    recoverable_faults += 1
                    dev.clear_faults()
                    dev.configure_current_regulation(imode=0b10, use_int_vref=False,
                                                     reg_ctrl=(1 if state.args.cbc else 0))
                    if recoverable_faults > RECOVERABLE_FAULT_MAX:
                        stalled = True
                        if stall_enter_t is None:
                            stall_enter_t = now_t
                        hold_on_stall = False

                # Hard faults: brake
                if (f & (FLT_TSD | FLT_NPOR)) != 0:
                    stalled = True
                    if stall_enter_t is None:
                        stall_enter_t = now_t
                    hold_on_stall = False

            cnt = dev.read_count16_stable()
            if cnt > max_seen:
                max_seen = cnt

        # No-progress stall detector
        if cnt != last_count:
            last_count = cnt
            last_progress_t = now_t
        elif not stalled and (now_t - last_progress_t) >= float(no_progress_s):
            stalled = True
            stall_enter_t = now_t
            with state.i2c_lock:
                if m.have_dac and m.dac:
                    _set_limit(m, m.stall_ilimit_a, state.args.ripropi, state.args.aipropi)
                dev.clear_faults()

        if max_seen >= target:
            break

        if stalled and not hold_on_stall:
            break

        if stalled and stall_enter_t is not None and stall_hold_max_s is not None:
            if float(stall_hold_max_s) > 0 and (now_t - stall_enter_t) >= float(stall_hold_max_s):
                break

        time.sleep(0.02)

    with state.i2c_lock:
        dev.motor_brake()
        time.sleep(0.010)
        final_cnt = dev.read_count16_stable()
        dev.last_dir = forward
        final_fault = dev.get_fault()
        dev.clear_faults()

    result = {
        "ok": True,
        "motor": motor_n,
        "cmd_counts": signed_counts,
        "target_counts": int(target),
        "dir": "FWD" if forward else "REV",
        "max_seen": int(max_seen),
        "final_cnt": int(final_cnt),
        "stalled": bool(stalled),
        "fault": int(final_fault),
        "fault_flags": _fault_summary(int(final_fault)),
        "recoverable_faults": int(recoverable_faults),
    }
    m.last_result = result
    return result

# ======================================================
# Shared command parser (CLI + HTTP)
# ======================================================
def print_help():
    print("\nCommands:")
    print("  m1 <signed_counts>        -> move motor1 (negative = reverse)")
    print("  m2 <signed_counts>        -> move motor2 (negative = reverse)")
    print("  brake1 | brake2 | brakeall")
    print("  status1 | status2 | statusall")
    print("  ilimit1 <A> | ilimit2 <A> -> set RUN current limit")
    print("  istall1 <A> | istall2 <A> -> set STALL current limit")
    print("  help | quit")

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

        if cmd in ("m1", "m2"):
            if len(tokens) < 2:
                return {"ok": False, "error": f"usage: {cmd} <signed_counts>"}
            motor_n = 1 if cmd == "m1" else 2
            signed_counts = int(tokens[1], 10)
            return move_relative_counts(state, motor_n, signed_counts)

        if cmd in ("brake1", "brake2", "brakeall", "stop1", "stop2", "stopall"):
            with state.i2c_lock:
                if cmd.endswith("all"):
                    state.m[1].dev.motor_brake()
                    state.m[2].dev.motor_brake()
                elif cmd.endswith("1"):
                    state.m[1].dev.motor_brake()
                else:
                    state.m[2].dev.motor_brake()
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
                        "rc_cnt": int(dev.read_count16_stable()),
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

# ======================================================
# HTTP server
# ======================================================
def build_app(state: HWState) -> "web.Application":
    if web is None:
        raise RuntimeError("aiohttp is not installed; cannot run in --server mode")

    async def health(_request: web.Request):
        with state.i2c_lock:
            m1 = state.m[1]
            m2 = state.m[2]
            out = {
                "ok": True,
                "bus": state.args.bus,
                "motors": {
                    "m1": {
                        "drv_addr": m1.dev.addr,
                        "dac_addr": (m1.dac.addr if m1.dac else None),
                        "have_dac": bool(m1.have_dac),
                        "run_ilimit_a": float(m1.run_ilimit_a),
                        "stall_ilimit_a": float(m1.stall_ilimit_a),
                        "fault": int(m1.dev.get_fault()),
                        "rc_cnt": int(m1.dev.read_count16_stable()),
                    },
                    "m2": {
                        "drv_addr": m2.dev.addr,
                        "dac_addr": (m2.dac.addr if m2.dac else None),
                        "have_dac": bool(m2.have_dac),
                        "run_ilimit_a": float(m2.run_ilimit_a),
                        "stall_ilimit_a": float(m2.stall_ilimit_a),
                        "fault": int(m2.dev.get_fault()),
                        "rc_cnt": int(m2.dev.read_count16_stable()),
                    },
                },
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
        try:
            data = await request.json()
        except Exception:
            data = {}

        timeout_s = float(data.get("timeout_s", MOVE_TIMEOUT_S_DEFAULT))
        hold_on_stall = bool(data.get("hold_on_stall", True))
        stall_hold_max_s = float(data.get("stall_hold_max_s", 2.0))
        no_progress_s = float(data.get("no_progress_s", STALL_NO_PROGRESS_S_DEFAULT))

        cmds = []
        if "m1" in data:
            cmds.append((1, int(data["m1"])))
        if "m2" in data:
            cmds.append((2, int(data["m2"])))
        if not cmds:
            return web.json_response({"ok": False, "error": "send JSON like {'m1':2000} or {'m2':-500}"}, status=200)

        results = {}
        for n, sc in cmds:
            with state.motor_lock[n]:
                r = await asyncio.to_thread(
                    move_relative_counts, state, n, sc,
                    timeout_s=timeout_s,
                    no_progress_s=no_progress_s,
                    hold_on_stall=hold_on_stall,
                    stall_hold_max_s=stall_hold_max_s,
                )
            results[f"m{n}"] = r

        return web.json_response({"ok": True, "results": results}, status=200)

    async def on_cleanup(_app: web.Application):
        try:
            with state.i2c_lock:
                state.m[1].dev.motor_brake()
                state.m[2].dev.motor_brake()
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
    dev.enable_ripple(True, inv_r=1)
    dev.clear_count()

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

    with i2c_lock:
        if m[1].have_dac and m[1].dac:
            _set_limit(m[1], m[1].run_ilimit_a, args.ripropi, args.aipropi)
        if m[2].have_dac and m[2].dac:
            _set_limit(m[2], m[2].run_ilimit_a, args.ripropi, args.aipropi)

    return HWState(bus=bus, i2c_lock=i2c_lock, motor_lock=motor_lock, m=m, args=args)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bus", type=int, default=5)

    ap.add_argument("--addr1", type=lambda x: int(x, 0), default=0x32, help="Motor1 DRV8234 addr")
    ap.add_argument("--addr2", type=lambda x: int(x, 0), default=0x30, help="Motor2 DRV8234 addr")

    ap.add_argument("--dac1-addr", dest="dac1_addr", type=lambda x: int(x, 0), default=0x63, help="DAC1 addr (motor1 VREF)")
    ap.add_argument("--dac2-addr", dest="dac2_addr", type=lambda x: int(x, 0), default=0x62, help="DAC2 addr (motor2 VREF)")
    ap.add_argument("--dac-vdd", type=float, default=3.3)

    ap.add_argument("--ripropi", type=float, default=4700.0)
    ap.add_argument("--aipropi", type=float, default=1500.0)

    ap.add_argument("--ilimit1", type=float, default=0.25, help="RUN current limit motor1 (A)")
    ap.add_argument("--ilimit2", type=float, default=0.25, help="RUN current limit motor2 (A)")
    ap.add_argument("--stall-ilimit1", type=float, default=None, help="STALL current limit motor1 (A) (default=ilimit1)")
    ap.add_argument("--stall-ilimit2", type=float, default=None, help="STALL current limit motor2 (A) (default=ilimit2)")

    ap.add_argument("--cbc", action="store_true", help="Cycle-by-cycle current regulation (REG_CTRL=01b)")
    ap.add_argument("--server", action="store_true", help="Run HTTP server instead of CLI")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=int(os.getenv("PORT", "8000")))

    args = ap.parse_args()

    if args.stall_ilimit1 is None:
        args.stall_ilimit1 = args.ilimit1
    if args.stall_ilimit2 is None:
        args.stall_ilimit2 = args.ilimit2

    print("\n=== Dual DRV8234 + Dual DAC Current Limit (independent) ===")
    print(f"/dev/i2c-{args.bus}")
    print(f"M1 DRV=0x{args.addr1:02X} DAC1=0x{args.dac1_addr:02X} | M2 DRV=0x{args.addr2:02X} DAC2=0x{args.dac2_addr:02X}")
    print(f"RUN limits:   M1={args.ilimit1}A  M2={args.ilimit2}A")
    print(f"STALL limits: M1={args.stall_ilimit1}A  M2={args.stall_ilimit2}A")

    try:
        state = init_state(args)
    except Exception as e:
        print(f"[FATAL] Hardware init failed: {e}", file=sys.stderr)
        sys.exit(1)

    if args.server:
        if web is None:
            print("[FATAL] aiohttp is required for --server mode (it is present on your servo_server system).", file=sys.stderr)
            sys.exit(1)
        app = build_app(state)
        print(f"[HTTP] Listening on http://{args.host}:{args.port}  endpoints: /health /motor /cmd")
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
                state.m[1].dev.motor_brake()
                state.m[2].dev.motor_brake()
        except Exception:
            pass
        try:
            state.bus.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
