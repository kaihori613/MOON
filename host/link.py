"""
link.py
-------
Serial transport to the actuator_v1 sketch.

This deliberately speaks the console protocol that already exists rather than
inventing a binary one. Everything this module sends, you could have typed by
hand into the Serial Monitor -- which means when something misbehaves you can
unplug the script, open the monitor, and drive the same commands yourself to
see whether the problem is up here or down there.

Commands used (see actuator_v1.ino):
    s        status
    h        home
    c        calibrate
    g <n>    go to absolute count
    <Enter>  stop
    k        clear fault
"""

from __future__ import annotations

import re
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - import guard for a clearer message
    raise SystemExit(
        "pyserial is not installed.\n"
        "    pip install -r requirements.txt")


BANNER = "Actuator System Bench Console"

# printStatus() emits space separated key=value pairs. 'travel=? (run c)'
# trails a parenthesised hint, which this pattern simply ignores.
_KV = re.compile(r"(\w+)=(\S+)")


class ActuatorError(RuntimeError):
    pass


class ActuatorFault(ActuatorError):
    """The controller reported a FAULT. It has already cut the motor."""


class ActuatorLink:
    def __init__(self, port: str, baud: int = 115200, verbose: bool = False):
        self.port = port
        self.baud = baud
        self.verbose = verbose
        self._ser = None
        self.saw_banner = False

    # --- connection --------------------------------------------------------

    def open(self, boot_timeout: float = 6.0):
        self._ser = serial.Serial(self.port, self.baud, timeout=0.2)

        # Most Arduino boards reset when the port opens, so the first thing
        # out is setup()'s banner. Waiting for it means the first real command
        # cannot land in the middle of a reset and get eaten.
        deadline = time.time() + boot_timeout
        while time.time() < deadline:
            line = self._readline()
            if line is None:
                continue
            if BANNER in line:
                self.saw_banner = True
                break

        # A board that does not auto-reset never prints a banner. That is fine
        # -- it is already running -- so carry on either way.
        self._drain()
        return self

    def close(self):
        if self._ser is not None:
            try:
                self.stop()
            except Exception:
                pass
            self._ser.close()
            self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    @staticmethod
    def list_ports():
        return [(p.device, p.description) for p in list_ports.comports()]

    # --- raw io ------------------------------------------------------------

    def _readline(self):
        raw = self._ser.readline()
        if not raw:
            return None
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        if self.verbose and line:
            print(f"    <- {line}")
        return line

    def _write(self, text: str):
        if self.verbose:
            print(f"    -> {text!r}")
        self._ser.write((text + "\n").encode("ascii"))
        self._ser.flush()

    def _drain(self):
        while self._readline() is not None:
            pass

    # --- commands ----------------------------------------------------------

    def status(self, timeout: float = 2.0) -> dict:
        """
        Ask for a status line and parse it.

        Telemetry lines emitted during a move also carry pos= and pwm=, so the
        discriminator for a real status line is 'state=', which only
        printStatus() produces.
        """
        self._write("s")
        deadline = time.time() + timeout

        while time.time() < deadline:
            line = self._readline()
            if line is None:
                continue
            if "state=" not in line:
                continue

            fields = dict(_KV.findall(line))

            # Rev B positions in degrees and prints them with a decimal point,
            # so these must parse as float. Reading them as int would not
            # raise -- it would quietly yield 0 for every position, which is a
            # far worse failure than a crash on a control loop.
            unit = fields.get("unit", "counts")

            # Rev B names the fault in a field; Rev A only ever printed the
            # word FAULT into the line. Prefer the field where it exists.
            if "fault" in fields:
                fault = None if fields["fault"] == "none" else fields["fault"]
            else:
                fault = _fault_of(line)

            return {
                "state": fields.get("state", "?"),
                # pos and target are always printed, so a None here means a
                # mangled line; 0 keeps the display formatting alive rather
                # than crashing the tuning loop over one corrupt read.
                "pos": _as_float(fields.get("pos")) or 0.0,
                "target": _as_float(fields.get("target")) or 0.0,
                "unit": unit,
                "travel": _as_int(fields.get("travel")),   # None when uncalibrated
                "homed": fields.get("homed", "NO") == "yes",
                # Rev A calls it pwm, Rev B calls it duty and signs it.
                "pwm": _as_int(fields.get("pwm")) or 0,
                "duty": _as_int(fields.get("duty")) or 0,
                "encoder_ok": fields.get("encok") == "1",
                "limits": fields.get("lim", ".."),
                "hz": _as_float(fields.get("hz")) or 0.0,
                "at_hard_stop": "[at hard stop]" in line,
                "fault": fault,
                "raw": line,
            }

        raise ActuatorError(
            f"no status from {self.port} within {timeout:.0f}s -- wrong port, "
            "wrong baud, or the sketch is not running")

    def stop(self):
        """Bare newline is the sketch's unconditional panic stop."""
        self._write("")

    def clear_fault(self):
        self._write("k")

    def home(self):
        self._write("h")

    def calibrate(self):
        self._write("c")

    def move_to(self, counts: int):
        """Rev A: 'g' takes reed counts. See move_to_deg for Rev B."""
        self._write(f"g {int(round(counts))}")

    def move_to_deg(self, degrees: float):
        """
        Rev B: 'g' takes an absolute angle in degrees.

        The command letter did not change, only its unit, so the two sketches
        are indistinguishable from the wire until you read a status line --
        which is why actuator_v2 reports unit=deg and this module exposes
        both. Check unit_is_degrees() rather than guessing from the banner.
        """
        self._write(f"g {degrees:.3f}")

    def unit_is_degrees(self) -> bool:
        """True when the attached sketch positions in degrees (Rev B)."""
        return self.status().get("unit") == "deg"

    def set_zero(self):
        """Rev B: call the current position zero. 'c' is kept as an alias."""
        self._write("z")

    def set_gain(self, which: str, value: float):
        """Rev B live tuning: which is one of p, i, d, f."""
        if which not in ("p", "i", "d", "f"):
            raise ValueError(f"gain must be p, i, d or f, not {which!r}")
        self._write(f"{which} {value:.4f}")

    def save_settings(self):
        """Rev B: persist the encoder zero and the gains."""
        self._write("w")

    # --- waiting -----------------------------------------------------------

    def wait_idle(self, timeout: float = 180.0, on_progress=None) -> dict:
        """
        Poll until the controller leaves the busy states.

        Raises ActuatorFault if it faults. The sketch has already cut the motor
        by then; the exception exists so the caller does not keep issuing moves
        into a jammed actuator.
        """
        deadline = time.time() + timeout

        while time.time() < deadline:
            st = self.status()

            if st["fault"]:
                raise ActuatorFault(
                    f"{st['fault']} at pos={st['pos']} -- motor is stopped. "
                    "Clear it with 'k' once you know why.")

            if st["state"] == "IDLE":
                return st

            if on_progress:
                on_progress(st)
            time.sleep(0.2)

        # Do not leave the motor running on our way out.
        self.stop()
        raise ActuatorError(f"move did not finish within {timeout:.0f}s -- motor stopped")


# ---------------------------------------------------------------------------

def _as_int(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _fault_of(line: str):
    if "FAULT" not in line:
        return None
    for name in ("STALL", "TIMEOUT", "NOT_HOMED"):
        if name in line:
            return name
    return "UNKNOWN"
