"""
conftest.py
-----------
A fake pyrtlsdr, for the tests that need an RTL-SDR behind SdrSnrMetric.

The fake_rtlsdr fixture puts a stand-in `rtlsdr` module in sys.modules, so
iq_snr's `from rtlsdr import RtlSdr` gets FakeRtlSdr whether or not pyrtlsdr
is installed. It records every device opened, every setting, every read and
every close. It knows nothing about the real dongle -- its timing, its
driver, what its samples look like -- so it checks the call sequence only.
"""

import cmath
import math
import random
import sys
import types

import pytest


def qpsk_block(snr_db=10.0, n=8192, seed=1):
    """Constant-modulus QPSK plus complex Gaussian noise of unit power."""
    rng = random.Random(seed)
    amp = math.sqrt(10 ** (snr_db / 10))
    return [amp * cmath.exp(1j * (math.pi / 4 + math.pi / 2 * rng.randrange(4)))
            + complex(rng.gauss(0, 1), rng.gauss(0, 1)) / math.sqrt(2)
            for _ in range(n)]


class FakeDongle:
    """One test's fake hardware: what was opened, and what reads return."""

    def __init__(self):
        self.devices = []           # every FakeRtlSdr opened, in order
        self.block = qpsk_block()   # what a read returns; or a callable for it
        self.raise_on_read = {}     # read number, 1-based over all devices -> exception
        self.raise_on_setting = {}  # setting name -> exception
        self.reads = 0
        self._signals = {}

    def signal(self, snr_db):
        """A QPSK block at snr_db (to 0.1 dB), cached so a sweep reuses it."""
        key = round(snr_db, 1)
        if key not in self._signals:
            self._signals[key] = qpsk_block(key)
        return self._signals[key]

    @property
    def still_open(self):
        return [d for d in self.devices if not d.closed]


class FakeRtlSdr:
    def __init__(self, dongle, device_index=0):
        self.dongle = dongle
        self.device_index = device_index
        self.settings = []          # (name, value), in the order set
        self.reads = []             # num_samples of each read_samples()
        self.closed = False
        dongle.devices.append(self)

    def _setting(name, check=None):
        def get(self):
            return dict(self.settings)[name]

        def set_(self, value):
            exc = self.dongle.raise_on_setting.get(name)
            if exc is not None:
                raise exc
            if check is not None:
                check(value)
            self.settings.append((name, value))
        return property(get, set_)

    def _gain_check(value):
        # pyrtlsdr's set_gain checks for 'auto', then does arithmetic on
        # whatever else it got, so any other string raises this.
        if isinstance(value, str) and value != "auto":
            raise TypeError("unsupported operand type(s) for -: 'str' and 'int'")

    sample_rate = _setting("sample_rate")
    center_freq = _setting("center_freq")
    gain = _setting("gain", _gain_check)
    del _setting, _gain_check

    def read_samples(self, num_samples=1024):
        if self.closed:
            # The real one would hand a freed handle to librtlsdr. pytest.fail
            # is a BaseException, so no `except Exception` can swallow it.
            pytest.fail("read from a closed RtlSdr")
        self.reads.append(num_samples)
        self.dongle.reads += 1
        exc = self.dongle.raise_on_read.get(self.dongle.reads)
        if exc is not None:
            if isinstance(exc, Exception):
                # pyrtlsdr closes the device itself on a failed or short read
                self.closed = True
            raise exc
        block = self.dongle.block
        return block() if callable(block) else block

    def close(self):
        self.closed = True


@pytest.fixture
def fake_rtlsdr(monkeypatch):
    """Swap pyrtlsdr for FakeRtlSdr; returns the FakeDongle to inspect."""
    dongle = FakeDongle()
    module = types.ModuleType("rtlsdr")
    module.RtlSdr = lambda device_index=0: FakeRtlSdr(dongle, device_index)
    monkeypatch.setitem(sys.modules, "rtlsdr", module)
    return dongle
