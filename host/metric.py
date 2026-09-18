"""
metric.py
---------
Where the pointing metric comes from, and how noisy it is.

The step-track search needs one number that gets better as the dish points
better. This module produces that number, and — more importantly — tells you
whether it is stable enough to search with.

WHAT TO USE, IN ORDER OF USEFULNESS

  1. Viterbi corrected-error rate. Steepest against pointing error of
     anything available, because it sits after the demodulator where a
     fraction of a dB of C/N moves it a lot. goesrecv publishes it.
  2. Es/N0 or SNR from the demodulator. Smoother, slower to move, but it does
     not floor at zero the way corrected errors do.
  3. Raw RSSI or total band power. AVOID. It measures the noise as much as
     the signal, it includes ground pickup, and on a beam this wide it barely
     moves with pointing at all. It is the obvious thing to reach for and it
     is the wrong one.

All three need the demodulator locked. Acquire coarsely from geometry.py's
computed look angle first; a search that starts with no signal is climbing a
flat surface and will wander off.

HOW THE NUMBER GETS HERE

goesrecv has a [monitor] section that emits statsd over UDP. statsd is plain
text over a datagram socket, so listening for it costs no dependency and no
polling — goesrecv pushes:

    [monitor]
    statsd_address = "udp://127.0.0.1:8125"

THE METRIC NAMES ARE NOT GUESSED AT HERE ON PURPOSE. They vary with goesrecv
version and build, and inventing them in a config file that then silently
matches nothing is exactly the sort of failure this repo keeps trying to
avoid. Run the discovery mode, see what your build actually emits, and pick
from that:

    python3 metric.py --discover

Then measure how noisy it is before trusting it (see below):

    python3 metric.py --noise <name> --seconds 120

If you decode with SatDump instead, see satdump.py -- it provides the same
interface over SatDump's HTTP status endpoint, plus detection of when a
recording has actually started.
"""

from __future__ import annotations

import argparse
import re
import socket
import statistics
import sys
import threading
import time
from collections import defaultdict, deque

DEFAULT_STATSD_PORT = 8125

# statsd wire format: "name:value|type" with an optional "|@rate", and packets
# may carry several metrics separated by newlines.
_STATSD = re.compile(r"^([^:]+):(-?[0-9.eE+]+)\|(\w+)")


class MetricError(RuntimeError):
    pass


# ---------------------------------------------------------------------------
#  Sources
# ---------------------------------------------------------------------------

class MetricSource:
    """
    A named-value history with a background feeder.

    Everything downstream -- Metric, noise_floor, the search -- only ever
    needs names(), since() and latest(), so the transport underneath is
    interchangeable. statsd from goesrecv is one; SatDump's HTTP status
    (see satdump.py) is another.

    Every source keeps ALL names it sees rather than only a configured one.
    That is what makes discovery possible, and it means a mis-set name shows
    up as "that name never arrived" instead of as a silent stream of zeros.
    """

    def __init__(self, history: int = 20000):
        self._hist = defaultdict(lambda: deque(maxlen=history))
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def _record(self, name: str, value: float, when=None):
        with self._lock:
            self._hist[name].append((when if when is not None else time.time(),
                                     value))

    def names(self):
        with self._lock:
            return sorted(self._hist.keys())

    def since(self, name: str, t0: float):
        """Every (time, value) for `name` recorded at or after t0."""
        with self._lock:
            return [(t, v) for (t, v) in self._hist[name] if t >= t0]

    def latest(self, name: str):
        with self._lock:
            h = self._hist[name]
            return h[-1] if h else None

    def start(self):
        raise NotImplementedError

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


class StatsdSource(MetricSource):
    """
    Listens for statsd datagrams, as goesrecv's [monitor] section emits them.
    """

    def __init__(self, port: int = DEFAULT_STATSD_PORT,
                 bind: str = "127.0.0.1", history: int = 20000):
        super().__init__(history=history)
        self.port = port
        self.bind = bind
        self._sock = None

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind((self.bind, self.port))
        except OSError as exc:
            raise MetricError(
                f"cannot bind {self.bind}:{self.port} -- {exc}\n"
                "Something else is already listening, or goesrecv is pointed "
                "at a different port than this is.") from exc
        self._sock.settimeout(0.5)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.time()
            for line in data.decode("utf-8", "replace").splitlines():
                m = _STATSD.match(line.strip())
                if not m:
                    continue
                name, raw, _kind = m.groups()
                try:
                    value = float(raw)
                except ValueError:
                    continue
                self._record(name, value, now)

    def stop(self):
        self._stop.set()
        if self._sock:
            self._sock.close()
        super().stop()


# ---------------------------------------------------------------------------
#  Sampling
# ---------------------------------------------------------------------------

class Metric:
    """
    One named metric, with a direction.

    lower_is_better is not cosmetic. Viterbi corrected errors go DOWN as the
    signal improves and SNR goes UP, so a search that gets this backwards
    walks confidently off the peak. It is the single easiest sign error to
    make in the whole loop.
    """

    def __init__(self, source: MetricSource, name: str,
                 lower_is_better: bool = True):
        self.source = source
        self.name = name
        self.lower_is_better = lower_is_better

    def sample(self, dwell: float, settle: float = 0.0):
        """
        Average the metric over `dwell` seconds, after discarding `settle`
        seconds first.

        The settle discard exists because the dish is still moving, and still
        ringing, for a while after the controller reports IDLE. Averaging
        across that smears two pointing positions into one number.

        Returns (mean, stdev, n). Raises if nothing arrived, which is the
        honest outcome when the demodulator has lost lock -- a search must
        stop rather than treat silence as a reading.
        """
        if settle > 0:
            time.sleep(settle)

        t0 = time.time()
        time.sleep(dwell)
        rows = self.source.since(self.name, t0)

        if not rows:
            raise MetricError(
                f"no samples of {self.name!r} in {dwell:.1f}s. "
                "Either the name is wrong (try --discover), goesrecv is not "
                "running, or the demodulator has lost lock.")

        values = [v for (_, v) in rows]
        mean = statistics.fmean(values)
        stdev = statistics.stdev(values) if len(values) > 1 else 0.0
        return mean, stdev, len(values)

    def better(self, a: float, b: float) -> bool:
        """True when reading `a` is a better pointing than reading `b`."""
        return a < b if self.lower_is_better else a > b


# ---------------------------------------------------------------------------
#  Noise floor
# ---------------------------------------------------------------------------

def block_means(rows, dwell: float):
    """Split (time, value) rows into `dwell`-second blocks and average each."""
    if not rows:
        return []
    out = []
    start = rows[0][0]
    bucket = []
    for t, v in rows:
        if t - start >= dwell:
            if bucket:
                out.append(statistics.fmean(bucket))
            bucket = []
            start = t
        bucket.append(v)
    if bucket:
        out.append(statistics.fmean(bucket))
    return out


def noise_floor(source: MetricSource, name: str, seconds: float,
                dwells=(1.0, 2.0, 5.0, 10.0)):
    """
    Measure the metric with the dish NOT MOVING, and report how much the
    averaged reading still wanders.

    This is the same test the reed switch got, for the same reason. There the
    question was how many edges arrive when nothing is turning; here it is how
    much the metric moves when nothing is pointing differently. Both answer
    the only question that matters before you build a loop on a sensor: is the
    thing you are about to react to actually a signal?

    The output is what sets both the dwell and the step size:

        one step must change the metric by more than about 3x the
        standard deviation of the averaged reading at that dwell

    If it does not, the search is climbing noise, and it will wander away from
    a perfectly good peak with total confidence.
    """
    print(f"measuring {name!r} for {seconds:.0f}s -- DO NOT MOVE THE DISH")
    t0 = time.time()
    last_report = t0
    while time.time() - t0 < seconds:
        time.sleep(0.5)
        if time.time() - last_report >= 10.0:
            last_report = time.time()
            n = len(source.since(name, t0))
            print(f"  {time.time() - t0:5.0f}s  {n} samples")

    rows = source.since(name, t0)
    if len(rows) < 2:
        raise MetricError(
            f"only {len(rows)} samples of {name!r} arrived. "
            "Wrong name, goesrecv not running, or no lock.")

    values = [v for (_, v) in rows]
    rate = len(rows) / (rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0

    print()
    print(f"samples   {len(values)}  ({rate:.1f}/s)")
    print(f"mean      {statistics.fmean(values):.4g}")
    print(f"stdev     {statistics.stdev(values):.4g}   (single reading)")
    print(f"min/max   {min(values):.4g} / {max(values):.4g}")
    print()
    print("  dwell    blocks   sigma of the averaged reading   min detectable step")
    # A standard deviation from a handful of blocks is itself mostly noise --
    # with 3 blocks the estimate is worth roughly nothing, and it will happily
    # tell you a 5 s dwell is no quieter than a 1 s one. Eight is the point
    # where the number starts meaning something.
    MIN_BLOCKS = 8
    thin = False
    for d in dwells:
        means = block_means(rows, d)
        if len(means) < MIN_BLOCKS:
            thin = True
            note = f"only {len(means)} blocks -- run at least {d * MIN_BLOCKS:.0f}s"
            print(f"  {d:5.1f}s   {len(means):5d}   {note:>22}")
            continue
        sd = statistics.stdev(means)
        print(f"  {d:5.1f}s   {len(means):5d}   {sd:>22.4g}   {3 * sd:.4g}")

    if thin:
        print()
        print(f"Rows marked above had fewer than {MIN_BLOCKS} blocks. Averaging "
              f"should cut\nsigma by about the square root of the dwell ratio; "
              f"if it does not, the\nrun was too short rather than the metric "
              f"being stubbornly noisy.")

    print()
    if statistics.fmean(values) == 0.0:
        print("mean is exactly zero: the metric has FLOORED.")
        print("That is not a failure -- corrected errors hitting zero means you")
        print("are comfortably inside the beam and pointing better buys nothing")
        print("measurable. Peak on a metric that does not saturate (Es/N0), or")
        print("accept that anywhere in the flat region is a valid answer.")
    else:
        print("Pick the smallest dwell whose 'min detectable step' is below the")
        print("metric change one pointing step actually produces. Measure that")
        print("by stepping the dish once and comparing, before trusting a search.")


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[3])
    ap.add_argument("--port", type=int, default=DEFAULT_STATSD_PORT)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--discover", action="store_true",
                    help="list every metric name that arrives")
    ap.add_argument("--noise", metavar="NAME",
                    help="measure the noise floor of one metric")
    ap.add_argument("--watch", metavar="NAME",
                    help="print one metric live")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args(argv)

    if not (args.discover or args.noise or args.watch):
        ap.error("pick one of --discover, --noise or --watch")

    try:
        source = StatsdSource(port=args.port, bind=args.bind).start()
    except MetricError as exc:
        print(exc, file=sys.stderr)
        return 2

    try:
        if args.discover:
            print(f"listening on {args.bind}:{args.port} for {args.seconds:.0f}s")
            print("if nothing appears, check goesrecv's [monitor] statsd_address")
            deadline = time.time() + args.seconds
            seen = set()
            while time.time() < deadline:
                time.sleep(1.0)
                for n in source.names():
                    if n not in seen:
                        seen.add(n)
                        latest = source.latest(n)
                        print(f"  {n:<44} {latest[1] if latest else '?'}")
            if not seen:
                print("nothing arrived.")
                return 1
            print(f"\n{len(seen)} metric(s). Pick the one that tracks link "
                  f"quality and measure it with --noise.")

        elif args.noise:
            noise_floor(source, args.noise, args.seconds)

        elif args.watch:
            while True:
                time.sleep(1.0)
                latest = source.latest(args.watch)
                if latest is None:
                    print("  (nothing yet)")
                else:
                    print(f"  {latest[1]:.4g}")

    except KeyboardInterrupt:
        pass
    except MetricError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        source.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
