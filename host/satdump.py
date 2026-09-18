"""
satdump.py
----------
SatDump as the signal source: where the metric comes from, and how to tell
that a recording has actually begun.

TWO SIGNALS, AND THEY ARE NOT THE SAME ONE

Triggering the peak search on "SatDump started recording" is the right idea
but the wrong edge. A recording starting means the scheduler fired. It does
not mean the demodulator has locked, and until it has, the metric is either
absent or is measuring noise. A search that begins there is climbing a flat
surface and will wander off before the signal ever arrives.

So this module separates them:

    recording started  ->  ARM the search
    metric is live     ->  RUN it

RecordingWatcher gives you the first. Metric.sample() raising MetricError
gives you the second, by refusing to invent a reading when nothing is coming
in. wait_for_lock() below puts them together.

THE ENDPOINT FORMAT IS NOT GUESSED AT HERE

SatDump's live status is served by its built-in HTTP server:

    satdump live <pipeline> <output> --source ... --http_server 0.0.0.0:8080

What that endpoint returns varies by SatDump version and by pipeline, and a
field name baked in here that then silently matches nothing is exactly the
failure this repo keeps trying to avoid. So HttpSource flattens whatever JSON
arrives into dotted names and keeps all of them, and there is a probe mode to
show you what your build actually serves:

    python3 satdump.py --probe http://127.0.0.1:8080/

Pick the field that tracks link quality -- SatDump generally exposes an SNR
in dB, in which case HIGHER IS BETTER and Metric must be constructed with
lower_is_better=False. Getting that backwards is the easiest error in the
whole loop.

Then measure how noisy it is before trusting it, exactly as with any other
source:

    python3 -c "..."   # or: metric.py --noise, pointed at this source
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

from metric import MetricError, MetricSource

DEFAULT_URL = "http://127.0.0.1:8080/"


# ---------------------------------------------------------------------------
#  Metric over SatDump's HTTP status
# ---------------------------------------------------------------------------

def flatten(obj, prefix=""):
    """
    Turn nested JSON into {dotted.name: float} for every numeric leaf.

    Non-numeric leaves are dropped rather than coerced. A status string like
    "SYNCED" is genuinely useful to a human but there is nothing sensible for
    a search to do with it, and silently mapping it to 1.0 would put a
    constant into the loop that looks like a measurement.
    """
    out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(flatten(v, f"{prefix}{k}."))
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            out.update(flatten(v, f"{prefix}{i}."))
    elif isinstance(obj, bool):
        pass                      # a flag, not a measurement
    elif isinstance(obj, (int, float)):
        out[prefix.rstrip(".")] = float(obj)
    return out


class HttpSource(MetricSource):
    """
    Polls SatDump's HTTP status endpoint and records every numeric field.

    Polling rather than pushing, because SatDump does not push. The interval
    is the real resolution limit on the metric: at 1 s you get one sample per
    second to average, so a 5 s dwell is 5 samples, and the noise floor
    measurement in metric.py will tell you whether that is enough.
    """

    def __init__(self, url: str = DEFAULT_URL, interval: float = 1.0,
                 history: int = 20000, timeout: float = 3.0):
        super().__init__(history=history)
        self.url = url
        self.interval = interval
        self.timeout = timeout
        self.last_error = None
        self._ok = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def poll_once(self):
        """One fetch. Returns the flattened dict, or raises MetricError."""
        try:
            with urllib.request.urlopen(self.url, timeout=self.timeout) as r:
                body = r.read().decode("utf-8", "replace")
        except (urllib.error.URLError, OSError) as exc:
            raise MetricError(
                f"cannot reach {self.url} -- {exc}\n"
                "Is SatDump running with --http_server, and on this port?"
            ) from exc
        try:
            doc = json.loads(body)
        except json.JSONDecodeError as exc:
            raise MetricError(
                f"{self.url} did not return JSON ({exc}). "
                f"First 200 bytes: {body[:200]!r}") from exc
        return flatten(doc)

    def _run(self):
        while not self._stop.is_set():
            try:
                for name, value in self.poll_once().items():
                    self._record(name, value)
                self.last_error = None
                self._ok.set()
            except MetricError as exc:
                self.last_error = exc
                self._ok.clear()
            self._stop.wait(self.interval)

    def wait_reachable(self, timeout: float = 30.0) -> bool:
        return self._ok.wait(timeout)


# ---------------------------------------------------------------------------
#  "Has a recording started?"
# ---------------------------------------------------------------------------

class RecordingWatcher:
    """
    Detects that SatDump has begun writing a recording, by watching its output
    directory for a new entry.

    Filesystem rather than an API on purpose. It needs no SatDump feature, no
    version agreement and no cooperation from the scheduler -- SatDump makes a
    directory per recording whatever else changes, so this keeps working
    across upgrades. It is also entirely stdlib.

    It does NOT mean the demodulator has locked. See wait_for_lock().
    """

    def __init__(self, root, poll: float = 2.0):
        self.root = Path(root)
        self.poll = poll
        self._seen = set()

    def snapshot(self):
        """Record what is already there, so only NEW entries count as a start."""
        if not self.root.is_dir():
            raise MetricError(f"{self.root} is not a directory")
        self._seen = {p.name for p in self.root.iterdir()}
        return self._seen

    def poll_new(self):
        """Entries that have appeared since the last snapshot/poll."""
        if not self.root.is_dir():
            return []
        now = {p.name for p in self.root.iterdir()}
        fresh = sorted(now - self._seen)
        self._seen = now
        return [self.root / n for n in fresh]

    def wait_for_start(self, timeout=None):
        """
        Block until a new entry appears under the output root. Returns its
        path, or None on timeout.
        """
        self.snapshot()
        deadline = None if timeout is None else time.time() + timeout
        while deadline is None or time.time() < deadline:
            fresh = self.poll_new()
            if fresh:
                return fresh[0]
            time.sleep(self.poll)
        return None


def wait_for_lock(metric, timeout: float = 120.0, dwell: float = 2.0,
                  on_wait=None):
    """
    Block until the metric is actually producing readings.

    This is the gate the search should open on, not the recording start. It
    returns the first successful (mean, stdev, n), or raises on timeout --
    because a search that runs without a signal does not fail loudly, it just
    walks the dish somewhere wrong and reports success.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            return metric.sample(dwell=dwell)
        except MetricError:
            if on_wait:
                on_wait(deadline - time.time())
            time.sleep(1.0)
    raise MetricError(
        f"no lock within {timeout:.0f}s. The dish is probably not close "
        "enough to the bird yet -- point from the computed look angle first, "
        "then peak.")


# ---------------------------------------------------------------------------
#  CLI
# ---------------------------------------------------------------------------

def cmd_probe(url: str):
    src = HttpSource(url=url)
    print(f"probing {url}")
    try:
        fields = src.poll_once()
    except MetricError as exc:
        print(exc, file=sys.stderr)
        return 1
    if not fields:
        print("reached it, but no numeric fields came back.")
        print("The pipeline may not be running yet.")
        return 1
    print(f"\n{len(fields)} numeric field(s):\n")
    for name in sorted(fields):
        print(f"  {name:<50} {fields[name]:g}")
    print("\nPick the one that tracks link quality.")
    print("If it is an SNR in dB, HIGHER IS BETTER -- construct Metric with")
    print("lower_is_better=False, then measure its noise floor before use.")
    return 0


def cmd_watch_dir(root: str, timeout):
    w = RecordingWatcher(root)
    print(f"watching {root} for a new recording -- start one in SatDump")
    try:
        found = w.wait_for_start(timeout=timeout)
    except MetricError as exc:
        print(exc, file=sys.stderr)
        return 1
    if found is None:
        print("nothing appeared before the timeout.")
        return 1
    print(f"recording started: {found}")
    print("That is the ARM signal. Wait for the metric before searching.")
    return 0


def cmd_watch_metric(url: str, name: str, seconds: float):
    src = HttpSource(url=url).start()
    try:
        if not src.wait_reachable(timeout=10.0):
            print(src.last_error or "endpoint unreachable", file=sys.stderr)
            return 1
        deadline = time.time() + seconds
        while time.time() < deadline:
            time.sleep(1.0)
            latest = src.latest(name)
            print("  (nothing yet)" if latest is None else f"  {latest[1]:g}")
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="SatDump metric source and recording detector")
    ap.add_argument("--url", default=DEFAULT_URL,
                    help=f"SatDump --http_server address (default {DEFAULT_URL})")
    ap.add_argument("--probe", action="store_true",
                    help="show every numeric field the endpoint serves")
    ap.add_argument("--watch-dir", metavar="PATH",
                    help="block until a new recording appears under PATH")
    ap.add_argument("--watch", metavar="FIELD",
                    help="print one field live")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args(argv)

    if args.probe:
        return cmd_probe(args.url)
    if args.watch_dir:
        return cmd_watch_dir(args.watch_dir, args.seconds)
    if args.watch:
        return cmd_watch_metric(args.url, args.watch, args.seconds)
    ap.error("pick one of --probe, --watch-dir or --watch")


if __name__ == "__main__":
    raise SystemExit(main())
