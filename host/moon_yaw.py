#!/usr/bin/env python3
"""
moon_yaw.py
-----------
Yaw pointing for the MOON antenna: run the automatic cycle, then hand the
keyboard over for manual peaking.

    home  ->  compute where GOES-18 is  ->  drive there  ->  you nudge it in

The nudging is the point. Everything up to "drive there" is open loop as far
as the *signal* is concerned -- the controller knows where the carriage is, but
nothing in the system knows where the beam actually is. Survey error, mount
flex, a linkage measurement off by a few millimetres, and backlash all land on
top of each other. So the last step is a human watching a signal meter and
pressing arrow keys.

What the manual step produces is a TRIM, not a position. Trim is stored as an
offset from the computed target:

    commanded = computed(satellite) + trim

which means re-homing, power cycling, or recomputing does not throw your
correction away. For a geostationary bird from a fixed site the look angle
never changes, so once the trim is right it stays right -- it is a permanent
site correction, and it is the most valuable number this program produces.
Press 's' to save it.

    python moon_yaw.py --port COM3
    python moon_yaw.py --dry-run          (no hardware; just the math)
    python moon_yaw.py --list-ports
"""

from __future__ import annotations

import argparse
import sys
import time

from config import Config, ConfigError, DEFAULT_PATH
from geometry import LinkageError, degrees_per_count, look_angles
from keys import KeyReader
from link import ActuatorError, ActuatorFault, ActuatorLink

# Keep in step with SOFT_LIMIT_MARGIN in actuator_v1/Config.h. The
# sketch enforces its own copy; this one only exists so the host can warn
# before sending a command it knows will be clamped.
SOFT_LIMIT_MARGIN = 5


# ===========================================================================
#  Pointing
# ===========================================================================

def compute_target(cfg: Config, linkage):
    """
    Returns (counts_before_trim, azimuth_in_linkage_frame, elevation, info).

    Raises ConfigError if the satellite is below the horizon -- there is no
    sensible actuator position for that and it means the site coordinates are
    wrong, not that the mount needs to try harder.
    """
    lat, lon, alt = cfg.site
    az_true, el, rng = look_angles(lat, lon, alt, cfg.satellite_lon)

    if el <= 0.0:
        raise ConfigError(
            f"{cfg.satellite_name} computes to {el:.2f} deg elevation from "
            f"{lat:.4f}, {lon:.4f} -- that is below the horizon. Check the site "
            "coordinates, and remember longitude is East-positive.")

    az = cfg.to_linkage_frame(az_true)
    counts = linkage.counts_for_angle(az)

    info = {
        "az_true": az_true,
        "az_linkage": az,
        "elevation": el,
        "range_km": rng / 1000.0,
    }
    return counts, az, el, info


def clamp_to_travel(counts: float, travel):
    """Keep off the mechanical stops. Returns (clamped, was_clamped)."""
    low = SOFT_LIMIT_MARGIN
    if travel is None:
        return max(low, counts), counts < low
    high = travel - SOFT_LIMIT_MARGIN
    clamped = min(max(counts, low), high)
    return clamped, clamped != counts


# ===========================================================================
#  The automatic cycle
# ===========================================================================

def run_auto_cycle(link: ActuatorLink, cfg: Config, linkage, force_home: bool):
    print("--- automatic cycle ----------------------------------------")

    st = link.status()

    if st["fault"]:
        print(f"  controller is faulted ({st['fault']}) -- clearing")
        link.clear_fault()
        st = link.status()

    if st["travel"] is None:
        print("  ! travel is not calibrated. Soft limits are unknown, so the only")
        print("    thing standing between a move and the hard stop is stall")
        print("    detection. Run 'c' in the Arduino console first.")

    if force_home or not st["homed"]:
        why = "forced" if force_home else "position unknown"
        print(f"  homing ({why}) -- retracting into the hard stop...")
        link.home()
        st = link.wait_idle(timeout=240.0,
                            on_progress=lambda s: _progress(f"  homing  pos={s['pos']}"))
        print(f"\r  homed at pos={st['pos']}" + " " * 20)
    else:
        # actuator_v1 does not persist position, so it always boots un-homed and
        # this branch never fires against it. Kept because it costs nothing and
        # a later sketch may well restore a saved origin.
        print(f"  already homed, pos={st['pos']}")

    counts, az, el, info = compute_target(cfg, linkage)
    target = counts + cfg.trim
    target, clamped = clamp_to_travel(target, st["travel"])

    print(f"  {cfg.satellite_name} at {cfg.satellite_lon:.1f} deg lon:")
    print(f"     true azimuth   {info['az_true']:8.3f} deg")
    if abs(info["az_linkage"] - info["az_true"]) > 1e-9:
        print(f"     magnetic       {info['az_linkage']:8.3f} deg  (linkage frame)")
    print(f"     elevation      {info['elevation']:8.3f} deg   "
          f"(not actuated on this axis)")
    print(f"     slant range    {info['range_km']:8.0f} km")
    print(f"  yaw target       {counts:8.1f} counts"
          f"  {cfg.trim:+d} trim  ->  {target:.0f}")

    if clamped:
        print("  ! target was clamped to stay clear of a mechanical stop. The")
        print("    linkage cannot reach the computed heading -- check the")
        print("    calibration, or the mount needs repositioning.")

    print(f"  seeking {target:.0f}...")
    link.move_to(target)
    st = link.wait_idle(on_progress=lambda s: _progress(
        f"  moving  pos={s['pos']}  hz={s['hz']:.1f}"))
    print(f"\r  arrived at pos={st['pos']}" + " " * 30)

    return counts


def _progress(text: str):
    sys.stdout.write("\r" + text + " " * 12)
    sys.stdout.flush()


# ===========================================================================
#  Manual tuning
# ===========================================================================

HELP = """
--- manual tuning ------------------------------------------
  left / right   nudge by the step size   (also: - and +)
  up / down      double / halve the step size
  space          STOP
  s              save this trim to the config
  0              discard trim, return to the computed target
  a              re-run the automatic cycle
  k              clear a fault
  ?              this help
  q  or  Esc     quit
------------------------------------------------------------
Nudges move the actuator and update the trim as you go. Nothing is written to
disk until you press 's'.
"""


def manual_tuning(link: ActuatorLink, cfg: Config, linkage, computed: float):
    print(HELP)

    st = link.status()
    desired = float(st["pos"])
    step = cfg.nudge
    dirty = False
    message = ""
    last_poll = 0.0
    pending = None            # what we last commanded, so we do not re-send it

    with KeyReader() as reader:
        while True:
            key = reader.get()

            if key is not None:
                if key in ("q", "esc"):
                    link.stop()
                    print()
                    if dirty:
                        print("  trim was NOT saved (press 's' next time to keep it)")
                    return

                elif key in ("right", "+", "="):
                    desired += step
                    message = ""

                elif key in ("left", "-", "_"):
                    desired -= step
                    message = ""

                elif key == "up":
                    step = min(step * 2, 500)
                    message = f"step {step}"

                elif key == "down":
                    step = max(step // 2, 1)
                    message = f"step {step}"

                elif key == "space":
                    link.stop()
                    pending = None
                    st = link.status()
                    desired = float(st["pos"])
                    message = "STOPPED"

                elif key == "s":
                    cfg.trim = int(round(desired - computed))
                    cfg.nudge = step
                    cfg.save()
                    dirty = False
                    message = f"saved trim {cfg.trim:+d} to {cfg.path.name}"

                elif key == "0":
                    desired = computed + 0
                    message = "trim discarded"

                elif key == "a":
                    link.stop()
                    print("\n")
                    computed = run_auto_cycle(link, cfg, linkage, force_home=False)
                    st = link.status()
                    desired = float(st["pos"])
                    pending = None
                    reader.drain()
                    print(HELP)
                    continue

                elif key == "k":
                    link.clear_fault()
                    pending = None
                    message = "fault cleared"

                elif key == "?":
                    print("\n" + HELP)
                    continue

                if key in ("right", "left", "+", "-", "=", "_"):
                    dirty = True

            # -----------------------------------------------------------
            now = time.time()
            if now - last_poll < 0.25:
                time.sleep(0.01)
                continue
            last_poll = now

            try:
                st = link.status()
            except ActuatorError as exc:
                print(f"\n  link error: {exc}")
                return

            if st["fault"]:
                message = f"FAULT {st['fault']} -- press k to clear"
                pending = None
                desired = float(st["pos"])

            # Coalesce held keys: only issue a move once the previous one has
            # finished, and only if the destination actually changed. Sending
            # a 'g' per keypress would queue up moves and overshoot badly.
            elif st["state"] == "IDLE" and pending != round(desired):
                clamped, was_clamped = clamp_to_travel(desired, st["travel"])
                if was_clamped:
                    desired = clamped
                    message = "at the soft limit"
                if round(desired) != st["pos"]:
                    link.move_to(desired)
                    pending = round(desired)

            _draw(st, desired, computed, step, linkage, cfg, message)


def _draw(st, desired, computed, step, linkage, cfg, message):
    trim = desired - computed

    try:
        heading = f"{linkage.angle_for_counts(st['pos']):7.3f}"
        res = degrees_per_count(linkage, st["pos"])
        res_txt = f"{res:.4f} deg/ct"
    except LinkageError:
        heading = "    ---"
        res_txt = "off-scale"

    line = (f"  pos {st['pos']:5d} -> {round(desired):5d}   "
            f"yaw {heading} deg   "
            f"trim {trim:+6.0f}   "
            f"step {step:3d}   "
            f"{res_txt}   "
            f"{st['state']:<11}")

    if message:
        line += f"  {message}"

    sys.stdout.write("\r" + line[:158].ljust(158))
    sys.stdout.flush()


# ===========================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--config", default=DEFAULT_PATH, help="path to config.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="compute the pointing and exit; touches no hardware")
    ap.add_argument("--home", action="store_true",
                    help="home even if the controller thinks it knows where it is")
    ap.add_argument("--no-auto", action="store_true",
                    help="skip the automatic cycle and go straight to manual tuning")
    ap.add_argument("--list-ports", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="echo every serial line")
    args = ap.parse_args(argv)

    if args.list_ports:
        ports = ActuatorLink.list_ports()
        if not ports:
            print("no serial ports found")
        for device, description in ports:
            print(f"  {device:<12} {description}")
        return 0

    try:
        cfg = Config.load(args.config)
    except ConfigError as exc:
        print(f"config: {exc}", file=sys.stderr)
        return 2

    # The linkage cannot be calibrated until the actuator can be driven and
    # mm_per_count has been measured, but the look angles depend only on the
    # site coordinates. So a dry run stays useful with the linkage still blank
    # -- checking the azimuth against a map is worth doing early, and long
    # before anything can move.
    linkage = None
    linkage_problem = None
    try:
        linkage = cfg.linkage()
    except (ConfigError, LinkageError) as exc:
        if not args.dry_run:
            print(f"config: {exc}", file=sys.stderr)
            return 2
        linkage_problem = exc

    try:
        lat, lon, alt = cfg.site
    except ConfigError as exc:
        print(f"config: {exc}", file=sys.stderr)
        return 2

    print(f"site     {lat:.4f}, {lon:.4f} at {alt:.0f} m")
    print(f"linkage  {linkage.describe() if linkage else 'not calibrated'}")
    print(f"trim     {cfg.trim:+d} counts (saved)")

    if args.dry_run:
        az_true, el, rng = look_angles(lat, lon, alt, cfg.satellite_lon)
        print()
        print(f"{cfg.satellite_name}: azimuth {az_true:.3f} deg true, "
              f"elevation {el:.3f} deg, range {rng / 1000.0:.0f} km")

        if el <= 0.0:
            print("\n! that is below the horizon -- check the site coordinates, "
                  "and\n  remember longitude is East-positive.", file=sys.stderr)
            return 2

        az = cfg.to_linkage_frame(az_true)
        if abs(az - az_true) > 1e-9:
            print(f"{'':13}{az:.3f} deg magnetic (linkage frame)")

        if linkage is None:
            print(f"\nno yaw target: {linkage_problem}")
            print("The azimuth above is still valid -- it depends only on where "
                  "you are.")
        else:
            try:
                counts = linkage.counts_for_angle(az)
            except LinkageError as exc:
                print(f"\n! {exc}", file=sys.stderr)
                return 2
            print(f"yaw target: {counts:.1f} counts {cfg.trim:+d} trim "
                  f"= {counts + cfg.trim:.0f}")
            print(f"resolution at that point: "
                  f"{degrees_per_count(linkage, counts):.4f} deg per reed count")

        print("\n(dry run -- nothing was moved)")
        return 0

    if not args.port:
        print("\n--port is required. Run --list-ports to see what is attached.",
              file=sys.stderr)
        return 2

    link = ActuatorLink(args.port, args.baud, verbose=args.verbose)
    try:
        link.open()
    except Exception as exc:
        print(f"\ncould not open {args.port}: {exc}", file=sys.stderr)
        return 2

    if not link.saw_banner:
        print(f"\n! no boot banner from {args.port}. Continuing, but if commands")
        print("  are ignored, check the baud rate and that actuator_v1 is")
        print("  the sketch actually loaded.")

    try:
        if args.no_auto:
            computed, _, _, _ = compute_target(cfg, linkage)
        else:
            computed = run_auto_cycle(link, cfg, linkage, force_home=args.home)

        manual_tuning(link, cfg, linkage, computed)

    except ActuatorFault as exc:
        print(f"\n\nFAULT: {exc}", file=sys.stderr)
        return 1
    except (ActuatorError, ConfigError, LinkageError) as exc:
        print(f"\n\nerror: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n\ninterrupted -- stopping the motor")
        return 130
    finally:
        link.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
