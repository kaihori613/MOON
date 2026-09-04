# Repurposing a 30-year-old TVRO antenna for GEO weather satellite imagery

**Project MOON** — status summary

Our next objective is to acquire imagery from GEO weather satellites with a
parabolic satellite dish.

## The antenna

We retrieved a 30-year-old TVRO antenna from a lady in Santa Clara and began
rebuilding it into an L-band full-disk imagery reception antenna. The structure
was fairly rusted, so we had to disassemble it before we could transport it back
to the lab. After reassembling it with new screws, we installed a newer version
of the cantenna feed mount. The dish has four feed-support arms, held together
at the focus by the mount plate.

Out in the field we received a clear picture at an SNR of about 13 dB, which
confirmed the reflector and feed are sound. The antenna itself is no longer the
open question.

## The actuator

We are now working on the linear actuator that came with the antenna. Its
gearbox is equipped with a reed switch and cam switches. Hooking the actuator
straight to a DC power supply confirmed that it still works, though the brushed
motor made a squeaky sound, which told us it would need to come apart and be
cleaned.

We wrote software to check the functionality of each switch. Only the reed
switch gives digital feedback, which tells us it is the main method of
controlling the actuator; the cam switches are mechanical cut-offs. They sit in
series with the motor inside the housing, each with a bypass diode so the motor
can still drive back off a limit once the switch has opened. That is why the
actuator presents only four wires — two heavy ones for the motor, two light ones
for the reed, which is an isolated dry contact.

We decided to use an L298N motor driver to create extension and retraction
control, and it works.

## Position is counted, not timed

The single most important design decision so far is that the actuator is
positioned by **reed count, never by elapsed time**.

Millimetres per reed count is a property of the mechanism — gear ratio through a
fixed screw pitch — so it holds at any duty, any load and either direction.
Millimetres per second holds at none of them. Ten seconds of extension followed
by ten seconds of retraction does not return the rod to where it started,
because the dish weight assists one direction and opposes the other. Fifty
counts out and fifty counts back should, and whatever it misses by is backlash
measured cleanly, with the speed difference taken out.

Counting is dead reckoning, so it needs an absolute reference. The controller
homes into the retract cam on every boot — the only direction-unambiguous move
available, since from anywhere in the stroke, retracting far enough arrives. A
saved position is loaded but deliberately not trusted; it is checked against
where the cam actually turns out to be, and the difference is drift. Wind
back-driving the dish while the power is off has no other symptom.

## What the bench has found

Several problems surfaced once we started driving the actuator and reading the
sensor in the same run.

**The supply was current-limiting.** A dish actuator draws several amps running
and considerably more at breakaway. Our bench supply was set to 2 A, then 3 A;
at those limits the supply drops into constant current, the rail collapses, and
the motor never breaks away. It wants 8–10 A.

**The L298N is thermally marginal.** It is a BJT bridge that drops 3–5 V across
its output stage, which at a few amps is 6–12 W in a package whose stock
heatsink handles perhaps 2–3 W. Thermal shutdown presents exactly as a run dying
early. We raised the supply from 24 V to 30 V to compensate — that is not
overvoltage at the motor, since after the bridge drop the actuator still sees
roughly 25–27 V. The 30 V is paying for the bridge.

**The motor wiring is undersized.** The actuator is still connected with
male-to-male jumper leads rated around 1 A while carrying several. This may
account for a large share of the breakaway trouble we had been attributing to
the supply.

**The reed signal needs a debounce, and currently has none.** We captured a
single magnet pass on the oscilloscope at 100 µs/div and measured a contact
bounce burst of roughly 350–400 µs before the signal settles. Every spike in
that burst is a false edge. The sketch is presently running with the debounce
set to zero, so all of them are being counted as travel — which explains the
inflated reed counts and very likely the extend/retract count asymmetry we saw.
The indicated setting is 800 µs to 1 ms: comfortably above the measured bounce,
and well below the fastest real pulse gap.

**The actuator runs rough since it was cleaned.** Grease loss is the leading
explanation — solvent does not distinguish between old grease and grit, and a
dry acme screw has not just more friction but *variable* friction, which is what
stick-slip is. It needs repacking with lithium or moly grease. Retract has also
failed in a way we have not yet separated between a cam limit and a bypass
diode.

## Manual controls

We added a control panel to both sketches: hold-to-run extend and retract
buttons, and a latching kill switch. The kill acts on the first sample with no
debounce, latches, and requires both a physical release and an explicit command
before anything will move again. The kill switch is wired normally closed, so a
broken wire or a pulled connector kills exactly as a press does.

Manual jogs go through the same motion primitives as commanded moves, so
position tracking and pulse counting are unaffected by using the panel. The
stall watchdog still applies and re-arms only on button release — without that,
holding a button against a cam limit would re-energise the motor indefinitely.

## What everything waits on

**Millimetres per reed count has never been measured on this hardware.**

One count is the smallest move the controller can distinguish, so it is also the
hard floor on pointing accuracy — the loop cannot hold tighter than one count
however it is tuned, and backlash sits on top of that. Until that number is
known, the pointing resolution of the system is unknown and every constant
downstream of it is a guess.

If it turns out coarser than acquisition needs, the fix is mechanical — a longer
moment arm from the actuator to the yaw pivot — not a change to any code.

It is one count-targeted run and a pair of calipers away.

## Next

1. Repack the gearbox and screw with grease, and resolve the retract fault.
2. Replace the jumper leads with properly sized wire and raise the supply
   current limit.
3. Restore the debounce to about 900 µs and confirm the fastest real pulse gap
   sits clear of it.
4. Run the noise floor test with the carriage still and the bridge powered. Zero
   edges is the only passing result — a false count is indistinguishable from a
   real one and corrupts position permanently.
5. Measure millimetres per reed count: a count-targeted run, calipers on the rod.
6. Find the trim duty by dropping speed until the overshoot fits inside one
   count. Overshoot is coast measured in counts, and it is the floor on where
   any move can land.
7. Mount the dish and repeat. Millimetres per count must come back *identical* —
   that is a free integrity check on the count. The overshoot will not, because
   the dish adds inertia, so the trim duty has to be found loaded.
8. Install the permanent cable run to the roof: a 10 m route, 12 AWG SOOW for
   the motor pair and a separate shielded twisted pair for the reed, never
   sharing a jacket.
