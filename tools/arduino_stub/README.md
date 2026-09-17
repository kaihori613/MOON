# Arduino stub headers

Just enough of `Arduino.h`, `Wire.h` and `EEPROM.h` to let a host `g++`
syntax-check these sketches. Not a simulator and not a substitute for the real
toolchain: no AVR sizes, no flash figures, and nothing here executes.

What it does catch is everything that is wrong before you plug anything in --
typos, missing declarations, wrong argument types, a `#ifdef` branch nobody
ever built. On a project where several sketches have never been compiled at
all, that is most of the value of a compiler.

`Print` mirrors the real class's overload set on purpose, so an ambiguous or
missing `Serial.print` overload fails here the way it would on hardware. Where
the real Arduino core pulls in AVR-libc or the C library implicitly
(`dtostrf`, `snprintf`, `strcpy`, `tolower`, `map`, the `A0`-`A7` constants),
those are declared here too -- their absence is a gap in the stub, not a bug
in a sketch.
