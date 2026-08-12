"""
keys.py
-------
Single-keypress reading, so the manual tuning console responds the moment you
press an arrow instead of waiting for Enter.

This is the whole reason the tuning lives on the PC rather than in the sketch.
The Arduino IDE's Serial Monitor is line buffered: it sends nothing until you
hit Enter, which makes "nudge and watch the signal meter" impossible.

Returns a normalised name for the keys we care about:

    'up' 'down' 'left' 'right' 'enter' 'esc' 'space'

and the literal lowercase character for anything else. Returns None if no key
is waiting, so the caller's loop stays free to poll the actuator.
"""

from __future__ import annotations

import sys

_WINDOWS = sys.platform.startswith("win")

if _WINDOWS:
    import msvcrt
else:
    import select
    import termios
    import tty


# Windows sends arrows as a two-character sequence: a 0x00 or 0xe0 prefix,
# then a letter identifying which arrow.
_WIN_SPECIAL = {
    "H": "up",
    "P": "down",
    "K": "left",
    "M": "right",
}

# POSIX terminals send ESC [ A and friends.
_ANSI_SPECIAL = {
    "A": "up",
    "B": "down",
    "C": "right",
    "D": "left",
}


class KeyReader:
    """
    Context manager that puts the terminal into raw/cbreak mode for its
    lifetime and restores it on the way out -- including on an exception, so a
    crash never leaves the user with a terminal that has stopped echoing.

    On Windows no mode change is needed; msvcrt reads the console directly.
    """

    def __init__(self):
        self._fd = None
        self._saved = None

    def __enter__(self):
        if not _WINDOWS:
            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if not _WINDOWS and self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        return False

    # -----------------------------------------------------------------------

    def get(self):
        """Non-blocking. Returns a key name, a character, or None."""
        if _WINDOWS:
            return self._get_windows()
        return self._get_posix()

    def _get_windows(self):
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getwch()

        if ch in ("\x00", "\xe0"):
            # Arrow or function key: the next character says which.
            code = msvcrt.getwch()
            return _WIN_SPECIAL.get(code)

        return _normalise(ch)

    def _get_posix(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)

        if ch == "\x1b":
            # Could be a bare Escape or the start of an arrow sequence. If
            # nothing follows immediately it was a bare Escape.
            if not select.select([sys.stdin], [], [], 0.02)[0]:
                return "esc"
            if sys.stdin.read(1) != "[":
                return "esc"
            return _ANSI_SPECIAL.get(sys.stdin.read(1))

        return _normalise(ch)

    def drain(self):
        """Throw away anything queued. Used after a long blocking operation."""
        while self.get() is not None:
            pass


def _normalise(ch: str):
    if ch in ("\r", "\n"):
        return "enter"
    if ch == " ":
        return "space"
    if ch == "\x1b":
        return "esc"
    if ch == "\x03":
        raise KeyboardInterrupt
    return ch.lower()
