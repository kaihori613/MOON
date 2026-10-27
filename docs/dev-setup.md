# Setting up a Windows machine

Everything needed to run the tests and the simulation, check the firmware
compiles, and read the RTL-SDR, on Windows 11. These are the steps the
project's laptop was set up with on 27 Oct 2026; Python 3.12, GCC 16.2 and VS
Code 1.143 are what they were tested with.

**Open a new terminal after each install.** A terminal that was already open
keeps the old PATH and won't find what you just installed.

## 1. Git and the repo

```bash
winget install --id Git.Git -e
```
This also installs **Git Bash**, which `tools/syntax_check.sh` needs.

```bash
git clone https://github.com/kaihori613/MOON.git
cd MOON
```

Optionally, install the GitHub CLI for PRs from the terminal:

```bash
winget install --id GitHub.cli -e
gh auth login
```
Choose GitHub.com → HTTPS → Login with a web browser.

## 2. Python and the tests

```bash
winget install --id Python.Python.3.12 -e --scope user
```

If typing `python` opens the Microsoft Store instead, Python isn't installed
yet, or the terminal predates the install. That's the Windows store alias, not
a real Python.

From the repo root, make the project's own environment and install into it:

```bash
py -3.12 -m venv .venv
.venv\Scripts\python -m pip install -r host\requirements.txt numpy matplotlib pytest "pyrtlsdr[lib]"
```

`.venv/` is gitignored. What each package is for: the
[host README's setup section](../host/README.md#setup).

Check it works:

```bash
cd host
..\.venv\Scripts\python -m pytest
..\.venv\Scripts\python snr_peak.py --simulate --sweep -8 8 1 --plot
```

All tests should pass, and the simulation should end with `OK` and write
`host\sweep.png`.

## 3. The compiler for the firmware syntax check

```bash
winget install --id BrechtSanders.WinLibs.POSIX.UCRT -e
```

That's GCC for Windows, about 250 MB. Then, in **Git Bash**, from the repo
root:

```bash
bash tools/syntax_check.sh --matrix
```

Expect `0 failed`. This compiles every sketch, and every `Config.h` switch
combination, against stand-in Arduino headers. It catches typos and missing
declarations; it says nothing about whether the sketch fits the board or works
on hardware. Flashing still needs the Arduino IDE (step 6).

## 4. VS Code

```bash
winget install --id Microsoft.VisualStudioCode -e
code --install-extension ms-python.python
```

Open the **repo root folder** (File → Open Folder), not a single file. Then:

- **Trust the folder.** If the title bar says **Restricted Mode**, VS Code won't
  run or debug anything, and pressing ▶ does nothing. Use `Ctrl+Shift+P` →
  *Workspaces: Manage Workspace Trust* → *Trust*.
- **Use `.venv`'s Python.** `Ctrl+Shift+P` → *Python: Select Interpreter* →
  `.venv`. The newer *Python Environments* extension can quietly write
  `"python-envs.defaultEnvManager": "ms-python.python:system"` into
  `.vscode/settings.json`, which points VS Code at the system Python. That
  Python has none of the packages, so the tests fail with "No module named
  pytest".
- **Run things from Run and Debug** (`Ctrl+Shift+D`). `.vscode/launch.json` is
  committed, with the simulation scenarios, the Monte Carlo comparison and the
  test suite. Every entry names `.venv`'s Python explicitly, so it works
  whatever the interpreter picker says.

## 5. The RTL-SDR

`pyrtlsdr[lib]` from step 2 already includes the librtlsdr library. What's left
is Windows' driver for the dongle, which has to be swapped once per machine:

1. Plug the dongle in.
2. Download Zadig from <https://zadig.akeo.ie> and run it.
3. **Options → List All Devices.**
4. Pick **Bulk-In, Interface (Interface 0)**, or whichever entry shows USB ID
   **0BDA 2838**. Be careful: Realtek makes laptop webcams too, so the webcam
   may also show an ID starting `0BDA`. Don't pick that.
5. Set the target driver to **WinUSB** and click **Replace Driver**.

Check Python can see it:

```bash
.venv\Scripts\python -c "from rtlsdr import librtlsdr; print(librtlsdr.rtlsdr_get_device_count())"
```

`1` means it's ready. `0` means Windows doesn't see a dongle, or the driver swap
went to the wrong device.

The dongle is **single-client**. Close SDR#, SatDump or anything else using it
before running `iq_metric.py` or `snr_peak.py`.

What to do with it next: [snr-bringup.md](snr-bringup.md).

## 6. The Arduino

Flashing `actuator_v2` uses the **Arduino IDE 2** from
<https://www.arduino.cc/en/software>. Open `actuator_v2/actuator_v2.ino`,
select your board and port, and upload. The port appears in Device Manager
under *Ports (COM & LPT)* when the board is plugged in. It's the same `COMx`
that `snr_peak.py --port` takes.

Before flashing, check `MOTOR_DRIVER` in `actuator_v2/Config.h` matches the
driver actually wired (`DRV_HW039` on this build), and read the bring-up
cautions in [WIRING.md](../WIRING.md).

## Harmless noise

- **`LF will be replaced by CRLF`** on `git add` is Git for Windows converting
  line endings. Nothing is wrong.
