# Working on MOON

Two people work on this repo: [@kaihori613](https://github.com/kaihori613), the
owner, and Luis ([@lonulu](https://github.com/lonulu)). Luis has the
hardware on his bench. Setting up a machine: [docs/dev-setup.md](docs/dev-setup.md).

## Branches and pull requests

`main` is protected on GitHub:

- **Nothing is pushed to `main` directly.** Every change arrives through a pull
  request.
- **Both CI checks must pass** before a PR can merge: `host-tests` and
  `firmware-syntax` (see [Before opening a PR](#before-opening-a-pr)).
- **No approving review is required.** Since 27 Oct 2026, either of you can
  merge a PR once its checks are green. A second pair of eyes is still worth
  asking for on anything that changes the firmware or the serial protocol,
  because CI can't tell whether the dish still moves correctly.
- **Force pushes and deleting `main` are blocked.**
- **The owner can bypass these rules,** because they don't apply to admins.
  With no review required, the only thing a bypass skips is CI. Don't use it
  unless the checks themselves are what's broken.

The flow:

1. Start from an up-to-date `main`:
   ```bash
   git switch main
   git pull
   git switch -c <branch-name>
   ```
2. Name the branch after the work, for example `sdr-snr-closed-loop` or
   `luis/actuator-pid`.
3. Commit, push, and open a PR into `main`. Mark it **Draft** while it is still
   in progress.
4. Wait for both CI checks to go green, ask the other person to look if it
   touches the firmware or the protocol, then merge.
5. Delete the branch once it's merged, unless work continues on it.

**Long-running branches** (like `sdr-snr-closed-loop`) should take in `main`
whenever it changes, so they keep building on the current firmware:

```bash
git switch sdr-snr-closed-loop
git merge main
```

## Before opening a PR

Run what applies to what you changed. Say in the PR what you ran.

| changed | run | from |
|---|---|---|
| anything in `host/` | `..\.venv\Scripts\python -m pytest` — all must pass | `host/` |
| `snr_peak.py`, `peak.py`, `iq_metric.py`, `iq_snr.py` | `..\.venv\Scripts\python snr_peak.py --simulate --sweep -8 8 1` as well | `host/` |
| any sketch, or `Config.h` | `bash tools/syntax_check.sh --matrix` — 0 failed | repo root, in Git Bash |
| firmware you've flashed | a note of what you saw on hardware | the PR description |

CI (`.github/workflows/ci.yml`) runs the pytest and `--matrix` rows on every PR
into `main`, as `host-tests` and `firmware-syntax`, and both must pass before it
can merge. It doesn't run the simulated sweep or touch hardware, so those rows
are still yours.

**Say plainly what was tested on hardware and what wasn't.** "Simulated only"
and "ran on the bench, moved to g 5 and held" are both fine. What isn't fine is
leaving the reader to guess, because on this project a passing simulation and a
working dish are very different claims.

**If you change the serial protocol** (the `g`, `s` or `k` commands, the status
line's fields, the boot banner), say so in the PR. `host/link.py` and
`host/snr_peak.py` parse all of them.

## Commits

- **The subject line** is an imperative summary: "Make snr_psd refuse pure
  noise", not "fixed psd".
- **The body says why**, what was measured or verified, and anything the reader
  might otherwise assume. See `git log` for the house style.
- **One topic per commit** where it's practical.

## Repo conventions

- **Archive, don't delete.** Superseded sketches move to `archive/` and stay
  buildable. `actuator_system/` was once deleted outright and had to be
  recovered from git history twice.
- **Site-specific files stay out of git.** `host/config.json` (your
  coordinates and trim) and `host/pointing.json` (your dish's peak) are
  gitignored, as are the `host/sweep*.png` charts.
- **Mirrored constants must match.** `host/snr_peak.py` keeps its own copies of
  `SOFT_LIMIT_DEG` and `DEADBAND_DEG` from `actuator_v2/Config.h`. Change one,
  change both. Its `BEAMWIDTH_DEG = 12` matches the figure in `Config.h`'s
  comments, which is a calculation for a 1 m dish at 1694 MHz, not a
  measurement.
- **Write down what isn't measured yet.** `Config.h` marks guessed constants
  `PLACEHOLDER`. Keep doing that until a measurement replaces them.
