# Expose this build's git revision to the firmware as SUPEROS_BUILD_REV, which
# src/sysex.h publishes on SysEx 0x34 -> 0x35 as "<version> <rev>".
#
# Runs as a PlatformIO pre-script and standalone (`python3 tools/buildrev.py
# --print` emits the bare revision, which the sim Makefile passes through).
#
# Why this exists: the 101 was the only one of the four machines whose identity
# reply carried no build identity, so every baseline capture identified its 101
# row by an agent's word rather than by the machine's own answer (LESSONS
# L12/L14). Note that nothing else in this repo computed a revision: unlike the
# 606, neither tools/makesyx.py nor tools/mkrom.py touches git, and the images
# are named plainly (app-update.syx), so the value had to be built here.
#
# ONE DELIBERATE DIFFERENCE FROM THE 606's tools/version_flag.py, which is
# otherwise mirrored byte for byte in its output shape: that script passes
# --untracked-files=no, so an untracked file does not mark the build dirty.
# On this machine build_src_filter is `+<*>`, so an untracked .cpp dropped in
# src/ IS COMPILED INTO THE IMAGE while leaving the marker clean. A flash
# carries the whole tree, not your diff. Counting untracked files can only
# over-report dirty, never under-report, so the stricter rule is the safe one.
import os
import subprocess
import sys

try:
    Import("env")  # noqa: F821 -- defined when run inside PlatformIO/SCons
    ROOT = env["PROJECT_DIR"]  # noqa: F821
except Exception:
    env = None  # noqa: F811
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def revision():
    """Short HEAD sha, plus '*' when the working tree is not clean.

    Returns "unknown" rather than raising: a build that cannot reach git is a
    build whose identity is genuinely unknown, and saying so is the honest
    answer. It must never be mistaken for a clean sha, so it carries no hex.
    """
    try:
        rev = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
            stderr=subprocess.DEVNULL).decode().strip()
        if not rev:
            return "unknown"
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=ROOT,
            stderr=subprocess.DEVNULL).decode().strip()
        if dirty:
            rev += "*"
    except Exception:
        return "unknown"
    # The reply is 7-bit ASCII on a SysEx wire: anything outside 0x20..0x7E
    # would either set the status bit and truncate the message or render as
    # something the host cannot echo back. Drop it rather than send it.
    return "".join(c for c in rev if 0x20 <= ord(c) <= 0x7E) or "unknown"


if __name__ == "__main__" and "--print" in sys.argv:
    print(revision())
elif env is not None:
    env.Append(CPPDEFINES=[("SUPEROS_BUILD_REV", env.StringifyMacro(revision()))])  # noqa: F821
