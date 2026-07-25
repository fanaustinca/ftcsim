"""Single source of truth for the project version.

Semantic versioning: MAJOR.MINOR.PATCH

    MAJOR  incompatible changes -- an existing OpMode or script breaks
    MINOR  new features, backwards compatible
    PATCH  bug fixes only, no new features

Still 0.x, which means the API isn't promised to be stable yet. The two things
holding it back from 1.0.0 are the gamepad path (written, never run against a
real controller) and the Onshape import (wired up, never run against a real
document). Both need to work on real hardware before this claims 1.0.

WHEN YOU CHANGE SOMETHING, BUMP IT HERE and add a CHANGELOG.md entry. This
number is shown in the corner of the teleop window, so it is how anyone tells
which build they're actually looking at.
"""

import subprocess
from pathlib import Path

__version__ = "0.10.0"

VERSION_INFO = tuple(int(p) for p in __version__.split("."))


def git_suffix() -> str:
    """Short commit hash, plus -dirty if there are uncommitted edits.

    Matters more than it looks: the version constant only changes when someone
    remembers to bump it, but the hash changes on every commit. If you pulled
    mid-release or have local edits, this is what tells you.

    Returns "" when git isn't available or this isn't a checkout.
    """
    root = Path(__file__).resolve().parent
    try:
        rev = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=2)
        if rev.returncode != 0:
            return ""
        out = "+" + rev.stdout.strip()
        dirty = subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                               capture_output=True, text=True, timeout=2)
        if dirty.returncode == 0 and dirty.stdout.strip():
            out += "-dirty"
        return out
    except (OSError, subprocess.SubprocessError):
        return ""


def describe(with_git: bool = True) -> str:
    """e.g. 'v0.9.0+d6c7f37' or 'v0.9.0+d6c7f37-dirty' or just 'v0.9.0'."""
    return f"v{__version__}" + (git_suffix() if with_git else "")


if __name__ == "__main__":
    print(describe())
