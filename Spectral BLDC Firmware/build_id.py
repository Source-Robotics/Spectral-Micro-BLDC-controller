"""Puts the git commit and the build time into the firmware: parameters 84 and 85, and #Info.

BUILD_COMMIT is the commit's first 7 hex digits, with bit 28 set when the firmware folder has uncommitted changes
(0 when git is not available). BUILD_TIME is the build's Unix time in seconds, UTC.
"""
import subprocess
import time

Import("env", "projenv")


def git(*args):
    try:
        return subprocess.run(["git", "-C", env.subst("$PROJECT_DIR"), *args], capture_output=True, text=True,
                              timeout=20, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


commit = git("rev-parse", "--short=7", "HEAD")
value = 0
if commit:
    value = int(commit[:7], 16)
    if git("status", "--porcelain", "--", "."):
        value |= 1 << 28

projenv.Append(CPPDEFINES=[("BUILD_COMMIT", "0x%08Xu" % value), ("BUILD_TIME", "%du" % int(time.time()))])
print("Build id: %s%s" % (commit or "unknown", "+" if value >> 28 else ""))
