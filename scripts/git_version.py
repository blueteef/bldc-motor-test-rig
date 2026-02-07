Import("env")
import subprocess
from datetime import datetime, timezone

def run(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return ""

# Short SHA
sha = run("git rev-parse --short HEAD") or "nogit"

# Dirty working tree?
dirty = run("git status --porcelain")
is_dirty = "1" if dirty else "0"

# UTC build timestamp (ISO-ish)
build_utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

# Inject as preprocessor defines.
# Note: PlatformIO expects -DKEY=\"value\" for string macros.
env.Append(
    CPPDEFINES=[
        ("GIT_SHA", '\\"%s\\"' % sha),
        ("GIT_DIRTY", is_dirty),
        ("BUILD_UTC", '\\"%s\\"' % build_utc),
    ]
)
