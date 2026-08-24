#!/usr/bin/env python3
"""Make a macOS .app bundle self-contained.

Homebrew builds link against /opt/homebrew/... absolute paths, so the bundle
only runs on a machine that has the same Homebrew packages installed. This
copies every non-system dependency into Contents/Frameworks, rewrites the load
commands to @rpath, and adds an rpath pointing at that folder.

Usage: bundle_macos_dylibs.py path/to/App.app [executable-name]
"""

import os
import shutil
import subprocess
import sys

# Paths under these prefixes ship with macOS and must NOT be copied.
SYSTEM_PREFIXES = ("/usr/lib", "/System/")


def dependencies(binary):
    """Non-system dylibs this binary links against."""
    out = subprocess.check_output(["otool", "-L", binary]).decode()
    deps = []
    for line in out.splitlines()[1:]:
        path = line.strip().split(" ")[0]
        if not path or path.startswith("@"):
            continue
        if path.startswith(SYSTEM_PREFIXES):
            continue
        deps.append(path)
    return deps


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    app = sys.argv[1].rstrip("/")
    exe_name = sys.argv[2] if len(sys.argv) > 2 else "oscilloplot"
    exe = os.path.join(app, "Contents", "MacOS", exe_name)
    frameworks = os.path.join(app, "Contents", "Frameworks")

    if not os.path.isfile(exe):
        print(f"error: no executable at {exe}", file=sys.stderr)
        return 1

    os.makedirs(frameworks, exist_ok=True)

    # Walk the dependency graph; queue holds source paths still to copy.
    collected = {}          # basename -> destination path
    queue = dependencies(exe)
    while queue:
        src = queue.pop()
        name = os.path.basename(src)
        if name in collected:
            continue
        if not os.path.exists(src):
            print(f"warning: missing dependency {src}", file=sys.stderr)
            continue
        dst = os.path.join(frameworks, name)
        shutil.copy2(src, dst)
        os.chmod(dst, 0o755)
        collected[name] = dst
        queue += dependencies(dst)

    # Give each copied library an @rpath identity...
    for name, dst in collected.items():
        subprocess.run(["install_name_tool", "-id", f"@rpath/{name}", dst],
                       check=True)

    # ...then repoint every reference, in the exe and between the libraries.
    for binary in [exe] + list(collected.values()):
        for src in dependencies(binary):
            name = os.path.basename(src)
            if name in collected:
                subprocess.run(
                    ["install_name_tool", "-change", src, f"@rpath/{name}", binary],
                    check=True)

    # The loader needs to know where @rpath points.
    subprocess.run(
        ["install_name_tool", "-add_rpath", "@executable_path/../Frameworks", exe],
        check=False)   # harmless failure if the rpath is already present

    print(f"bundled {len(collected)} librar{'y' if len(collected) == 1 else 'ies'}:")
    for name in sorted(collected):
        print(f"  {name}")

    # Verify nothing external survived.
    leftovers = dependencies(exe)
    if leftovers:
        print("error: executable still references external paths:", file=sys.stderr)
        for path in leftovers:
            print(f"  {path}", file=sys.stderr)
        return 1

    print("bundle is self-contained")
    return 0


if __name__ == "__main__":
    sys.exit(main())
