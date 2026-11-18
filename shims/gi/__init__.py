"""A PyGObject (`gi`) stand-in for the sim: just enough of `gi.repository.Gst`
for project-otto, backed by hw-emulator's `arducam` devices. See
gi/repository/Gst.py. scripts/run.sh puts aruw-sim's shims/ first on otto's
PYTHONPATH, so `import gi` gets this instead of the real package."""

IS_HWEMU_SHIM = True
version_info = (3, 42, 1)
_versions: dict = {}


def require_version(namespace: str, version: str) -> None:
    if namespace != "Gst" or version != "1.0":
        from hwemu import gaps

        gaps.report("gi", f"require:{namespace}", f"gi.require_version({namespace!r}, {version!r}): the sim's gi "
                                                  f"shim only has Gst 1.0")
    _versions[namespace] = version
