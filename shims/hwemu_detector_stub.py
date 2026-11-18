"""The sim's stand-in for otto's armor detector.

otto's IconTargetDetector runs its ONNX model through CUDA and TensorRT only
(CUDA tensors and IO bindings throughout), so it can't run, or be tested, on
a machine without a GPU. In a sim run (HWEMU_RUNTIME_DIR set),
sitecustomize puts this class in its place: it touches neither torch nor
CUDA, detects nothing, and reports the sim gap `detector: disabled` once.
Everything else in otto -- cameras, odometry, lidar, autonav, comms --
keeps running, and scripts/smoke.sh fails on the gap, so a run never looks
like working vision when it isn't.

otto's own detector code is untouched: patches/project-otto.patch has no
detector changes.
"""

from __future__ import annotations

from typing import Any, List, Set


def make_stub(target_detector_base: type) -> type:
    """A TargetDetector subclass (built on otto's own ABC) that detects nothing."""
    from hwemu import gaps

    class HwemuDisabledDetector(target_detector_base):  # type: ignore[misc, valid-type]
        IS_HWEMU_STUB = True

        def __init__(self, model_path: str, confidence_threshold: float = 0.5, *_: Any, **__: Any):
            self._model_path = model_path
            gaps.report(
                "detector",
                "disabled",
                "otto's armor detector is CUDA/TensorRT-only and isn't run in the sim: a stub stands in "
                f"for it (model {model_path} not loaded) and reports no detections, so auto-aim never "
                "sees a target",
            )

        def detect_targets(self, imagesets: List[Any]) -> List[Set[Any]]:
            return [set() for _ in imagesets]

        def close(self) -> None:
            pass

        def __enter__(self) -> "HwemuDisabledDetector":
            return self

        def __exit__(self, *_: Any) -> None:
            self.close()

    HwemuDisabledDetector.__name__ = HwemuDisabledDetector.__qualname__ = "IconTargetDetector"
    return HwemuDisabledDetector
