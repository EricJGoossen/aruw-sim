"""A GStreamer stand-in backed by hw-emulator's simulated Arducams.

What project-otto does with GStreamer, and what this does instead:

- **Arducam capture** (`ArducamImagesetSource`): `parse_launch("v4l2src
  device=/dev/v4l/by-path/<id> ! ... ! video/x-raw,format=BGRx,... ! appsink
  name=sink ...")`. The hub's `arducam` device for that V4L2 id writes
  <runtime dir>/arducam/<id>.json and a BGRx frame ring in /dev/shm; once the
  pipeline is PLAYING, a thread here waits for each new frame and emits the
  appsink's `new-sample`, whose buffer maps to the frame's BGRx bytes. The
  caps' format, width and height are checked against the hub's.
- **Video dumping** (`HWAccelVideoDumper`): appsrc -> videoconvert ->
  nvvidconv -> nvv4l2h264enc -> h264parse -> matroskamux -> filesink. The
  Jetson's encoder isn't simulated: every element is accepted, buffers pushed
  into the appsrc are dropped, and `end-of-stream` posts EOS on the bus, so
  the dumper starts and stops as on the robot. Reported as a gap.

Anything else in a pipeline (an element or a property this doesn't model) is
reported as a sim gap, never ignored silently.
"""

from __future__ import annotations

import enum
import json
import os
import re
import struct
import threading
from typing import Any, Callable, Optional

from hwemu import gaps
from hwemu.frame_shm import FrameShmReader
from hwemu.serial import runtime_dir

IS_HWEMU_SHIM = True


def init(_args: Any = None) -> None:
    pass


def is_initialized() -> bool:
    return True


class State(enum.IntEnum):
    VOID_PENDING = 0
    NULL = 1
    READY = 2
    PAUSED = 3
    PLAYING = 4


class StateChangeReturn(enum.IntEnum):
    FAILURE = 0
    SUCCESS = 1
    ASYNC = 2
    NO_PREROLL = 3


class FlowReturn(enum.IntEnum):
    OK = 0
    NOT_LINKED = -1
    FLUSHING = -2
    EOS = -3
    ERROR = -5


class MapFlags(enum.IntFlag):
    READ = 1
    WRITE = 2


class Format(enum.IntEnum):
    UNDEFINED = 0
    DEFAULT = 1
    BYTES = 2
    TIME = 3
    BUFFERS = 4


class MessageType(enum.IntFlag):
    UNKNOWN = 0
    EOS = 1 << 0
    ERROR = 1 << 1
    WARNING = 1 << 2
    INFO = 1 << 3
    STATE_CHANGED = 1 << 6
    STREAM_STATUS = 1 << 13


CLOCK_TIME_NONE = 2**64 - 1
SECOND = 1_000_000_000


class Caps:
    def __init__(self, text: str):
        self._text = text.strip()
        self.fields: dict[str, str] = {}
        parts = [p.strip() for p in self._text.split(",")]
        self.media = parts[0] if parts else ""
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                v = re.sub(r"^\(\w+\)", "", v.strip())
                self.fields[k.strip()] = v

    @staticmethod
    def from_string(text: str) -> "Caps":
        return Caps(text)

    def to_string(self) -> str:
        return self._text


class MapInfo:
    def __init__(self, data: memoryview):
        self.data = data
        self.size = len(data)


class Buffer:
    def __init__(self, data: bytes):
        self._data = data
        self.pts = CLOCK_TIME_NONE
        self.dts = CLOCK_TIME_NONE
        self.duration = CLOCK_TIME_NONE
        self.offset = 0
        self.offset_end = 0

    @staticmethod
    def new_wrapped(data: bytes) -> "Buffer":
        return Buffer(data)

    def map(self, _flags: MapFlags) -> tuple[bool, MapInfo]:
        return True, MapInfo(memoryview(self._data))

    def unmap(self, _info: MapInfo) -> None:
        pass

    def get_size(self) -> int:
        return len(self._data)


class Sample:
    def __init__(self, buffer: Buffer, caps: Optional[Caps] = None):
        self._buffer = buffer
        self._caps = caps

    def get_buffer(self) -> Buffer:
        return self._buffer

    def get_caps(self) -> Optional[Caps]:
        return self._caps


class Message:
    def __init__(self, type_: MessageType, src: "Element", **data: Any):
        self.type = type_
        self.src = src
        self._data = data

    def parse_state_changed(self):
        return self._data["old"], self._data["new"], self._data["pending"]

    def parse_error(self):
        return self._data["error"], self._data.get("debug", "")

    def parse_warning(self):
        return self._data["error"], self._data.get("debug", "")

    def parse_stream_status(self):
        return self._data["status"], self._data["owner"]


class Bus:
    def __init__(self):
        self._messages: list[Message] = []
        self._lock = threading.Lock()

    def post(self, message: Message) -> None:
        with self._lock:
            self._messages.append(message)

    def pop(self) -> Optional[Message]:
        with self._lock:
            return self._messages.pop(0) if self._messages else None

    def timed_pop(self, _timeout: int) -> Optional[Message]:
        return self.pop()

    def timed_pop_filtered(self, _timeout: int, types: MessageType) -> Optional[Message]:
        with self._lock:
            for i, m in enumerate(self._messages):
                if m.type & types:
                    return self._messages.pop(i)
        return None


# Elements: what each accepts. Properties not listed are reported as gaps.
_KNOWN = {
    "pipeline": set(),
    "v4l2src": {"device", "io-mode", "do-timestamp"},
    "nvv4l2decoder": {"mjpeg", "enable-max-performance"},
    "nvvidconv": {"flip-method"},
    "videoconvert": set(),
    "capsfilter": {"caps"},
    "appsink": {"name", "max-buffers", "drop", "emit-signals", "sync", "caps"},
    "appsrc": {"format", "caps", "max-bytes", "is-live", "do-timestamp", "block"},
    "nvv4l2h264enc": {"maxperf-enable", "bitrate", "preset-level", "insert-sps-pps", "iframeinterval"},
    "h264parse": set(),
    "matroskamux": set(),
    "filesink": {"location", "sync"},
    "queue": {"max-size-buffers", "leaky"},
}
_ENCODE_CHAIN = {"appsrc", "nvvidconv", "nvv4l2h264enc", "h264parse", "matroskamux", "filesink"}


class Element:
    def __init__(self, factory: str, name: Optional[str] = None):
        self.factory = factory
        self._name = name or f"{factory}0"
        self._props: dict[str, Any] = {}
        self._handlers: dict[str, list[Callable]] = {}
        self.pipeline: Optional["Pipeline"] = None
        self.downstream: Optional["Element"] = None
        if factory not in _KNOWN:
            gaps.report("gst", f"element:{factory}", f"GStreamer element '{factory}' isn't simulated; it passes "
                                                     f"nothing through")

    def get_name(self) -> str:
        return self._name

    def set_property(self, key: str, value: Any) -> None:
        if key not in _KNOWN.get(self.factory, set()) and key != "name":
            gaps.report("gst", f"prop:{self.factory}:{key}",
                        f"GStreamer {self.factory} property '{key}' isn't simulated; ignored")
        if key == "name":
            self._name = str(value)
        self._props[key] = value

    def get_property(self, key: str) -> Any:
        return self._props.get(key)

    def connect(self, signal: str, handler: Callable, *args: Any) -> int:
        self._handlers.setdefault(signal, []).append(lambda *a: handler(*a, *args))
        return len(self._handlers[signal])

    def _fire(self, signal: str, *args: Any) -> Any:
        result = None
        for h in self._handlers.get(signal, []):
            result = h(self, *args)
        return result

    def emit(self, signal: str, *args: Any) -> Any:
        if self.factory == "appsrc":
            if signal == "push-buffer":
                return FlowReturn.OK  # the encoder isn't simulated: dropped
            if signal == "end-of-stream":
                if self.pipeline is not None:
                    self.pipeline.bus.post(Message(MessageType.EOS, self.pipeline))
                return FlowReturn.OK
        if self.factory == "appsink" and signal == "pull-sample":
            return self._pull()
        gaps.report("gst", f"emit:{self.factory}:{signal}",
                    f"GStreamer {self.factory} signal '{signal}' isn't simulated")
        return None

    def link(self, dest: "Element") -> bool:
        self.downstream = dest
        return True

    def link_filtered(self, dest: "Element", _caps: Caps) -> bool:
        return self.link(dest)

    # appsink: the newest sample, set by the pipeline's frame thread.
    _sample: Optional[Sample] = None

    def _pull(self) -> Optional[Sample]:
        s, self._sample = self._sample, None
        return s

    @staticmethod
    def state_get_name(state: State) -> str:
        return State(state).name


class ElementFactory:
    @staticmethod
    def make(factory: str, name: Optional[str] = None) -> Element:
        return Element(factory, name)


class Pipeline(Element):
    def __init__(self, name: str = "pipeline"):
        super().__init__("pipeline", name)
        self.elements: list[Element] = []
        self.bus = Bus()
        self.state = State.NULL
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._source: Optional[dict] = None  # the arducam this pipeline reads
        self._caps: Optional[Caps] = None

    @staticmethod
    def new(name: str) -> "Pipeline":
        return Pipeline(name)

    def add(self, element: Element) -> bool:
        element.pipeline = self
        self.elements.append(element)
        return True

    def get_by_name(self, name: str) -> Optional[Element]:
        for e in self.elements:
            if e.get_name() == name:
                return e
        return None

    def get_bus(self) -> Bus:
        return self.bus

    def get_state(self, _timeout: int):
        return StateChangeReturn.SUCCESS, self.state, State.VOID_PENDING

    def set_state(self, state: State) -> StateChangeReturn:
        old = self.state
        self.state = State(state)
        self.bus.post(Message(MessageType.STATE_CHANGED, self, old=old, new=self.state, pending=State.VOID_PENDING))
        factories = {e.factory for e in self.elements}
        if self.state == State.PLAYING and factories & _ENCODE_CHAIN and "appsrc" in factories:
            gaps.report("gst", "video-dump", "video dumping isn't simulated: the Jetson's NVENC pipeline "
                                             "(nvv4l2h264enc -> matroskamux -> filesink) is accepted, and the "
                                             "frames pushed into it are dropped; no video is written")
        if self.state == State.PLAYING and self._source is not None and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name=f"gst-{self._name}", daemon=True)
            self._thread.start()
        elif self.state != State.PLAYING and self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=1.0)
            self._thread = None
        return StateChangeReturn.SUCCESS

    def _sink(self) -> Optional[Element]:
        for e in self.elements:
            if e.factory == "appsink":
                return e
        return None

    def _run(self) -> None:
        # Polls on wall time: this is the shim's own wait, not otto's.
        from hwemu.clock import _real_sleep as sleep

        src = self._source
        assert src is not None
        reader: Optional[FrameShmReader] = None
        last = 0
        sink = self._sink()
        while not self._stop.is_set():
            if reader is None:
                try:
                    reader = FrameShmReader(src["shm"])
                except (FileNotFoundError, ValueError):
                    sleep(0.05)
                    continue
            frame = reader.read_latest(last)
            if frame is None:
                sleep(0.001)
                continue
            last = frame.sequence
            _ver, width, height, fmt = struct.unpack_from("<IIII", frame.meta, 0)
            if (width, height) != (src["width"], src["height"]) or fmt != 2:
                gaps.report("gst", "arducam-format", f"arducam frame {width}x{height} format {fmt} doesn't match "
                                                     f"what the pipeline asked for; dropped")
                continue
            buf = Buffer(frame.payload)
            capture = struct.unpack_from("<d", frame.meta, 24)[0]
            buf.pts = int(capture * SECOND)
            if sink is not None:
                sink._sample = Sample(buf, self._caps)
                if sink.get_property("emit-signals"):
                    sink._fire("new-sample")


def _arducam_for(device: str) -> dict:
    """The hub's arducam serving V4L2 `device` (/dev/v4l/by-path/<id>)."""
    v4l = os.path.basename(device)
    path = os.path.join(runtime_dir(), "arducam", f"{v4l}.json")
    if not os.path.exists(path):
        raise RuntimeError(f"[hwemu] no simulated Arducam for {device}: the hub has no arducam device with "
                           f"v4l '{v4l}' ({path} is missing)")
    with open(path) as f:
        return json.load(f)


def parse_launch(description: str) -> Pipeline:
    """A pipeline from gst-launch syntax: elements and caps separated by `!`."""
    pipe = Pipeline("pipeline0")
    prev: Optional[Element] = None
    caps: Optional[Caps] = None
    for chunk in (c.strip() for c in description.replace("\n", " ").split("!")):
        if not chunk:
            continue
        if chunk.startswith(("video/", "audio/", "image/")):
            caps = Caps(re.sub(r",\s+", ",", chunk))
            continue
        tokens = re.findall(r'(?:[^\s"]|"[^"]*")+', chunk)
        factory, props = tokens[0], tokens[1:]
        el = Element(factory)
        pipe.add(el)
        for p in props:
            if "=" not in p:
                gaps.report("gst", f"token:{p}", f"GStreamer launch token '{p}' not understood; ignored")
                continue
            k, v = p.split("=", 1)
            v = v.strip('"')
            el.set_property(k, {"True": True, "true": True, "False": False, "false": False}.get(v, v))
        if prev is not None:
            prev.link(el)
        prev = el
        if factory == "v4l2src":
            pipe._source = _arducam_for(el.get_property("device") or "")
    if pipe._source is not None:
        src = pipe._source
        if caps is not None:
            want = (caps.fields.get("format"), caps.fields.get("width"), caps.fields.get("height"))
            have = (src.get("format"), str(src["width"]), str(src["height"]))
            if want != have:
                raise RuntimeError(f"[hwemu] the pipeline asks for {want} but the sim's Arducam serves {have}")
            if "framerate" in caps.fields:
                gaps.report("gst", "arducam-framerate", f"the Arducam pipeline's framerate "
                                                        f"({caps.fields['framerate']}) comes from the sim camera's "
                                                        f"period in devices.yaml, not from the caps")
        pipe._caps = caps
    return pipe
