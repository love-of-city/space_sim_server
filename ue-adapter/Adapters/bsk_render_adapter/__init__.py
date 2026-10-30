"""Public API for the renderer-neutral Basilisk visualization adapter."""

__version__ = "0.2.0"

from .bridge import BasiliskRenderBridge
from .descriptors import (
    CameraVisual,
    CelestialBodyVisual,
    GeometryVisual,
    SceneSettings,
    VisualElement,
)
from .protocol import (
    PROTOCOL_V2,
    RecordingOnlyPublisher,
    RenderPublisher,
    decode_packet,
    encode_packet,
)
from .mjcf_assets import (
    load_asset_catalog,
    parse_mjcf_body_parents,
    parse_mjcf_geometry_metadata,
    parse_mjcf_scene_metadata,
)
from .recording import BskRecordingReader, BskRecordingWriter
from .capture import (
    CAPTURE_PROTOCOL,
    decode_capture_payload,
    receive_capture_packet,
    save_capture_frame,
)
from .stl_conversion import StlConversionResult, convert_stl_to_obj, prepare_stl_for_import, read_stl
from .ue_support import (
    enable_unreal_visualization,
    enableUnrealVisualization,
    setActuatorGuiSetting,
    setInstrumentGuiSetting,
)

__all__ = [
    "BasiliskRenderBridge",
    "BskRecordingReader",
    "BskRecordingWriter",
    "CameraVisual",
    "CAPTURE_PROTOCOL",
    "CelestialBodyVisual",
    "GeometryVisual",
    "PROTOCOL_V2",
    "RecordingOnlyPublisher",
    "RenderPublisher",
    "SceneSettings",
    "StlConversionResult",
    "VisualElement",
    "__version__",
    "decode_packet",
    "decode_capture_payload",
    "convert_stl_to_obj",
    "enable_unreal_visualization",
    "enableUnrealVisualization",
    "encode_packet",
    "load_asset_catalog",
    "parse_mjcf_geometry_metadata",
    "parse_mjcf_body_parents",
    "parse_mjcf_scene_metadata",
    "prepare_stl_for_import",
    "read_stl",
    "receive_capture_packet",
    "save_capture_frame",
    "setActuatorGuiSetting",
    "setInstrumentGuiSetting",
]
