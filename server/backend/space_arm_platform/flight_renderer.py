"""Start the local Linux renderer for flight using the shared rendering contract."""
import subprocess
from types import SimpleNamespace

from .collection_runtime import renderer_command, renderer_environment


def start_renderer(config, output, render_port):
    from collection.runtime_provenance import load_build_identity
    adapter = config.runtime_adapter_root or config.project_root.parent / 'ue-adapter'
    engine = config.runtime_unreal_root
    if engine is None:
        raise RuntimeError('Configure the platform UE installation before a live flight')
    project = adapter / 'Unreal/BskUnrealRenderer'
    load_build_identity(adapter).verify_build_manifest(project, engine)
    output.mkdir(parents=True, exist_ok=True)
    args = SimpleNamespace(adapter=adapter, unreal=engine, render_port=render_port,
                           capture_port=config.capture_port, signalling_port=config.runtime_pixel_streamer_port,
                           offline=False)
    command = renderer_command(args, output)
    prefixes = ('-BskCaptureProducts=', '-BskCaptureRate=', '-PixelStreamingID=', '-BskPixelStreamingBaseId=',
                '-BskPixelStreamingCameras=', '-PixelStreamingWebRTCFps=', '-BskPixelStreamingCameraFps=',
                '-BskPixelStreamingCameraWidth=', '-BskPixelStreamingCameraHeight=')
    command = [part for part in command if not part.startswith(prefixes)]
    cameras = ['team/' + camera.removeprefix('teleop/') for camera in config.runtime_pixel_streaming_camera_ids]
    command += [f'-PixelStreamingID={config.pixel_streaming_streamer_id}',
                f'-BskPixelStreamingBaseId={config.pixel_streaming_streamer_id}',
                '-BskPixelStreamingCameras=' + '+'.join(cameras),
                f'-PixelStreamingWebRTCFps={config.runtime_preview_rate}',
                f'-BskPixelStreamingCameraFps={config.runtime_preview_rate}',
                f'-BskPixelStreamingCameraWidth={config.runtime_pixel_streaming_camera_width}',
                f'-BskPixelStreamingCameraHeight={config.runtime_pixel_streaming_camera_height}',
                f'-PixelStreamingEncoderMinQuality={config.runtime_encoder_min_quality}']
    with (output / 'renderer-start.log').open('ab') as log:
        process = subprocess.Popen(command, cwd=config.project_root, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, env=renderer_environment(args))
    (project / 'Saved').mkdir(exist_ok=True)
    (project / 'Saved/BskRenderer.pid').write_text(str(process.pid))
    return process
