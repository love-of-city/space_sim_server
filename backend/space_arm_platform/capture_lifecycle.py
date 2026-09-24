"""On-demand capture transitions, shared by episode/task/shutdown entry points."""
from __future__ import annotations

import asyncio

from .models import EpisodeStart, EpisodeStop


class CaptureLifecycle:
    def __init__(self, hub, recorder):
        self.hub = hub
        self.recorder = recorder
        self._lock = asyncio.Lock()
        self._gated_episode = None

    @property
    def busy(self):
        return self._lock.locked()

    async def start(self, request: EpisodeStart):
        async with self._lock:
            if self.hub.resetting:
                raise RuntimeError("场景正在重置，暂不能开始采集")
            gated = bool(request.camera_ids)
            if gated and (not self.hub.connected or not self.hub.capture_on_demand_supported):
                raise RuntimeError("仿真未连接或不支持按需采集，请更新并重启场景")
            preparation = asyncio.create_task(asyncio.to_thread(
                self.recorder.start, request, capture_on_demand=gated))
            try:
                metadata = await asyncio.shield(preparation)
            except asyncio.CancelledError:
                # A cancelled HTTP request cannot abandon a still-running writer
                # preparation thread and leave an orphan active episode behind.
                try:
                    await preparation
                except Exception:
                    pass
                else:
                    await asyncio.to_thread(self.recorder.stop,
                        EpisodeStop(outcome="aborted", note="capture start cancelled"))
                raise
            if not gated:
                return metadata
            self._gated_episode = metadata["episode_id"]
            try:
                await self.hub.set_capture_episode(self._gated_episode)
            except (Exception, asyncio.CancelledError):
                await asyncio.to_thread(self.recorder.freeze_observations)
                self.recorder.fail("start capture was not confirmed; episode aborted")
                try:
                    await self.hub.set_capture_episode("", timeout=2.)
                except RuntimeError:
                    pass  # The OFF request is retained and replayed on reconnect.
                await asyncio.to_thread(self.recorder.stop, EpisodeStop(outcome="aborted", note="capture start failed"))
                self._gated_episode = None
                raise
            return metadata

    async def stop(self, request: EpisodeStop):
        # Drain and OFF must finish even if the HTTP client disconnects.
        task = asyncio.create_task(self._stop(request))
        try:
            return await asyncio.shield(task)
        except asyncio.CancelledError:
            await task
            raise

    async def _stop(self, request: EpisodeStop):
        async with self._lock:
            await asyncio.to_thread(self.recorder.freeze_observations)
            if self._gated_episode:
                try:
                    await self.hub.set_capture_episode("")
                except RuntimeError as error:
                    self.recorder.fail(str(error))
            # The cutoff is already frozen; pending RGB still matches accepted
            # observations while the renderer drains its reliable FIFO.
            result = await asyncio.to_thread(self.recorder.stop, request)
            self._gated_episode = None
            return result
