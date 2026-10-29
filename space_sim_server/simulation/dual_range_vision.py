"""Image quality gates; extra-camera experiments withdrawn on 2026-10-29."""
from dataclasses import dataclass

import numpy as np


def long_focus_candidate(near, manifest, *, fov_deg=12.):
    """Reject the withdrawn optical candidate before changing either input."""
    raise ValueError('Additional cameras withdrawn by user on 2026-10-29; use original stereo only')


@dataclass(frozen=True)
class ImageGateLimits:
    samples: int = 10
    max_age_s: float = .35
    max_gap_s: float = .15
    minimum_side_px: float = 24.
    depth_sigma_m: float = .03
    navigation_sigma_m: float = .04

    def __post_init__(self):
        if isinstance(self.samples, bool) or not isinstance(self.samples, int) or self.samples < 1:
            raise ValueError('positive integer image count required')
        vals = [v for k, v in vars(self).items() if k != 'samples']
        if not np.isfinite(vals).all() or min(vals) <= 0:
            raise ValueError('positive finite image gate limits required')


class AcceptedImageGate:
    """Only distinct, fresh, image-derived and filter-accepted samples count.

    Pixel size is the minimum edge of an actually used complete marker in EACH
    eye. Model projections never count as detections. Rejection resets the
    streak; a stream must acquire its own streak before range transfer.
    """
    def __init__(self, limits=ImageGateLimits()):
        self.limits = limits
        self.count = 0
        self.last_stamp = self.last_receive = self.last_frame = None

    def observe(self, *, now_s, stamp_s, frame_id, image_accepted, ekf_accepted,
                used_marker_edges_px, depth_sigma_m, navigation_sigma_m):
        vals = np.asarray([now_s, stamp_s, depth_sigma_m, navigation_sigma_m], float)
        sides = np.asarray([used_marker_edges_px.get(e, np.nan) for e in ('left', 'right')], float)
        distinct = (self.last_stamp is None or
                    (stamp_s > self.last_stamp and frame_id > self.last_frame))
        monotonic = self.last_receive is None or now_s > self.last_receive
        valid = bool(np.isfinite(vals).all() and np.isfinite(sides).all()
                     and 0 <= now_s-stamp_s <= self.limits.max_age_s
                     and np.all(sides >= self.limits.minimum_side_px)
                     and 0 < depth_sigma_m < self.limits.depth_sigma_m
                     and 0 <= navigation_sigma_m < self.limits.navigation_sigma_m
                     and image_accepted and ekf_accepted and distinct and monotonic)
        uninterrupted = bool(self.last_receive is not None and
                             0 < now_s-self.last_receive <= self.limits.max_gap_s+1e-9
                             and 0 < stamp_s-self.last_stamp <= self.limits.max_gap_s+1e-9)
        self.count = (self.count+1 if uninterrupted else 1) if valid else 0
        # Keep monotonic high-water marks, including rejected fresh frames.
        if np.isfinite([now_s, stamp_s]).all() and distinct:
            self.last_receive, self.last_stamp, self.last_frame = now_s, stamp_s, frame_id
        return self.current(now_s, navigation_sigma_m)

    def current(self, now_s, navigation_sigma_m):
        fresh = bool(self.last_stamp is not None and np.isfinite([now_s, navigation_sigma_m]).all()
                     and 0 <= now_s-self.last_stamp <= self.limits.max_age_s
                     and 0 <= now_s-self.last_receive <= self.limits.max_gap_s+1e-9
                     and 0 <= navigation_sigma_m < self.limits.navigation_sigma_m)
        if not fresh:
            self.count = 0
        return fresh and self.count >= self.limits.samples


class RangeChannelHandoff:
    """Transfer far -> near only after the near pair has independently passed.

    Channel health must be checked every control tick. A lost active channel
    requests hold immediately; this object cannot disable mission protections.
    There is no distance-only automatic switch or continuation on stale poses.
    """
    def __init__(self):
        self.active = 'far'

    def update(self, *, far_ready, near_ready):
        if near_ready:
            self.active = 'near'
        ready = near_ready if self.active == 'near' else far_ready
        return dict(active=self.active, visual_ready=bool(ready), hold_required=not ready)
