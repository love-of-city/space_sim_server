"""Nonblocking, bounded localhost channel from image worker to control process.

The outer fixed-delivery scheduler handles missing results and process health.
The receiver checks task/session/calibration before exposing a result to navigation.
Image bytes and simulator truth do not travel on this result channel.
"""
import json
import socket


class VisionResultSender:
    def __init__(self, port):
        self.port = int(port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.dropped = 0

    def __call__(self, result):
        data = json.dumps(result, allow_nan=False, separators=(',', ':')).encode('utf-8')
        if len(data) > 60_000:
            self.dropped += 1
            return
        try:
            self.socket.sendto(data, ('127.0.0.1', self.port))
        except OSError:
            self.dropped += 1

    def close(self):
        self.socket.close()


class VisionResultReceiver:
    def __init__(self, mission_id, session_id, calibration_id, *, port=0):
        self.identity = (str(mission_id), str(session_id), str(calibration_id))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', int(port)))
        self.socket.setblocking(False)
        self.port = self.socket.getsockname()[1]
        self.rejected = 0
        self.last_error = None
        self.invalid_events = []

    def poll(self):
        results = []
        # Bound work per control tick, including malformed datagrams.
        for _ in range(32):
            try:
                data = self.socket.recv(65_536)
            except BlockingIOError:
                break
            result = {}
            try:
                result = json.loads(data)
                if tuple(result[k] for k in ('mission_id', 'session_id', 'calibration_id')) != self.identity:
                    raise ValueError('vision task/session/calibration mismatch')
                int(result['sim_time_ns'])
                if not str(result['source_frame_id']):
                    raise ValueError('missing source frame')
                results.append(result)
            except (ValueError, KeyError, TypeError) as exc:
                self.rejected += 1
                self.last_error = str(exc)
                self.invalid_events.append(dict(result if isinstance(result, dict) else {},
                    status='rejected', reason=self.last_error, transport_rejection=True))
        return results

    def drain_invalid(self):
        events, self.invalid_events = self.invalid_events, []
        return events

    def close(self):
        self.socket.close()
