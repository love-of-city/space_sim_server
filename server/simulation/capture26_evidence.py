"""Append complete evidence while bounding a long flight's working memory."""
from collections import deque
import json


class EvidenceRows:
    def __init__(self, path, *, retain=0):
        self.file = path.open('x', encoding='utf-8', buffering=256*1024)
        self.rows = deque(maxlen=retain)
        self.count = 0

    def append(self, row):
        self.file.write(json.dumps(row, default=lambda a: a.tolist())+'\n')
        self.rows.append(row)
        self.count += 1
        if self.count % 20 == 0:
            self.file.flush()

    def __iter__(self):
        return iter(self.rows)

    def __getitem__(self, key):
        return self.rows[key]

    def close(self):
        self.file.close()
