"""Consumer-side freshness contract. Not a hardware watchdog."""
from math import isfinite
from .core import Status


class VerdictLease:
    def __init__(self, ttl_s=0.5):
        if not isfinite(ttl_s) or ttl_s <= 0:
            raise ValueError("Invalid verdict TTL")
        self.ttl_s = ttl_s
        self.status = None
        self.received_s = None

    def receive(self, status, at_s):
        if not isinstance(status, Status) or not isfinite(at_s):
            return False
        if self.received_s is not None and at_s < self.received_s:
            return False
        self.status = status
        self.received_s = at_s
        return True

    def is_current_stop(self, now_s):
        return (
            isfinite(now_s)
            and self.status is Status.STOP_EVIDENCED
            and self.received_s is not None
            and 0 <= now_s - self.received_s <= self.ttl_s
        )
