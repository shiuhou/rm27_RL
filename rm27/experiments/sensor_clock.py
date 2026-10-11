"""Acquisition clocks for asynchronously delivered camera and IMU streams."""


class InputClock:
    def __init__(self):
        self.previous={}

    def observe(self,kind,timestamp_ns):
        if kind not in ('camera','imu') or type(timestamp_ns) is not int or timestamp_ns<0:
            raise ValueError('supported sensor and non-negative integer acquisition time required')
        if timestamp_ns<=self.previous.get(kind,-1):
            raise ValueError(kind+' acquisition time repeated or reversed')
        self.previous[kind]=timestamp_ns
