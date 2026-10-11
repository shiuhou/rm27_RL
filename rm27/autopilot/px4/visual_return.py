"""Fresh assigned-slot image admission for the PX4 return/landing state machine."""
import numpy as np


class VisualReturnGuard:
    def __init__(self, assigned_tag_id, *, maximum_age_s=.2, maximum_correction_m=.25):
        self.assigned_tag_id=int(assigned_tag_id)
        self.maximum_age_s=maximum_age_s
        self.maximum_correction_m=maximum_correction_m
        self.source_time=None
        self.correction=None
        self.position=None
        self.rejected={}

    def _reject(self, reason):
        self.rejected[reason]=self.rejected.get(reason,0)+1
        return False

    def observe(self, row, now, position_nwu_xy, *, telemetry_age_s):
        if not row or not row.get('valid'):
            return False
        if row.get('assigned_tag_id')!=self.assigned_tag_id:
            return self._reject('wrong assigned ID')
        stamp=row.get('source_time_s',float('nan'))
        if not np.isfinite(stamp) or not 0<=now-stamp<=self.maximum_age_s:
            return self._reject('image time')
        if self.source_time is not None and stamp<=self.source_time:
            return False  # Cached images never extend observation freshness.
        if not 0<=telemetry_age_s<=self.maximum_age_s:
            return self._reject('telemetry time')
        measured=np.asarray(position_nwu_xy,dtype=float)
        image=np.asarray(row.get('position_relative_slot_nwu_xy_m',[]),dtype=float)
        if measured.shape!=(2,) or image.shape!=(2,) or not np.isfinite(np.r_[measured,image]).all():
            return self._reject('nonfinite position')
        correction=measured-image
        if np.linalg.norm(correction)>self.maximum_correction_m:
            return self._reject('registration bound')
        self.source_time=float(stamp)
        self.position=image.copy()
        self.correction=correction
        return True

    def fresh(self, now):
        return self.source_time is not None and 0<=now-self.source_time<=self.maximum_age_s

    def aligned(self, now, tolerance_m=.003):
        return self.fresh(now) and np.linalg.norm(self.position)<=tolerance_m

