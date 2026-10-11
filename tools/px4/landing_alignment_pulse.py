"""Declared short pixel translation at the first final-descent acquisition."""
from dataclasses import replace
import hashlib
import json

import cv2
import numpy as np


def pulsed_runtime(base, shift_px):
    if not 0 < shift_px <= 25:
        raise ValueError('Explicit bounded downward-image translation required')
    class PulsedRuntime(base):
        def __enter__(self):
            result = super().__enter__()
            camera = self.return_camera.camera
            self.pulse_start = None
            self.pulse_records = []
            runtime = self
            class PixelPulse:
                def capture(self, data, timestamp_us):
                    frame = camera.capture(data, timestamp_us)
                    now = timestamp_us / 1e6
                    if (frame is not None and runtime.pulse_start is not None and
                            runtime.return_phase == 'FINAL_DESCEND' and now-runtime.pulse_start <= 1.):
                        image = cv2.warpAffine(frame.image_rgb,
                            np.array([[1., 0., shift_px], [0., 1., 0.]]),
                            (frame.image_rgb.shape[1], frame.image_rgb.shape[0]),
                            flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
                        runtime.pulse_records.append(dict(source_time_us=frame.timestamp_us,
                            received_time_us=timestamp_us, phase=runtime.return_phase,
                            shift_px=shift_px,
                            original_sha256=hashlib.sha256(frame.image_rgb.tobytes()).hexdigest(),
                            delivered_sha256=hashlib.sha256(image.tobytes()).hexdigest()))
                        frame = replace(frame, image_rgb=image)
                    return frame
                def close(self):
                    camera.close()
            self.return_camera.camera = PixelPulse()
            return result

        def step(self, timeout=5.):
            height = -self.telemetry.get('local_position',[0.,0.,-1.])[2]
            if (self.return_phase == 'FINAL_DESCEND' and self.pulse_start is None and
                    0 < height <= .121 and
                    0 <= float(self.plant.data.time)-self.telemetry.get('local_time',-100) <= .2):
                self.pulse_start = float(self.plant.data.time)
                # Request a real new acquisition at admission; no cached image
                # or rewritten observation can substitute for the pixel fault.
                self.return_camera.next_image = self.pulse_start
            return super().step(timeout)

        def close(self):
            if hasattr(self, 'pulse_records'):
                (self.output / 'landing_pixel_pulse.json').write_text(json.dumps(dict(
                    records=self.pulse_records, onset_s=self.pulse_start, maximum_duration_s=1.,
                    scope='Development camera pixel displacement; no truth or telemetry modification'), indent=2)+'\n')
            return super().close()
    return PulsedRuntime
