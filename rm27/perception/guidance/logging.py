"""GuidanceLogger - machine-readable JSONL logging for V0-V2.

One record per guidance update, containing the TargetEstimate input
summary and the GuidanceCommand output, so that later analysis can plot:

    yaw error -> yaw-rate command -> actual yaw response
"""

import json


class GuidanceLogger:
    def __init__(self, path):
        self._file = open(path, "a", encoding="utf-8")

    def log(self, now_us, estimate, output):
        record = {
            "now_us": now_us,
            "target": None,
            "guidance": {
                "state": output.state.name,
                "vx_body_mps": output.command.vx_body_mps,
                "vy_body_mps": output.command.vy_body_mps,
                "vz_body_mps": output.command.vz_body_mps,
                "yaw_rate_rad_s": output.command.yaw_rate_rad_s,
                "valid": output.command.valid,
                "reject_reason": output.reject_reason,
            },
        }
        if estimate is not None:
            record["target"] = {
                "timestamp_us": estimate.timestamp_us,
                "measurement_age_us": estimate.measurement_age_us,
                "yaw_rad": estimate.yaw_rad,
                "pitch_rad": estimate.pitch_rad,
                "confidence": estimate.confidence,
                "valid": estimate.valid,
                "safe_for_control": estimate.safe_for_control,
                "predicted": estimate.predicted,
            }
        self._file.write(json.dumps(record) + "\n")
        self._file.flush()

    def close(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
