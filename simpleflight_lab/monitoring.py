"""Optional W&B sidecar: proxy training, independent evaluation and witnesses stay separate.

Set SF_WANDB=1 to enable. The default is offline. A bounded subprocess owns all
SDK/network work; local simulation never waits on initialization or logging.
This module does not grant training authorization or alter an evaluation.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
import csv
from hashlib import sha256
import json
import math
import netrc
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs/rm27_wandb.json"
SAFETY_REASONS = {"FLIGHT_VOLUME_EXIT", "EXCESSIVE_SPEED", "EXCESSIVE_BODY_RATE", "NUMERICAL_FAILURE",
                  "MUJOCO_WARNING", "INITIAL_OVERLAP", "NONFINITE_POLICY_ACTION"}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def training_metrics(row, *, optimization=None, log_std=None):
    """Pure scalar adapter; all completed-event counts remain Isaac proxy results."""
    metrics = {f"isaac_proxy/{key}": value for key, value in row.items() if _finite(value)}
    if "frames" in row:
        metrics["env/transitions"] = int(row["frames"])
    if "samples_per_s" in row:
        metrics["performance/transitions_per_s"] = float(row["samples_per_s"])
    if optimization is not None:
        for key, value in zip(("actor_loss", "critic_loss", "actor_grad", "critic_grad", "approx_kl"), optimization, strict=True):
            if _finite(float(value)):
                metrics[f"ppo/{key}"] = float(value)
    for key, dest in (("actor_loss", "actor_loss"), ("value_loss", "critic_loss"), ("approx_kl", "approx_kl")):
        if key in row and _finite(row[key]):
            metrics[f"ppo/{dest}"] = row[key]
    if log_std is not None:
        metrics["ppo/raw_gaussian_entropy_nats"] = sum(float(s) + .5 * math.log(2 * math.pi * math.e) for s in log_std)
    if row.get("episodes", 0):
        metrics["isaac_proxy/qualified_rate"] = row.get("qualified", 0) / row["episodes"]
    return metrics


def evaluation_metrics(summary, *, source, controller):
    """Extract actual completed episodes, with every failed trial in the denominator."""
    if source not in ("mujoco_independent", "isaac_proxy") or controller not in ("ppo", "track_control", "constant_control", "deterministic_witness"):
        raise ValueError("Explicit simulator and controller identity are required")
    suite = str(summary.get("suite") or summary.get("profile") or summary.get("flight_suite") or summary.get("task") or "evaluation")
    suite = suite.replace("/", "_")
    prefix = f"{source}/{controller}/{suite}"
    episodes = int(summary["episodes"])
    hold = summary.get("profile") == "hold_regression"
    flight = hold or ("finish_count" in summary and "gate" in summary)
    passed = int(summary.get("success_count", 0) if flight else
                 summary.get("qualified", summary.get("qualified_contact_count", summary.get("success_count", 0))))
    outcome = "flight_passes" if flight else "qualified_contacts"
    rate = "flight_pass_rate" if flight else "qualified_contact_rate"
    metrics = {f"{prefix}/episodes": episodes, f"{prefix}/{outcome}": passed,
               f"{prefix}/{rate}": passed / episodes if episodes else 0.0}
    cases = list((summary.get("cases") or {}).values())
    causes = Counter()
    saturation = safety = 0
    velocities, alignments = [], []
    for case in cases:
        reasons = case.get("failure_modes") or ([case["reason"]] if case.get("reason") else [])
        if hold and reasons == ["CONTACT_TIMEOUT"]:
            reasons = [] if case.get("hold_passed") else ["NOT_STABLE_AT_HORIZON"]
        causes.update(set(reasons))
        safety += bool(SAFETY_REASONS.intersection(reasons))
        saturation += int(case.get("saturated_controls", case.get("mixer_saturated_controls", 0)))
        first = case.get("first_contact")
        if first:
            for gate in first.get("gates", []):
                for key, values in (("normal_speed_m_s", velocities), ("bottom_normal_error_deg", alignments)):
                    value = gate.get(key)
                    if value is not None and _finite(value):
                        values.append(float(value))
    metrics[f"{prefix}/safety_exits"] = int(summary.get("safety_exits", safety))
    metrics[f"{prefix}/mixer_saturated_controls"] = saturation
    metrics[f"{prefix}/physical_contacts"] = int(summary.get("physical_contacts", summary.get("physical_contact_count", sum(bool(c.get("first_contact")) for c in cases))))
    for label, values in (("contact_normal_speed_m_s", velocities), ("contact_alignment_deg", alignments)):
        if values:
            values.sort()
            middle = len(values) // 2
            metrics[f"{prefix}/{label}_median"] = values[middle] if len(values) % 2 else .5 * (values[middle - 1] + values[middle])
            metrics[f"{prefix}/{label}_max"] = max(values)
    for cause, count in causes.items():
        metrics[f"{prefix}/rejections/{cause}"] = count
    return metrics


class Monitor:
    """Bounded nonblocking transport. Disabled mode creates no files or SDK imports."""

    def __init__(self, output, name, *, metadata=None, enabled=None):
        self.process = self.thread = self.events = None
        self.dropped = 0
        self.output = Path(output)
        if enabled is None:
            enabled = os.environ.get("SF_WANDB") == "1"
        if not enabled:
            return
        try:
            config = Path(os.environ.get("SF_WANDB_CONFIG", DEFAULT_CONFIG)).resolve()
            self.output.mkdir(parents=True, exist_ok=True)
            self.events = queue.Queue(maxsize=4096)
            self.process = subprocess.Popen([sys.executable, "-m", "simpleflight_lab.monitoring", "worker", "--config", str(config),
                                             "--output", str(self.output), "--name", str(name)],
                                            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                            text=True, bufsize=1, start_new_session=True)
            self.thread = threading.Thread(target=self._send, daemon=True, name="rm27-wandb-sidecar")
            self.thread.start()
            self.log({"monitor/no_training_authorization": 1}, metadata=metadata or {})
        except Exception as exc:
            self._status("UNAVAILABLE", type(exc).__name__)
            self.process = None

    def _status(self, status, error=None):
        try:
            self.output.mkdir(parents=True, exist_ok=True)
            (self.output / "monitoring-status.json").write_text(json.dumps(dict(status=status, error_type=error, dropped_events=self.dropped)) + "\n")
        except OSError:
            pass

    def _send(self):
        try:
            while True:
                event = self.events.get()
                self.process.stdin.write(json.dumps(event, allow_nan=False) + "\n")
                self.process.stdin.flush()
                if event.get("finish"):
                    self.process.stdin.close()
                    break
        except (BrokenPipeError, OSError, ValueError):
            self.dropped += 1

    def log(self, metrics, *, step=None, metadata=None, video=None):
        if self.process is None:
            return
        event = {"metrics": {k: v for k, v in metrics.items() if _finite(v)}, "step": step}
        if metadata:
            event["metadata"] = metadata
        if video:
            event["video"] = str(Path(video).resolve())
        try:
            self.events.put_nowait(event)
        except queue.Full:
            self.dropped += 1

    def finish(self, timeout_s=5):
        if self.process is None:
            return
        try:
            self.events.put_nowait({"finish": True, "dropped_events": self.dropped})
        except queue.Full:
            self.dropped += 1
        # SDK flush is outside simulation/learning. Never leave a wedged network worker.
        self.thread.join(timeout=.1)
        try:
            self.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=.5)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=.5)
            self._status("SERVICE_TIMEOUT")
        self.process = None


def publish_evaluation(summary, output, *, source, controller):
    """Optional post-evaluation hook; authoritative local output is already complete."""
    if os.environ.get("SF_WANDB") != "1":
        return
    monitor = Monitor(Path(output).parent / "monitoring" / Path(output).name, Path(output).name,
                      metadata={"kind": "evaluation", "source": source, "controller": controller,
                                "checkpoint_sha256": summary.get("checkpoint_sha256")})
    monitor.log(evaluation_metrics(summary, source=source, controller=controller))
    monitor.finish()


def worker(args):
    """All SDK, authentication and network work stays in this expendable process."""
    destination = args.output / "monitoring-status.json"
    def status(value, **extra):
        destination.write_text(json.dumps(dict(status=value, **extra), allow_nan=False) + "\n")
    try:
        import wandb
        cfg = json.loads(args.config.read_text())
        mode = os.environ.get("SF_WANDB_MODE", cfg["mode"])
        if mode not in ("offline", "online"):
            raise ValueError("Use offline or online monitoring")
        base_url = os.environ.get("WANDB_BASE_URL") or cfg.get("base_url")
        if mode == "online" and not base_url:
            status("ONLINE_BLOCKED_NO_CONFIGURED_ENDPOINT")
            return
        if base_url:
            parsed = urlsplit(base_url)
            if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
                raise ValueError("Endpoint must be a credential-free HTTPS service base URL")
        env = dict(WANDB_SILENT="true", WANDB_CONSOLE="off", WANDB_DISABLE_CODE="true", WANDB_IGNORE_GLOBS="*",
                   WANDB_INIT_TIMEOUT="30", WANDB_HTTP_TIMEOUT="10", WANDB_DISABLE_GIT="true")
        if base_url:
            env["WANDB_BASE_URL"] = base_url
        os.environ.update(env)
        run = wandb.init(project=os.environ.get("WANDB_PROJECT", cfg["project"]),
                         entity=os.environ.get("WANDB_ENTITY", cfg["entity"]),
                         group=os.environ.get("SF_WANDB_GROUP", cfg["group"]),
                         job_type="sf-monitoring", name=args.name, dir=str(args.output), mode=mode,
                         resume="never", tags=["rtx4090", "simulation-only", "state-based", "monitoring"],
                         config={"source_workstation": "RTX4090", "scientific_result_namespaces": "simulator/controller/suite",
                                 "monitoring_does_not_authorize_training": True},
                         settings=wandb.Settings(init_timeout=30, console="off", save_code=False))
        status("ACTIVE", mode=mode, run_id=run.id, group=run.group, base_url=base_url)
        events = videos = 0
        for line in sys.stdin:
            event = json.loads(line)
            if event.get("finish"):
                run.summary["monitor/dropped_events"] = int(event["dropped_events"])
                break
            metrics = event.get("metrics", {})
            if event.get("metadata"):
                run.config.update(event["metadata"], allow_val_change=True)
            if event.get("video") and videos < cfg["max_videos_per_run"]:
                path = Path(event["video"])
                if path.is_file() and path.stat().st_size <= cfg["max_video_bytes"]:
                    metrics["media/representative_evaluation"] = wandb.Video(str(path), format="mp4")
                    videos += 1
            run.log(metrics, step=event.get("step"), commit=True)
            events += 1
        run.finish()
        status("FINISHED", mode=mode, run_id=run.id, group=run.group, events=events, videos=videos, base_url=base_url)
    except Exception as exc:
        # SDK exceptions can include endpoint/auth details: publish only the type.
        status("UNAVAILABLE", error_type=type(exc).__name__)



def probe_service(config_path):
    """Read-only project query using only an explicitly configured service endpoint."""
    cfg = json.loads(config_path.read_text())
    endpoint = os.environ.get("WANDB_BASE_URL") or cfg.get("base_url")
    key = os.environ.get("WANDB_API_KEY")
    if not endpoint:
        return {"status": "UNCONFIGURED_ENDPOINT", "environment_auth_present": bool(key),
                "project_access": "NOT_VERIFIED", "dashboard_url_is_not_api_configuration": True}
    parsed = urlsplit(endpoint)
    if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
        return {"status": "INVALID_ENDPOINT", "project_access": "NOT_VERIFIED"}
    if not key:
        try:
            auth = netrc.netrc().authenticators(parsed.hostname)
            key = auth[2] if auth else None
        except (OSError, netrc.NetrcParseError):
            pass
    if not key:
        return {"status": "NO_CONFIGURED_AUTH", "endpoint": endpoint, "project_access": "NOT_VERIFIED"}
    query = {"query": "query($name:String!,$entity:String!){project(name:$name,entityName:$entity){name}}",
             "variables": {"name": os.environ.get("WANDB_PROJECT", cfg["project"]),
                           "entity": os.environ.get("WANDB_ENTITY", cfg["entity"])}}
    authorization = base64.b64encode(("api:" + key).encode()).decode()
    request = Request(endpoint.rstrip("/") + "/graphql", data=json.dumps(query).encode(),
                      headers={"Content-Type": "application/json", "Authorization": "Basic " + authorization})
    try:
        with urlopen(request, timeout=5) as response:
            payload = json.load(response)
        exists = bool((payload.get("data") or {}).get("project")) and not payload.get("errors")
        return {"status": "PROJECT_ACCESS_CONFIRMED" if exists else "PROJECT_QUERY_REJECTED", "endpoint": endpoint,
                "project_access": "PASS" if exists else "NOT_VERIFIED"}
    except HTTPError as exc:
        return {"status": "HTTP_ERROR", "http_status": exc.code, "project_access": "NOT_VERIFIED"}
    except Exception as exc:
        return {"status": "SERVICE_UNAVAILABLE", "error_type": type(exc).__name__, "project_access": "NOT_VERIFIED"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    work = sub.add_parser("worker")
    work.add_argument("--config", required=True, type=Path)
    work.add_argument("--output", required=True, type=Path)
    work.add_argument("--name", required=True)
    probe = sub.add_parser("probe", help="Read-only configured endpoint/auth/project check; never uses the dashboard URL")
    probe.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    report = sub.add_parser("report", help="Log saved evidence in a new 4090 run; does not simulate or train")
    report.add_argument("--output", required=True, type=Path)
    report.add_argument("--name", required=True)
    report.add_argument("--training-csv", type=Path)
    report.add_argument("--evaluation-json", nargs="*", type=Path, default=[])
    report.add_argument("--controller", choices=("ppo", "track_control", "constant_control", "deterministic_witness"), default="ppo")
    report.add_argument("--source", choices=("mujoco_independent", "isaac_proxy"), default="mujoco_independent")
    report.add_argument("--video", type=Path)
    args = parser.parse_args()
    if args.command == "probe":
        print(json.dumps(probe_service(args.config)))
        return
    if args.command == "worker":
        worker(args)
        return
    args.output.mkdir(parents=True, exist_ok=False)
    monitor = Monitor(args.output, args.name, enabled=True, metadata={"kind": "saved-evidence-import", "no_new_learning": True})
    if args.training_csv:
        with args.training_csv.open() as stream:
            for row in csv.DictReader(stream):
                values = {k: float(v) for k, v in row.items() if v and v not in ("nan", "inf", "-inf")}
                monitor.log(training_metrics(values), step=int(values.get("frames", 0)))
    for path in args.evaluation_json:
        summary = json.loads(path.read_text())
        monitor.log(evaluation_metrics(summary, source=args.source, controller=args.controller),
                    metadata={"evaluation_source_sha256": sha256(path.read_bytes()).hexdigest()})
    if args.video:
        monitor.log({}, video=args.video)
    # Explicit archive publication may wait for network transfer; learning and
    # evaluation hooks retain their five-second shutdown bound.
    monitor.finish(timeout_s=120)
    print(json.dumps(json.loads((args.output / "monitoring-status.json").read_text())))


if __name__ == "__main__":
    main()
