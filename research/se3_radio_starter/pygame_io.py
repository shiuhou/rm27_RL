"""pygame/SDL joystick tools. pygame is intentionally imported lazily."""
from __future__ import annotations

from collections import deque
import os
from pathlib import Path
import time

import numpy as np

from core import CHANNELS, RadioProfile, infer_axis


class Panel:
    def __init__(self, title: str):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError("pygame is missing. Run: python -m pip install -r requirements.txt") from exc
        self.pg = pygame
        pygame.display.init()
        pygame.font.init()
        pygame.joystick.init()
        self.screen = pygame.display.set_mode((920, 640))
        pygame.display.set_caption(title)
        self.font = pygame.font.SysFont("Consolas", 18)
        self.title_font = pygame.font.SysFont("Consolas", 23, bold=True)

    def events(self) -> list:
        return self.pg.event.get()

    def commands(self, events: list) -> list[str]:
        result = []
        for event in events:
            if event.type == self.pg.QUIT:
                result.append("QUIT")
            elif event.type == self.pg.KEYDOWN:
                if event.key == self.pg.K_ESCAPE:
                    result.append("QUIT")
                elif event.key == self.pg.K_RETURN:
                    result.append("ENTER")
                elif event.key in (self.pg.K_g, self.pg.K_h, self.pg.K_r, self.pg.K_p):
                    result.append(self.pg.key.name(event.key).upper())
        return result

    def draw(self, title: str, lines: list[str]) -> None:
        self.screen.fill((20, 23, 28))
        self.screen.blit(self.title_font.render(title, True, (235, 238, 240)), (20, 16))
        for index, line in enumerate(lines[:23]):
            # English UI avoids requiring a bundled CJK font. README is in Chinese.
            text = self.font.render(str(line)[:91], True, (216, 223, 230))
            self.screen.blit(text, (20, 57 + index * 24))
        self.pg.display.flip()

    def close(self) -> None:
        self.pg.quit()


class Radio:
    def __init__(self, panel: Panel, device_index: int):
        self.pg = panel.pg
        count = self.pg.joystick.get_count()
        print(f"Detected {count} joystick device(s).")
        for index in range(count):
            item = self.pg.joystick.Joystick(index)
            item.init()
            print(f"  [{index}] {item.get_name()} | axes={item.get_numaxes()} | GUID={item.get_guid()}")
            item.quit()
        if not 0 <= device_index < count:
            raise RuntimeError("Selected joystick not found. Pair it with the OS first; then restart this command.")
        self.joy = self.pg.joystick.Joystick(device_index)
        self.joy.init()
        self.instance_id = self.joy.get_instance_id()
        self.name = self.joy.get_name()
        self.guid = self.joy.get_guid()
        self.axis_count = self.joy.get_numaxes()
        self.button_count = self.joy.get_numbuttons()
        self.connected = True

    def read(self, events: list) -> np.ndarray | None:
        for event in events:
            if event.type == self.pg.JOYDEVICEREMOVED and event.instance_id == self.instance_id:
                self.connected = False
        if not self.connected:
            return None
        try:
            values = np.array([self.joy.get_axis(index) for index in range(self.axis_count)], dtype=float)
            if not np.isfinite(values).all():
                self.connected = False
                return None
            return values
        except self.pg.error:
            self.connected = False
            return None

    def buttons(self) -> list[int]:
        if not self.connected:
            return []
        try:
            return [index for index in range(self.button_count) if self.joy.get_button(index)]
        except self.pg.error:
            self.connected = False
            return []


def axis_lines(raw: np.ndarray) -> list[str]:
    return ["  ".join(f"a{i}={raw[i]:+.3f}" for i in range(start, min(start + 5, len(raw))))
            for start in range(0, len(raw), 5)]


def probe(device_index: int, profile_path: Path) -> None:
    panel = Panel("SE3 Radio - Axis Monitor")
    try:
        radio = Radio(panel, device_index)
        profile = RadioProfile.load(profile_path) if profile_path.exists() else None
        if profile:
            profile.validate_device(radio.guid, radio.axis_count)
        last_print = 0.0
        while True:
            events = panel.events()
            if "QUIT" in panel.commands(events):
                break
            raw = radio.read(events)
            if raw is None:
                raise RuntimeError("Joystick disconnected. Reconnect it and restart probe.")
            mapped = []
            if profile:
                normalized = profile.normalize(raw)
                mapped = ["CALIBRATED values (neutral should be exactly 0):"]
                mapped += [f"{key:>12s} : {value:+.3f}" for key, value in zip(CHANNELS, normalized)]
            lines = [radio.name, f"axes={radio.axis_count}, buttons={radio.button_count}",
                     "Move ONE axis at a time. ESC closes this monitor.", "", "RAW AXES:"]
            lines += axis_lines(raw) + [f"Pressed buttons: {radio.buttons()}", ""] + mapped
            panel.draw("Joystick monitor", lines)
            if time.monotonic() - last_print > 1.0:
                print(" | ".join(axis_lines(raw)))
                last_print = time.monotonic()
            time.sleep(1 / 60)
    finally:
        panel.close()


def calibrate(device_index: int, output_path: Path) -> None:
    panel = Panel("SE3 Radio - Calibration")
    try:
        radio = Radio(panel, device_index)
        if radio.axis_count < 4:
            raise RuntimeError("Four independent axes are required; inspect the radio channel configuration.")
        instructions = [
            ("right", "RIGHT stick fully RIGHT", "RIGHT stick fully LEFT"),
            ("forward", "RIGHT stick fully FORWARD / away", "RIGHT stick fully BACK / toward you"),
            ("up", "LEFT stick fully FORWARD / up", "LEFT stick fully BACK / down"),
            ("yaw_right", "LEFT stick fully RIGHT", "LEFT stick fully LEFT"),
        ]
        history: deque[tuple[float, np.ndarray]] = deque()
        center = None
        positive = None
        axes = {}
        stage = "center"
        channel = 0
        status = "Hold the requested position still, then press ENTER in this window."
        print("校準：先把四個方向置中，油門也必須手動放在中間。按 pygame 視窗的 ENTER 擷取。")
        print("每組依序：指定正方向到底、反方向到底、回中。每一步都按 ENTER。")
        while True:
            events = panel.events()
            actions = panel.commands(events)
            if "QUIT" in actions:
                print("Calibration cancelled; no new profile saved.")
                return
            raw = radio.read(events)
            if raw is None:
                raise RuntimeError("Device disconnected during calibration. Restart calibration.")
            now = time.monotonic()
            history.append((now, raw.copy()))
            while history and now - history[0][0] > 0.30:
                history.popleft()
            if stage == "center":
                prompt = "CENTER all sticks. Put non-spring throttle at its PHYSICAL MIDPOINT."
            elif stage == "positive":
                prompt = f"[{channel + 1}/4] Hold {instructions[channel][1]}."
            elif stage == "negative":
                prompt = f"[{channel + 1}/4] Hold {instructions[channel][2]}."
            else:
                prompt = "Return ALL axes to the original neutral positions."
            if "ENTER" in actions:
                try:
                    if len(history) < 10:
                        raise ValueError("Hold still for a moment before pressing ENTER.")
                    samples = np.array([values for _, values in history])
                    if float(np.max(np.ptp(samples, axis=0))) > 0.06:
                        raise ValueError("The stick is still moving. Hold the endpoint steady, then press ENTER.")
                    sample = np.median(samples, axis=0)
                    if stage == "center":
                        center = sample
                        stage = "positive"
                    elif stage == "positive":
                        positive = sample
                        stage = "negative"
                    elif stage == "negative":
                        key = instructions[channel][0]
                        try:
                            axes[key] = infer_axis(center, positive, sample,
                                                   {axis.index for axis in axes.values()})
                        except ValueError:
                            stage = "positive"  # Re-capture BOTH endpoints after a bad pair.
                            raise
                        stage = "recenter"
                    else:
                        if float(np.max(np.abs(sample - center))) > 0.16:
                            raise ValueError("Not back at the original neutral positions. Check throttle midpoint.")
                        channel += 1
                        if channel == 4:
                            profile = RadioProfile(radio.name, radio.guid, radio.axis_count, axes)
                            profile.save(output_path)
                            print(f"Saved: {output_path.resolve()}")
                            for key in CHANNELS:
                                print(f"  {key}: {axes[key]}")
                            return
                        stage = "positive"
                    status = "Captured. Follow the NEXT instruction; other axes stay centered."
                    history.clear()
                except ValueError as exc:
                    status = str(exc)
                    print(status)
            lines = [radio.name, "", prompt, "", "ENTER = capture     ESC = cancel", "",
                     status, "", "Raw readings:"] + axis_lines(raw)
            lines += ["", f"Assigned axes: {', '.join(f'{k}=a{v.index}' for k, v in axes.items())}",
                      "This wizard uses Mode-2 stick roles. No automatic throttle centering."]
            panel.draw("Calibration - move only the requested axis", lines)
            time.sleep(1 / 60)
    finally:
        panel.close()
