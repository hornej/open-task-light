#!/usr/bin/env python3
"""Watch Open Task Light PWM on a Siglent SDS1000X-E over LAN.

This script uses scopeloop's instrument + device helpers, but does not require
the scopeloop MCP server to be running.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import re
import sys
import time
from pathlib import Path

from scopeloop.config import load_config
from scopeloop.devices import DeviceManager, DeviceMatch
from scopeloop.instruments.serial_monitor import SerialMonitor
from scopeloop.instruments.siglent_scope import SiglentSDS1000X


def _parse_vdiv(scale: str | None) -> float | None:
    """Parse scopeloop.yaml scale strings like '1V/div' or '200mV/div'."""
    if not scale:
        return None
    s = scale.strip().lower().replace(" ", "")
    m = re.match(r"^([0-9]*\.?[0-9]+)(mv|v)/div$", s)
    if not m:
        return None
    value = float(m.group(1))
    unit = m.group(2)
    return value * (1e-3 if unit == "mv" else 1.0)


def _should_sample_on_line(line: str) -> bool:
    # ESP-IDF log lines look like: "I (12345) touch: Brightness increased ..."
    keywords = (
        "Brightness",
        "Temperature",
        "Cool temp",
        "Warm temp",
        "NeoPixel",
    )
    return any(k in line for k in keywords)


async def _measure(scope: SiglentSDS1000X, channels: list[str]) -> dict:
    out: dict[str, dict] = {}
    for ch in channels:
        freq = await scope.measure_frequency(ch)
        duty = await scope.measure_duty_cycle(ch)
        vpp = await scope.measure_amplitude(ch)
        out[ch] = {
            "frequency_hz": freq.value,
            "duty_percent": duty.value,
            "vpp_v": vpp.value,
        }
    return out


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        type=Path,
        default=Path("scopeloop.yaml"),
        help="Path to scopeloop.yaml (default: ./scopeloop.yaml).",
    )
    ap.add_argument(
        "--timebase",
        type=float,
        default=1e-5,
        help="Seconds/div to set on the scope (default: 1e-5 = 10us/div).",
    )
    ap.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Polling interval seconds when not using --serial (default: 0.5).",
    )
    ap.add_argument(
        "--serial",
        action="store_true",
        help="Also monitor DUT serial, and sample on brightness/temp change logs.",
    )
    ap.add_argument(
        "--screenshot-dir",
        type=Path,
        default=None,
        help="If set, save a PNG screenshot after each sample to this directory.",
    )
    args = ap.parse_args()

    cfg = load_config(args.config)

    if not cfg.instruments.oscilloscope or not cfg.instruments.oscilloscope.address:
        print("No oscilloscope.address configured in scopeloop.yaml", file=sys.stderr)
        return 2

    scope = SiglentSDS1000X(cfg.instruments.oscilloscope.address)

    channels = sorted(cfg.instruments.oscilloscope.probes.keys()) or ["CH1", "CH2"]

    if args.screenshot_dir:
        args.screenshot_dir.mkdir(parents=True, exist_ok=True)

    async with scope:
        # Basic setup (keep minimal; you can still tweak settings on the front panel).
        await scope.set_timebase(args.timebase)

        for ch_name, probe in cfg.instruments.oscilloscope.probes.items():
            await scope.set_channel_enabled(ch_name, True)
            if probe.coupling:
                await scope.set_channel_coupling(ch_name, probe.coupling)
            vdiv = _parse_vdiv(probe.scale)
            if vdiv is not None:
                await scope.set_channel_scale(ch_name, vdiv)

        async def sample(reason: str, serial_line: str | None = None) -> None:
            m = await _measure(scope, channels)
            payload = {
                "ts": time.time(),
                "reason": reason,
                "serial_line": serial_line,
                "measurements": m,
            }
            print(json.dumps(payload, sort_keys=True))

            if args.screenshot_dir:
                png = await scope.screenshot()
                fname = f"{int(payload['ts'] * 1000)}_{reason}.png"
                (args.screenshot_dir / fname).write_bytes(png)

        if not args.serial:
            while True:
                await sample("poll")
                await asyncio.sleep(args.interval)

        # Serial-triggered sampling mode
        if not cfg.serial:
            print("No serial section configured in scopeloop.yaml", file=sys.stderr)
            return 2

        device_cfg = cfg.resolve_device_alias(cfg.serial.device)
        match = DeviceMatch(
            vid=device_cfg.match.vid,
            pid=device_cfg.match.pid,
            serial=device_cfg.match.serial,
        )
        dev = await DeviceManager().find_device(match)
        if not dev:
            print(f"Could not find device for {match}", file=sys.stderr)
            return 2

        monitor = SerialMonitor(port=dev.port, baud=cfg.serial.baud)
        await monitor.start()
        try:
            async for line in monitor.lines():
                print(line.line)
                if _should_sample_on_line(line.line):
                    # Give fade logic a moment to settle so duty measurements are stable.
                    await asyncio.sleep(0.15)
                    await sample("serial", serial_line=line.line)
        finally:
            await monitor.stop()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        raise SystemExit(130)

