#!/usr/bin/env python3
"""Build a standalone dashboard preview from dyno-esp.ino.

The dashboard HTML is extracted directly from the sketch so the preview stays
in sync. A small script replaces fetch() with a simulated dyno/BMS feed.
Pack voltage in this preview is simulated; the firmware reads it from the BMS.
"""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SKETCH = ROOT / "dyno-esp.ino"
OUT = ROOT / "preview" / "index.html"

BANNER = """
<div style="position:relative;z-index:5;background:#fff3cd;color:#5c4500;border-bottom:1px solid #f0d98a;padding:9px 14px;font:600 12px/1.4 system-ui,sans-serif;text-align:center">
  PREVIEW · simulated dyno and BMS data · on the ESP32 the pack voltage comes from the BMS 0x90 frame
</div>
"""

MOCK = r"""
<script>
// ---------------------------------------------------------------------------
// Preview-only simulated API. This is NOT used by the firmware.
// ---------------------------------------------------------------------------
(function () {
  let simT = 7.0;          // start mid-pull
  let simSoc = 87.5;
  let simLogged = 0;
  let simRunning = true;
  let simStopMs = 0;
  let peak = { rpm: 0, torque: 0, power: 0 };
  let sensorPhase = 0;
  let alphaNow = 0;
  let simCalib = { active: false, start: 0, samples: 0, result: 0, state: 0 };

  function makeData() {
    const dt = 0.2;
    simT += dt;

    // 20 s cycle: 13 s pull, 3 s coast, 4 s idle.
    const cycle = simT % 20;
    let rpm;
    let torque;
    if (cycle < 13) {
      const x = cycle / 13;
      rpm = 5000 * Math.pow(x, 1.15);
      torque = 2.4 * (1 - 0.85 * x) + 0.03 * Math.sin(simT * 3.1);
      alphaNow = 25 * (1 - x);
    } else if (cycle < 16) {
      const x = (cycle - 13) / 3;
      rpm = 5000 * (1 - x * x);
      torque = 0;
      alphaNow = -15 * (1 - x);
    } else {
      rpm = 0;
      torque = 0;
      alphaNow = 0;
    }
    rpm = Math.max(0, rpm + 8 * Math.sin(simT * 9));
    torque = Math.max(0, torque);

    const omega = rpm * 2 * Math.PI / 60;
    const mechPower = Math.max(0, torque * omega);
    const current = Math.max(0,
      6.0 * torque + 1.5 + 0.0012 * rpm + 0.05 * Math.sin(simT * 2));
    const voltage = 48.6 - Math.min(2.2, current * 0.09);
    const batteryPower = voltage * current;
    const eff = mechPower > 20 ? (mechPower / batteryPower) * 100 : 0;
    const effOk = eff > 5 && eff < 130;
    const cellV = voltage / 13;

    if (simCalib.active) {
      simCalib.samples++;
      if (simT - simCalib.start > 6) {
        simCalib.active = false;
        simCalib.state = 2;
        simCalib.result = 0.012;
      }
    }

    simSoc = Math.max(20, simSoc - current * dt / 36000);
    simLogged++;
    sensorPhase = (sensorPhase + 1) % 4;

    if (rpm > peak.rpm) peak.rpm = rpm;
    if (torque > peak.torque) peak.torque = torque;
    if (mechPower > peak.power) peak.power = mechPower;

    return {
      millis: Math.round(simT * 1000),
      rpm, omega, alpha: alphaNow, torque, power: mechPower,
      pMech: mechPower, pElec: batteryPower,
      eff: effOk ? eff : 0, effOk: effOk ? 1 : 0,
      pulses: Math.round(simT * 40), revolutions: Math.round(simT * 40),
      peakRpm: peak.rpm, peakTorque: peak.torque, peakPower: peak.power,
      mass: 43, radius: 0.15, inertia: 0.48375,
      friction: simCalib.result, ppr: 1, avgWindow: 8,
      sensor: sensorPhase < 2 ? 1 : 0,
      uptime: Math.round(simT + 100), runMs: Math.round(simT * 1000),
      runActive: simRunning ? 1 : 0,
      runStopMs: simRunning ? 0 : simStopMs,
      calibActive: simCalib.active ? 1 : 0,
      calibState: simCalib.state,
      calibElapsedMs: simCalib.active
        ? Math.round((simT - simCalib.start) * 1000) : 0,
      calibSamples: simCalib.samples,
      calibTorque: simCalib.result,
      log: '/preview.csv', logged: simLogged, logOk: 1, logFs: 'LittleFS',
      bleReady: 1,
      bms: {
        connected: 1, fresh: 1, ageMs: 160, ms: Math.round(simT * 1000),
        voltage,
        current,
        soc: simSoc,
        maxCell: Math.round((cellV + 0.006) * 1000), maxCellNo: 5,
        minCell: Math.round((cellV - 0.009) * 1000), minCellNo: 7,
        maxTemp: 30 + current * 0.25, minTemp: 29 + current * 0.18,
        remCapMah: Math.round(simSoc * 280), cycles: 42,
        chargeMos: 0, dischargeMos: 1
      }
    };
  }

  window.fetch = function (url) {
    url = String(url);
    if (url.indexOf('/data') === 0) {
      return Promise.resolve({
        ok: true, status: 200,
        json: () => Promise.resolve(makeData())
      });
    }
    if (url.indexOf('/reset') === 0 || url.indexOf('/start') === 0 ||
        url.indexOf('/format') === 0) {
      simT = 0;
      simSoc = 87.5;
      simLogged = 0;
      simRunning = true;
      simStopMs = 0;
      sensorPhase = 0;
      simCalib = { active: false, start: 0, samples: 0, result: 0, state: 0 };
      peak = { rpm: 0, torque: 0, power: 0 };
      return Promise.resolve({
        ok: true, status: 200,
        json: () => Promise.resolve({ ok: true })
      });
    }
    if (url.indexOf('/calibrate') === 0) {
      simCalib.active = !simCalib.active;
      simCalib.state = simCalib.active ? 1 : 0;
      simCalib.start = simT;
      simCalib.samples = 0;
      if (!simCalib.active) simCalib.result = 0;
      return Promise.resolve({
        ok: true, status: 200,
        json: () => Promise.resolve({
          ok: true, active: simCalib.active ? 1 : 0
        })
      });
    }
    if (url.indexOf('/stop') === 0) {
      simStopMs = Math.round(simT * 1000);
      simRunning = false;
      return Promise.resolve({
        ok: true, status: 200,
        json: () => Promise.resolve({ ok: true })
      });
    }
    if (url.indexOf('/config') === 0) {
      return Promise.resolve({
        ok: true, status: 200,
        json: () => Promise.resolve({ ok: true })
      });
    }
    return Promise.resolve({
      ok: false, status: 404,
      json: () => Promise.resolve({})
    });
  };

  // The preview is a single file, so CSV links are inert.
  document.addEventListener('click', event => {
    const link = event.target.closest && event.target.closest('a');
    if (link && (link.getAttribute('href') || '').startsWith('/')) {
      event.preventDefault();
      const msg = document.getElementById('logMsg');
      if (msg) msg.textContent = 'CSV download works on the ESP32 dashboard.';
    }
  });
})();
</script>
"""


def main() -> int:
    source = SKETCH.read_text()
    match = re.search(r'R"HTML\((.*?)\)HTML"', source, re.S)
    if not match:
        print(f"Could not find INDEX_HTML in {SKETCH}", file=sys.stderr)
        return 1

    html = match.group(1)
    if "<body>" not in html:
        print("Dashboard HTML has no <body>", file=sys.stderr)
        return 1

    html = html.replace("<body>", "<body>\n" + BANNER + MOCK, 1)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(html)
    print(f"Wrote {OUT} ({len(html)} bytes)")

    if "--open" in sys.argv:
        subprocess.Popen(["xdg-open", str(OUT)])
        print(f"Opened {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
