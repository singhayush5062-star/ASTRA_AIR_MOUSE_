#!/usr/bin/env python3
"""Read-only GCS <-> drone link test (runs inside the nidar_gcs container, scripts/gcs_docker.sh).

Connects through the GCS backend's own API (exactly what the HARDWARE page's CONNECT does),
then samples the /api/ws/hardware telemetry stream for --duration seconds and reports:
heartbeat age (link lag), MAVLink packet rate, WS telemetry rate/jitter, dropouts,
whether attitude/mode actually update, radio RSSI and the ROS link to the Jetson.
Never sends TAKEOFF/LAND/RTL/ABORT or any other command to the vehicle.

  T=/home/developer/NIDAR/catkin_ws/src/nidar_gcs/scripts/gcs_link_test.py
  docker exec nidar_gcs python3 $T udp    [--port 14550] [--duration 120]
  docker exec nidar_gcs python3 $T serial --dev /dev/ttyUSB0 --baud 57600

PASS needs: heartbeat never older than 2 s, no dropouts, >= 5 MAVLink msg/s, the GCS telemetry
stream >= 8 Hz with no stall over 500 ms. hardware/PI4_DEPLOYMENT.md §8.
"""
import argparse
import asyncio
import json
import statistics
import sys
import time
import urllib.request

sys.path.insert(0, "/opt/nidar_gcs/pydeps")
import websockets  # noqa: E402

API = "http://127.0.0.1:%d"


def http(method, url, body=None, timeout=30):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]


async def sample(port, duration):
    rx_times, hb_ages, pkts, rssi = [], [], [], []
    drops, att_changes, last_att, modes, ros = 0, 0, None, set(), set()
    async with websockets.connect("ws://127.0.0.1:%d/api/ws/hardware" % port,
                                  max_size=None) as ws:
        end = time.time() + duration
        last_print = 0.0
        while time.time() < end:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=5))
            now = time.time()
            rx_times.append(now)
            p = msg.get("payload", {})
            link = p.get("link", {}) or {}
            mav = link.get("mavlink", {}) or {}
            if not msg.get("connected") or not mav.get("connected"):
                drops += 1
            if mav.get("heartbeat_age") is not None:
                hb_ages.append(mav["heartbeat_age"])
            if mav.get("packets") is not None:
                pkts.append((now, mav["packets"]))
            if mav.get("radio"):
                rssi.append(mav["radio"].get("rssi"))
            ros.add("%s/%s" % ((link.get("ros") or {}).get("status"),
                               (link.get("ros") or {}).get("connected")))
            d = p.get("drone", {}) or {}
            modes.add(d.get("mode"))
            att = json.dumps(d.get("attitude") or d.get("orientation"), sort_keys=True)
            if last_att is not None and att != last_att:
                att_changes += 1
            last_att = att
            if now - last_print > 5:
                last_print = now
                print("  t=%5.1fs hb_age=%s pkts=%s mode=%s armed=%s ros=%s" % (
                    duration - (end - now), mav.get("heartbeat_age"), mav.get("packets"),
                    d.get("mode"), d.get("armed"), (link.get("ros") or {}).get("status")),
                    flush=True)
    return rx_times, hb_ages, pkts, rssi, drops, att_changes, modes, ros


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("link", choices=["udp", "serial"])
    ap.add_argument("--dev")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--port", type=int, default=14550, help="UDP port")
    ap.add_argument("--gcs-port", type=int, default=8000)
    ap.add_argument("--duration", type=int, default=120)
    a = ap.parse_args()
    base = API % a.gcs_port

    st, h = http("GET", base + "/api/health")
    print("[1] GCS backend health: HTTP %s %s" % (st, h))
    st, ports = http("GET", base + "/api/hardware/ports")
    real = [p for p in ports.get("ports", []) if p.get("hwid") != "n/a"]  # skip onboard ttyS*
    print("[2] USB serial ports seen by GCS: %s" % (json.dumps(real, indent=1) if real else "none"))

    req = {"connection_type": a.link, "baud_rate": a.baud, "udp_port": a.port}
    if a.link == "serial":
        req["serial_port"] = a.dev
    t0 = time.time()
    st, r = http("POST", base + "/api/hardware/connect", req, timeout=60)
    print("[3] CONNECT %s -> HTTP %s in %.1fs: %s" % (req, st, time.time() - t0,
                                                     r.get("message") or r.get("error")))
    if st != 200:
        print("RESULT: FAIL (no flight-controller heartbeat)")
        return 1

    print("[4] sampling /api/ws/hardware for %ds (read-only) ..." % a.duration)
    rx, hb, pk, rssi, drops, attc, modes, ros = asyncio.get_event_loop().run_until_complete(
        sample(a.gcs_port, a.duration))

    gaps = [b - c for c, b in zip(rx, rx[1:])]
    rate = (pk[-1][1] - pk[0][1]) / (pk[-1][0] - pk[0][0]) if len(pk) > 1 else 0.0
    ws_hz = len(rx) / (rx[-1] - rx[0]) if len(rx) > 1 else 0.0
    print("\n===== RESULTS (%s) =====" % (a.dev or "udp:%d" % a.port))
    print("heartbeat age  : mean %.2fs  p95 %.2fs  max %.2fs" % (
        statistics.mean(hb) if hb else float("nan"), pct(hb, 95), max(hb) if hb else float("nan")))
    print("MAVLink packets: %.1f msg/s" % rate)
    print("GCS WS stream  : %.1f Hz, gap p95 %.0f ms, max %.0f ms" % (
        ws_hz, pct(gaps, 95) * 1000, (max(gaps) if gaps else 0) * 1000))
    print("dropout samples: %d / %d" % (drops, len(rx)))
    print("attitude updates seen: %d   modes seen: %s" % (attc, sorted(m for m in modes if m)))
    print("radio RSSI     : %s" % ("n/a" if not rssi else "min %s max %s" % (min(rssi), max(rssi))))
    print("ROS link       : %s" % sorted(ros))

    fails = []
    if not hb or max(hb) > 2.0:
        fails.append("heartbeat age exceeded 2 s (lag / link stall)")
    if drops:
        fails.append("%d samples with link disconnected" % drops)
    if rate < 5:
        fails.append("MAVLink rate below 5 msg/s")
    if ws_hz < 8 or (gaps and max(gaps) > 0.5):
        fails.append("GCS telemetry stream slower than 8 Hz or stalled >500 ms")
    print("RESULT: %s" % ("PASS" if not fails else "FAIL -> " + "; ".join(fails)))
    print("(link left connected; use the HARDWARE page or POST /api/hardware/disconnect)")
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
