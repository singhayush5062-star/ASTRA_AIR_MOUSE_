"""Serial port discovery for the Hardware page's CONNECT DRONE dialog (GET /api/hardware/ports).

pyserial's comports() is the base. On Linux it is extended with what it skips -- Jetson on-board
UARTs (/dev/ttyTHS*), Raspberry-Pi style /dev/ttyAMA* -- and every port is annotated with the
things that actually stop a connection on a fresh laptop:

  * what is plugged in, from the USB VID:PID (Pixhawk, CubePilot, SiK/FTDI/CP210x/CH34x radios),
    with "MAVLink" in the description of flight controllers so the dialog pre-selects them;
  * no read/write permission (user not in the `dialout` group);
  * the port already open in another process (QGroundControl, MAVROS, a second GCS, ...).

The response keeps the dialog's {port, description, hwid} shape; the extra keys are informational.
"""

from __future__ import annotations

import glob
import os
import sys
from typing import Dict, List, Optional, Tuple

# (vid, pid or None) -> (label, is_flight_controller)
USB_IDS = {
    (0x26AC, None): ("PX4 / Pixhawk flight controller", True),
    (0x3162, None): ("Holybro Pixhawk flight controller", True),
    (0x2DAE, None): ("CubePilot flight controller", True),
    (0x1209, 0x5740): ("ArduPilot/ChibiOS flight controller", True),
    (0x1209, 0x5741): ("ArduPilot/ChibiOS flight controller", True),
    (0x0483, 0x5740): ("STM32 virtual COM port (flight controller?)", True),
    (0x0403, None): ("FTDI USB-UART (SiK radio / telemetry adapter)", False),
    (0x10C4, 0xEA60): ("CP210x USB-UART (SiK radio / RC data link / telemetry)", False),
    (0x1A86, None): ("CH34x USB-UART (telemetry adapter)", False),
    (0x067B, None): ("Prolific PL2303 USB-UART", False),
}


def _usb_label(vid: Optional[int], pid: Optional[int]) -> Tuple[Optional[str], bool]:
    if vid is None:
        return None, False
    hit = USB_IDS.get((vid, pid)) or USB_IDS.get((vid, None))
    return hit if hit else (None, False)


def _open_by(device: str) -> Optional[str]:
    """'name (pid N)' of a process holding `device` open, if one is visible to this user."""
    if not sys.platform.startswith("linux"):
        return None
    try:
        target = os.path.realpath(device)
        me = os.getpid()
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == me:
                continue
            fd_dir = "/proc/%s/fd" % pid
            try:
                for fd in os.listdir(fd_dir):
                    try:
                        if os.readlink(os.path.join(fd_dir, fd)) == target:
                            with open("/proc/%s/comm" % pid) as f:
                                return "%s (pid %s)" % (f.read().strip(), pid)
                    except OSError:
                        continue
            except OSError:
                continue  # not our process, or gone
    except OSError:
        pass
    return None


def _by_id_names() -> Dict[str, str]:
    """realpath -> stable /dev/serial/by-id name (survives replugging in a different order)."""
    out: Dict[str, str] = {}
    for link in glob.glob("/dev/serial/by-id/*"):
        out[os.path.realpath(link)] = link
    return out


def _annotate(entry: Dict, device: str, by_id: Dict[str, str]) -> Dict:
    notes = []
    accessible, holder = True, None
    if sys.platform.startswith("linux"):
        accessible = os.access(device, os.R_OK | os.W_OK)
        if not accessible:
            notes.append("NO PERMISSION: sudo usermod -aG dialout $USER, then log out/in")
        holder = _open_by(device)
        if holder:
            notes.append("IN USE by %s" % holder)
        entry["by_id"] = by_id.get(os.path.realpath(device), "")
    entry["accessible"] = accessible
    entry["in_use_by"] = holder or ""
    if notes:
        entry["description"] = "%s  [%s]" % (entry["description"], "; ".join(notes))
    return entry


def list_serial_ports() -> List[Dict]:
    ports: Dict[str, Dict] = {}
    by_id = _by_id_names()
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            label, is_fc = _usb_label(p.vid, p.pid)
            desc = p.description if p.description and p.description != "n/a" else p.device
            if label:
                desc = "%s — %s" % (label, desc) if label not in desc else desc
            if is_fc:
                desc += " — MAVLink"
            ports[p.device] = {"port": p.device, "description": desc, "hwid": p.hwid or "",
                               "kind": "flight_controller" if is_fc else
                               ("usb_serial" if p.vid is not None else "serial")}
    except Exception:
        pass

    if sys.platform.startswith("linux"):
        # Jetson UARTs (ttyTHS*: the 40-pin header and M.2 UARTs) and Pi/AMA UARTs are not in
        # pyserial's scan list. ttyTCU0 is the Jetson debug console: never a MAVLink port.
        for dev in sorted(glob.glob("/dev/ttyTHS*") + glob.glob("/dev/ttyAMA*")):
            if dev not in ports:
                ports[dev] = {"port": dev, "hwid": "",
                              "description": "On-board UART %s (companion/Jetson header)" % dev,
                              "kind": "onboard_uart"}

    if not ports and sys.platform == "win32":
        try:
            import winreg
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            for i in range(100):
                try:
                    _, val_data, _ = winreg.EnumValue(key, i)
                    if val_data:
                        ports[str(val_data)] = {"port": str(val_data), "hwid": "", "kind": "serial",
                                                "description": "Serial Device (%s)" % val_data}
                except OSError:
                    break
            winreg.CloseKey(key)
        except Exception:
            pass

    order = {"flight_controller": 0, "usb_serial": 1, "onboard_uart": 2, "serial": 3}
    out = [_annotate(e, e["port"], by_id) for e in ports.values()]
    out.sort(key=lambda e: (order.get(e.get("kind", "serial"), 9), e["port"]))
    return out


def describe_open_error(device: str, exc: BaseException) -> str:
    """Operator-facing reason a serial port would not open, with the fix."""
    text = str(exc)
    low = text.lower()
    if "permission denied" in low or getattr(exc, "errno", None) == 13:
        return ("Permission denied on %s. Add your user to the dialout group "
                "(sudo usermod -aG dialout $USER), then log out and back in." % device)
    if "busy" in low or getattr(exc, "errno", None) == 16:
        holder = _open_by(device)
        return ("%s is busy%s. Close QGroundControl/MAVROS/other GCS using it; on Ubuntu also "
                "stop ModemManager (sudo systemctl disable --now ModemManager)."
                % (device, " (open in %s)" % holder if holder else ""))
    if "no such file" in low or getattr(exc, "errno", None) == 2:
        return ("%s does not exist. Plug the cable/radio in and press Scan Ports; CH340 adapters "
                "on Ubuntu 22.04 are grabbed by brltty (sudo apt remove brltty)." % device)
    return "Could not open %s: %s" % (device, text)
