"""RosLink — supervises the ROS bridge process (scripts/gcs_ros_bridge.py) and holds what it says.

The backend never imports rospy. The bridge is a separate python3 process running in the ROS
environment; it prints one JSON object per line. When the ROS master goes away (sim reset), the
bridge exits and this class starts a fresh one, which waits for the next master. That is what
lets the UI outlive any number of simulation runs, and attach to runs started from a terminal.
"""

from __future__ import annotations

import asyncio
import base64
import json
import os
import sys
import time
from typing import Any, Callable, Dict, List, Optional

from app.core.config import settings

BRIDGE_SCRIPT = os.path.join(settings.NIDAR_GCS_DIR, "scripts", "gcs_ros_bridge.py")


class RosLink:
    def __init__(self) -> None:
        self.bridge_status: str = "stopped"     # stopped | waiting_master | connected
        self.state: Dict[str, Any] = {}
        self.state_time: float = 0.0
        self.map: Optional[Dict[str, Any]] = None
        self.map_time: float = 0.0
        self.survivors: List[Dict[str, Any]] = []
        self.frame_jpeg: Optional[bytes] = None
        self.frame_seq: int = 0
        self.frame_time: float = 0.0
        self.frame_event: Optional[asyncio.Event] = None   # created in start(), on the server loop
        self.camera_clients: int = 0
        self.on_log: Optional[Callable[[Dict[str, Any]], None]] = None
        self.on_survivors: Optional[Callable[[List[Dict[str, Any]]], None]] = None
        self.on_status: Optional[Callable[[str], None]] = None
        self._proc: Optional[asyncio.subprocess.Process] = None
        self._task: Optional[asyncio.Task] = None
        self._acks: List[Any] = []        # (command name, future), answered in order per name

    # -- lifecycle --------------------------------------------------------------------------

    def start(self) -> None:
        if self._task is None:
            self.frame_event = asyncio.Event()
            self._task = asyncio.get_event_loop().create_task(self._supervise())

    async def stop(self) -> None:
        if self._task is not None:
            self._task.cancel()
            self._task = None
        await self._kill()

    async def _kill(self) -> None:
        p = self._proc
        self._proc = None
        if p is not None and p.returncode is None:
            p.terminate()
            try:
                await asyncio.wait_for(p.wait(), 3.0)
            except asyncio.TimeoutError:
                p.kill()

    async def restart(self) -> None:
        """Drop the current bridge (e.g. after the sim was torn down) so a new one attaches."""
        await self._kill()

    async def _supervise(self) -> None:
        while True:
            try:
                # The bridge must run with the plain ROS environment: os.environ is the
                # environment start_gcs.sh sourced (the backend's own extra packages are added to
                # sys.path in run.py, not to PYTHONPATH, so they never leak into ROS processes).
                self._proc = await asyncio.create_subprocess_exec(
                    "python3", "-u", BRIDGE_SCRIPT,
                    stdin=asyncio.subprocess.PIPE,
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.DEVNULL,
                    limit=16 * 1024 * 1024,
                    env=dict(os.environ),
                )
                await self._read(self._proc)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # keep supervising whatever happens
                print("[RosLink] bridge error: %s" % e, file=sys.stderr)
            # The bridge exits when its ROS master goes away: that run is over, so forget its
            # map and survivors. (Not on "connected": latched topics such as /survivors can
            # arrive before the bridge announces itself.)
            self._set_status("stopped")
            self.state = {}
            self.map = None
            self.survivors = []
            await asyncio.sleep(2.0)

    async def _read(self, proc: asyncio.subprocess.Process) -> None:
        assert proc.stdout is not None
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            self._handle(msg)
        await proc.wait()

    # -- message handling -------------------------------------------------------------------

    def _set_status(self, status: str) -> None:
        if status != self.bridge_status:
            self.bridge_status = status
            if self.on_status:
                self.on_status(status)

    def _handle(self, msg: Dict[str, Any]) -> None:
        kind = msg.get("type")
        if kind == "state":
            self.state = msg
            self.state_time = time.time()
        elif kind == "frame":
            self.frame_jpeg = base64.b64decode(msg["jpeg"])
            self.frame_seq += 1
            self.frame_time = time.time()
            if self.frame_event is not None:
                self.frame_event.set()
            self.frame_event = asyncio.Event()
        elif kind == "map":
            self.map = msg
            self.map_time = time.time()
        elif kind == "survivors":
            self.survivors = msg.get("survivors", [])
            if self.on_survivors:
                self.on_survivors(self.survivors)
        elif kind == "log":
            if self.on_log:
                self.on_log(msg)
        elif kind == "bridge":
            status = msg.get("status", "stopped")
            self._set_status("stopped" if status == "master_lost" else status)
            if status == "connected":
                if self.camera_clients > 0:
                    self.send({"cmd": "camera", "enable": True})
        elif kind == "ack":
            for i, (name, fut) in enumerate(self._acks):
                if name == msg.get("cmd"):
                    del self._acks[i]
                    if not fut.done():
                        fut.set_result(msg)
                    break

    # -- commands -----------------------------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self.bridge_status == "connected" and time.time() - self.state_time < 3.0

    def send(self, cmd: Dict[str, Any]) -> bool:
        p = self._proc
        if p is None or p.stdin is None or p.returncode is not None:
            return False
        try:
            p.stdin.write((json.dumps(cmd) + "\n").encode())
            return True
        except Exception:
            return False

    async def command(self, cmd: Dict[str, Any], timeout: float = 8.0) -> Dict[str, Any]:
        fut = asyncio.get_event_loop().create_future()
        entry = (cmd.get("cmd"), fut)
        self._acks.append(entry)
        if not self.send(cmd):
            self._acks.remove(entry)
            return {"ok": False, "detail": "ROS bridge not running"}
        try:
            return await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError:
            if entry in self._acks:
                self._acks.remove(entry)
            return {"ok": False, "detail": "timeout"}

    def camera_client(self, delta: int) -> None:
        self.camera_clients = max(0, self.camera_clients + delta)
        self.send({"cmd": "camera", "enable": self.camera_clients > 0})

    def age(self, key: str) -> float:
        """Seconds since the bridge last saw `key` (a topic class), or +inf."""
        if not self.connected:
            return float("inf")
        a = self.state.get("age", {}).get(key)
        if a is None:
            return float("inf")
        return float(a) + (time.time() - self.state_time)
