"""Top-level orchestrator -- wires backends into a running aiohttp app.

This class knows nothing about specific cameras or robots; everything
hardware-specific arrives through the injected :class:`PointCloudSource`
and :class:`RobotDriver`.

Phases:
    idle         after-connect, before finger calibration
    finger_cal   walking through the FingerCalibrationFSM
    ready        calibration done, waiting for left-trigger to engage
    tracking     left-trigger held, CartesianTracker active
    fault        a safety event paused us; user must acknowledge

Loops (all asyncio tasks):
    _control_loop      handles inbound WS messages (hand state, buttons)
    _command_loop      sends RobotCommands at ~50 Hz
    _pointcloud_loop   grabs frames + broadcasts on the binary WS
    _safety_loop       runs SafetyMonitor + emits SafetyMsg events
"""

from __future__ import annotations

import asyncio
import contextlib
import ssl
from dataclasses import dataclass
from pathlib import Path

from aiohttp import WSMsgType, web

from .calibration import FingerCalibrationFSM
from .messages import (
    AnchorMsg, ButtonMsg, HandStateMsg, PhaseMsg, PromptMsg, TriggerMsg,
    WorkspaceMsg, decode, encode,
)
from .point_cloud import PointCloudSource, encode_frame
from .robot import RobotCommand, RobotDriver
from .safety import SafetyConfig, SafetyMonitor
from .tracking import CartesianTracker
from .types import Pose
from .workspace import Workspace

import time

import numpy as np


@dataclass
class ServerConfig:
    """Static configuration that doesn't change per-connection."""
    host: str = "0.0.0.0"
    port: int = 8000
    static_dir: Path = Path(__file__).parent.parent / "webxr_app" / "static"
    cert: Path | None = None
    key: Path | None = None
    command_hz: float = 50.0
    pointcloud_hz: float = 15.0
    safety_hz: float = 30.0


class TeleopServer:
    """Owns one source / one driver / one connection (single-operator design)."""

    def __init__(
        self,
        point_cloud_source: PointCloudSource,
        robot_driver: RobotDriver,
        workspace: Workspace,
        config: ServerConfig,
        safety_config: SafetyConfig | None = None,
    ) -> None:
        self._pc = point_cloud_source
        self._robot = robot_driver
        self._workspace = workspace
        self._config = config
        self._safety = SafetyMonitor(safety_config or SafetyConfig())
        self._tracker = CartesianTracker(workspace)
        self._calib = FingerCalibrationFSM()
        self._phase = "idle"
        self._latest_hand = HandStateMsg()
        self._shutdown = asyncio.Event()
        self._last_debug_print = 0.0

    async def run(self) -> None:
        """Start backends, start aiohttp, block until shutdown."""
        await self._pc.start()
        await self._robot.start()

        static_dir = Path(self._config.static_dir)
        app = web.Application()
        app.router.add_get("/ws", self._handle_ws)
        # Serve index.html at "/" explicitly, then fall through to static
        # files for everything else under the static dir.
        app.router.add_get(
            "/",
            lambda _req: web.FileResponse(static_dir / "index.html"),
        )
        app.router.add_static("/", path=str(static_dir), show_index=False)

        runner = web.AppRunner(app)
        await runner.setup()

        ssl_context = None
        if self._config.cert and self._config.key:
            ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            ssl_context.load_cert_chain(self._config.cert, self._config.key)

        site = web.TCPSite(
            runner, self._config.host, self._config.port, ssl_context=ssl_context,
        )
        try:
            await site.start()
            scheme = "https" if ssl_context else "http"
            print(f"[teleop] serving on {scheme}://{self._config.host}:{self._config.port}")
            await self._shutdown.wait()
        finally:
            await runner.cleanup()
            await self._pc.stop()
            await self._robot.stop()

    # ----- inbound channels -----

    async def _handle_ws(self, request) -> web.WebSocketResponse:
        """Main WebSocket handler. One connection -> spawn loops."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        # Tell the client where we are in the state machine right now.
        await ws.send_str(encode(PhaseMsg(phase=self._phase)))
        # Announce the workspace box so the client can draw the wireframe.
        await ws.send_str(encode(WorkspaceMsg(
            min=tuple(float(v) for v in self._workspace.min_corner),
            max=tuple(float(v) for v in self._workspace.max_corner),
        )))
        # Initial prompt: ask the operator to begin calibration.
        await ws.send_str(encode(PromptMsg(text=self._calib.current_prompt)))

        tasks = [
            asyncio.create_task(self._control_loop(ws), name="control_loop"),
            asyncio.create_task(self._pointcloud_loop(ws), name="pointcloud_loop"),
            asyncio.create_task(self._command_loop(), name="command_loop"),
        ]
        try:
            done, pending = await asyncio.wait(
                tasks, return_when=asyncio.FIRST_COMPLETED,
            )
            for t in pending:
                t.cancel()
            for t in pending:
                with contextlib.suppress(asyncio.CancelledError):
                    await t
            # Surface any exception from the loop that finished first.
            for t in done:
                exc = t.exception()
                if exc:
                    print(f"[teleop] {t.get_name()} crashed: {exc!r}")
        finally:
            if not ws.closed:
                await ws.close()
        return ws

    async def _control_loop(self, ws: web.WebSocketResponse) -> None:
        """Receive HandStateMsg/ButtonMsg/TriggerMsg; update internal state."""
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    decoded = decode(msg.data)
                except Exception as exc:
                    print(f"[teleop] control decode error: {exc!r}; raw={msg.data!r}")
                    continue
                if isinstance(decoded, HandStateMsg):
                    self._latest_hand = decoded
                elif isinstance(decoded, ButtonMsg):
                    await self._on_button(ws, decoded)
                elif isinstance(decoded, TriggerMsg):
                    await self._on_trigger(ws, decoded)
            elif msg.type == WSMsgType.ERROR:
                print(f"[teleop] ws error: {ws.exception()!r}")
                break

    async def _on_button(self, ws: web.WebSocketResponse, btn: ButtonMsg) -> None:
        """Handle a single rising-edge button event."""
        if btn.hand != "left" or not btn.pressed:
            return
        if btn.name == "x_click":
            await self._advance_calibration(ws)
        # 'y_click' (quit) and others -- TBD.

    async def _advance_calibration(self, ws: web.WebSocketResponse) -> None:
        """Drive the finger-calibration FSM from an X click."""
        if self._phase == "idle":
            self._calib.on_start()
            self._phase = "finger_cal"
            await ws.send_str(encode(PhaseMsg(phase=self._phase)))
            await ws.send_str(encode(PromptMsg(text=self._calib.current_prompt)))
            return

        if self._phase == "finger_cal":
            if not self._latest_hand.valid:
                await ws.send_str(encode(PromptMsg(
                    text=self._calib.current_prompt
                         + "\n(no hand tracked -- bring your right hand into view)",
                    severity="warn",
                )))
                return
            self._calib.on_confirm(self._latest_hand.curls, self._latest_hand.abduction)
            if self._calib.is_complete:
                self._phase = "ready"
                rec = self._calib.record
                print(
                    f"[teleop] calibration complete: "
                    f"curl_min={rec.min_curl.tolist()} curl_max={rec.max_curl.tolist()} "
                    f"abd_min={rec.min_abd:.3f} abd_max={rec.max_abd:.3f}"
                )
                await ws.send_str(encode(PhaseMsg(phase=self._phase)))
                await ws.send_str(encode(PromptMsg(
                    text="Ready. Hold LEFT trigger to engage tracking.",
                )))
            else:
                await ws.send_str(encode(PromptMsg(text=self._calib.current_prompt)))

    # ----- outbound loops -----

    async def _on_trigger(self, ws: web.WebSocketResponse, trg: TriggerMsg) -> None:
        """Engage / disengage Cartesian tracking on the left trigger."""
        if trg.hand != "left" or trg.name != "trigger":
            return
        # Threshold edges: the client sends analog values, the server makes
        # the engage decision so the policy (e.g. hysteresis) lives in one
        # place if we ever need to harden it.
        ENGAGE_THRESHOLD = 0.6
        DISENGAGE_THRESHOLD = 0.3
        engaged = self._tracker.is_engaged
        if not engaged and trg.value >= ENGAGE_THRESHOLD:
            if self._phase != "ready" and self._phase != "tracking":
                # Don't allow engage until finger calibration is complete --
                # otherwise the curls we send to the robot are uncalibrated.
                return
            if not self._latest_hand.valid:
                return
            user_wrist = self._latest_user_wrist_pose()
            if user_wrist is None:
                return
            robot_state = await self._robot.get_state()
            self._tracker.engage(
                user_wrist=user_wrist,
                robot_wrist=robot_state.wrist_pose,
                t=time.monotonic(),
            )
            # Tell the client where the robot-world origin lives in the
            # VR play_space. Helmet axes (x=right, y=up, z=back) differ
            # from robot axes (x=right, y=forward, z=up); the tracker
            # already accounts for this on the wrist. To position the
            # robot-frame origin in helmet coords:
            #   helmet_origin = user_anchor_helmet - R^-1 @ robot_anchor_robot
            # where R^-1 takes (x,y,z)_robot -> (x, z, -y)_helmet.
            user_pos = np.asarray(user_wrist.position, dtype=np.float64)        # helmet frame
            rp = np.asarray(robot_state.wrist_pose.position, dtype=np.float64)  # robot frame
            robot_anchor_in_helmet = np.array([rp[0], rp[2], -rp[1]], dtype=np.float64)
            vr_origin = (user_pos - robot_anchor_in_helmet).tolist()
            await ws.send_str(encode(AnchorMsg(
                vr_position_of_robot_origin=(
                    float(vr_origin[0]), float(vr_origin[1]), float(vr_origin[2]),
                ),
            )))
            print(
                f"[engage] anchor user_pos={np.asarray(user_wrist.position).round(3).tolist()}  "
                f"user_ori={np.asarray(user_wrist.orientation).round(3).tolist()}  "
                f"robot_pos={np.asarray(robot_state.wrist_pose.position).round(3).tolist()}  "
                f"robot_ori={np.asarray(robot_state.wrist_pose.orientation).round(3).tolist()}",
                flush=True,
            )
            self._phase = "tracking"
            await ws.send_str(encode(PhaseMsg(phase=self._phase)))
            await ws.send_str(encode(PromptMsg(
                text="Tracking. Release trigger to freeze.",
            )))
        elif engaged and trg.value <= DISENGAGE_THRESHOLD:
            self._tracker.disengage()
            self._phase = "ready"
            await ws.send_str(encode(PhaseMsg(phase=self._phase)))
            await ws.send_str(encode(PromptMsg(
                text="Held. Pull LEFT trigger to engage tracking again.",
            )))

    def _latest_user_wrist_pose(self) -> Pose | None:
        """Build a Pose from the most recent HandStateMsg, or None if stale."""
        h = self._latest_hand
        if not h.valid:
            return None
        return Pose(
            position=np.asarray(h.wrist_position, dtype=np.float64),
            orientation=np.asarray(h.wrist_orientation, dtype=np.float64),
            frame="play_space",
        )

    async def _command_loop(self) -> None:
        """At command_hz: compute target via tracker, send to RobotDriver."""
        period = 1.0 / max(self._config.command_hz, 1e-3)
        loop = asyncio.get_running_loop()
        while not self._shutdown.is_set():
            t0 = loop.time()
            try:
                await self._tick_command()
            except Exception as exc:
                # Don't let one bad frame kill the loop -- log and keep going,
                # the operator can disengage and re-engage to recover.
                print(f"[teleop] command tick error: {exc!r}")
            elapsed = loop.time() - t0
            await asyncio.sleep(max(0.0, period - elapsed))

    async def _tick_command(self) -> None:
        """One iteration of the command loop. Extracted for readability."""
        if not self._tracker.is_engaged:
            return
        user_wrist = self._latest_user_wrist_pose()
        if user_wrist is None:
            return
        result = self._tracker.update(user_wrist, time.monotonic())
        # DEBUG: print the wrist delta relative to the engage anchor so the
        # operator can sanity-check that head rotation doesn't leak into the
        # commanded pose. Rate-limited to ~5 Hz so the terminal stays readable.
        now = time.monotonic()
        if now - self._last_debug_print >= 0.2:
            self._last_debug_print = now
            anchor = self._tracker._anchor  # internal but fine for debug
            if anchor is not None:
                dpos = np.asarray(user_wrist.position) \
                    - np.asarray(anchor.user_wrist.position)
                print(
                    f"[wrist] dpos={dpos.round(3).tolist()}  "
                    f"abs_pos={np.asarray(user_wrist.position).round(3).tolist()}  "
                    f"abs_ori={np.asarray(user_wrist.orientation).round(3).tolist()}",
                    flush=True,
                )
        # Calibrated curls live alongside the wrist in the same HandStateMsg.
        # If calibration didn't run (shouldn't happen in 'tracking', but be
        # defensive) the record is identity-ish and apply_curl returns the
        # raw values clamped to [0,1].
        raw_curls = np.asarray(self._latest_hand.curls, dtype=np.float32)
        curls = self._calib.record.apply_curl(raw_curls)
        cmd = RobotCommand(
            target_wrist_pose=result.target,
            target_finger_curls=curls,
            timestamp=time.monotonic(),
        )
        await self._robot.send(cmd)

    async def _pointcloud_loop(self, ws: web.WebSocketResponse) -> None:
        """At pointcloud_hz: grab a frame, encode, send binary."""
        period = 1.0 / max(self._config.pointcloud_hz, 1e-3)
        loop = asyncio.get_running_loop()
        while not ws.closed:
            t0 = loop.time()
            try:
                frame = await self._pc.grab()
            except Exception as exc:
                print(f"[teleop] pc grab error: {exc!r}")
                frame = None
            if frame is not None:
                try:
                    await ws.send_bytes(encode_frame(frame))
                except ConnectionResetError:
                    break
            elapsed = loop.time() - t0
            await asyncio.sleep(max(0.0, period - elapsed))

    async def _safety_loop(self, ws) -> None:
        """At safety_hz: run SafetyMonitor; broadcast events to client."""
        raise NotImplementedError

    # ----- phase transitions (small enough to keep in one place) -----

    def _enter_finger_cal(self) -> None: raise NotImplementedError
    def _finish_finger_cal(self) -> None: raise NotImplementedError
    def _engage_tracking(self) -> None: raise NotImplementedError
    def _disengage_tracking(self) -> None: raise NotImplementedError
    def _fault(self, reason: str) -> None: raise NotImplementedError
