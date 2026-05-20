# Roadmap

Ordered punchlist of remaining work. Each item lists the files
you'll touch and a rough effort. Pick one numbered item, read the
relevant interface files, implement it.

## 1. Safety monitor ⏱ 1h

Workspace-exit feedback is already wired client-side (warning panel
+ red wireframe). What's still missing is the *server-side* safety
loop and the state-transition hooks it relies on.

- `teleop_core/safety.SafetyMonitor.step` — lag detection
  (commanded vs actual wrist) and stale-state detection.
- `TeleopServer._safety_loop` — call `step`, broadcast `SafetyMsg`,
  pause the tracker when severity escalates.
- Remaining `TeleopServer` state-transition hooks
  (`_enter_finger_cal`, `_finish_finger_cal`, `_engage_tracking`,
  `_disengage_tracking`, `_fault`) — currently stubs raising
  `NotImplementedError`.

Done when: yanking the robot driver offline pops a `fault` overlay
that the user has to acknowledge.

## 2. Multi-camera RealSense source ⏱ 3h

Swap the mock for real cameras.

- `teleop_backends/pointcloud/realsense_multi.MultiRealSenseSource.*`
  — `from_config_file`, `start`, `stop`, `grab`.
- Extrinsics config file: JSON of `{serial: 4x4_matrix}`. For v1,
  hand-tune by aligning known features in the rendered cloud.
- Workspace crop *inside the source* — drop points outside the
  configured box before encoding.
- Add `pyrealsense2` to `requirements.txt` (currently commented).

Done when: with N RealSenses on the workspace, the fused cloud in VR
looks like the actual workspace.

## 3. Pybullet point-cloud source ⏱ 2h

Useful for developing the pipeline without real cameras pointed at
something interesting.

- `teleop_backends/pointcloud/pybullet_render.PybulletPointCloudSource`
  — render a depth image from one or more virtual viewpoints inside
  the same pybullet sim used by the robot driver, convert to a cloud
  in world frame.

## 4. Phase 2 frame alignment ⏱ 1h

Replace the "fixed offset in local-floor" cheat with a one-time
recenter step:

- New `RecenterMsg` in the wire protocol.
- Operator stands at a known position relative to the robot, presses
  a button to capture the play_space → world transform.
- Server persists the transform and applies it to the point cloud and
  workspace renderings.

## 5. Real arm driver — *blocked on hardware*

`AeroArmDriver` is a stub today because we don't have the arm yet.
When the hardware ships:

- Connect arm SDK in `start()`.
- Solve / send wrist target in `send()`.
- Read back actual pose in `get_state()`.
- The Aero hand fingers piggy-back as in the old project.

`TeleopServer` does not change — same interface in, different
hardware out.

---

# Contributing — agent-facing guidance

If you are an AI agent picking this up, the rules of engagement:

1. **Pick one numbered item.** Don't try to do several at once. The
   interfaces let you commit progress without breaking the rest of
   the project.
2. **Implement against the interface, never against a concrete
   class.** If you're tempted to `import pybullet` from inside
   `teleop_core`, stop — add a method to the relevant ABC instead.
3. **Honor the wire format.** If you add a new control message,
   define the dataclass in `teleop_core/messages.py` *and* extend the
   frontend's `comms.js` switch. Do both in the same change.
4. **Don't bypass the dependency direction.** No imports from
   `teleop_core` into `teleop_backends`. No imports from anywhere
   else into `webxr_app`.
5. **No mutable globals in `teleop_core`.** All state lives on
   instance attributes of `TeleopServer`.
6. **Async-first.** All I/O is awaitable. Anything that has to block
   (`rs.pipeline.wait_for_frames`, `pybullet.stepSimulation`) goes
   inside `asyncio.to_thread(...)`.
7. **Match existing port code.** When you port logic from
   `../vr_tendon_arm_teleop`, keep the numerics identical so the two
   projects produce the same calibrations.
8. **Don't add a new top-level dependency without updating
   `requirements.txt`** and noting which backend needs it. Core has
   only `aiohttp` + `numpy`.
9. **Comment why, not what.** The `what` is read in the code; the
   `why` is what an agent six months from now needs.
