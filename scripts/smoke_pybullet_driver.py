"""Smoke test for PybulletRobotDriver.

Runs from a unit script (no server, no frontend), sends a sequence of
wrist targets near the home pose, and checks that get_state() reports
end-effector poses close to what we commanded. Mirrors the "Done when"
acceptance criterion for punchlist item 4.
"""

from __future__ import annotations

import asyncio
import time
from pathlib import Path

import numpy as np

from teleop_backends.robot import PybulletRobotDriver
from teleop_core.robot import RobotCommand


URDF = Path(__file__).resolve().parents[1] / "urdf_rc5_right_hand" \
    / "Robot_with_right_hand_cor.urdf"


async def _settle(driver: PybulletRobotDriver, seconds: float) -> None:
    # Let the position-controlled sim catch up to the commanded targets.
    await asyncio.sleep(seconds)


async def main() -> None:
    driver = PybulletRobotDriver(urdf_path=URDF, sim_hz=240.0)
    await driver.start()
    try:
        home = driver.home_pose
        print(f"home wrist: pos={home.position}  ori={home.orientation}")

        # Walk a small box around the home pose and verify tracking.
        offsets = [
            np.array([0.0, 0.0, 0.0]),
            np.array([0.05, 0.0, 0.0]),
            np.array([0.05, 0.05, 0.0]),
            np.array([0.0, 0.05, 0.05]),
            np.array([-0.05, 0.0, 0.05]),
            np.array([0.0, 0.0, 0.0]),
        ]
        max_err = 0.0
        for off in offsets:
            target = home.translated(off)
            curls = np.full(5, 0.3, dtype=np.float32)
            await driver.send(RobotCommand(
                target_wrist_pose=target,
                target_finger_curls=curls,
                timestamp=time.monotonic(),
            ))
            await _settle(driver, 0.5)
            st = await driver.get_state()
            err = float(np.linalg.norm(st.wrist_pose.position - target.position))
            max_err = max(max_err, err)
            print(
                f"target={target.position}  actual={st.wrist_pose.position}  "
                f"err={err*1000:.1f} mm  curls={st.finger_curls}"
            )

        print(f"\nmax position error across waypoints: {max_err*1000:.1f} mm")
        # 6 cm is a generous threshold. The 6-DOF arm can't always hit
        # both position and orientation simultaneously, so the IK trades
        # off; anything dramatically worse means the driver is broken
        # rather than the IK being merely imperfect.
        assert max_err < 0.06, f"tracking error too large: {max_err:.3f} m"
        print("OK")
    finally:
        await driver.stop()


if __name__ == "__main__":
    asyncio.run(main())
