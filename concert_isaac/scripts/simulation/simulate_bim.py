# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Spawn the CONCERT base inside a BIM environment loaded from a USD scene."""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

ENV_DIR = Path(__file__).resolve().parents[2] / "source/concert_isaac/assets/usd/environments"

parser = argparse.ArgumentParser(description="CONCERT inside a BIM scene.")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--real-time", action="store_true", default=False)
parser.add_argument("--rtf-period", type=float, default=1.0,
                    help="Seconds between RTF/status prints (<=0 to disable).")
parser.add_argument("--env", type=str, default="cantamessa",
                    help="BIM environment name (folder under assets/usd/environments).")
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import time
import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import Articulation
from isaaclab.scene import InteractiveScene
from isaaclab.sensors.imu import Imu
from isaaclab.sim import SimulationContext

from concert_isaac.scene.bim_scene import ConcertBimCfg
import xbot2_bridge as xb


def run_simulator(sim: SimulationContext, scene: InteractiveScene, urdf_str: str):
    robot: Articulation = scene["robot"]
    imu_sensors: dict[str, Imu] = {
        n: s for n, s in scene.sensors.items() if isinstance(s, Imu)
    }

    sim_dt = sim.get_physics_dt()
    bridge = xb.IsaacXBot2Bridge(robot, imu_sensors, urdf_str)

    robot.write_joint_position_to_sim(robot.data.default_joint_pos.clone())

    rtf_period = args_cli.rtf_period
    rtf_enabled = rtf_period > 0.0
    real_time = args_cli.real_time

    time_sim = 0.0
    wall_ref = time.perf_counter()
    sim_ref = 0.0

    while simulation_app.is_running():
        loop_start = time.perf_counter()

        bridge.send_to_clients(time_sim)
        bridge.recv_from_clients()

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        time_sim += sim_dt

        if rtf_enabled:
            elapsed = loop_start - wall_ref
            if elapsed >= rtf_period:
                rtf = (time_sim - sim_ref) / elapsed
                base_pos = robot.data.root_pos_w[0].cpu().numpy()
                print(f"[sim] t={time_sim:7.2f}s  rtf={rtf:.2f}x  base={base_pos}")
                wall_ref = loop_start
                sim_ref = time_sim

        if real_time:
            sleep_time = sim_dt - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([5.0, 5.0, 3.0], [0.0, 0.0, 1.0])

    scene_cfg = ConcertBimCfg()
    scene_cfg.scene.bim.spawn.usd_path = str(ENV_DIR / args_cli.env / f"{args_cli.env}.usd")
    scene_cfg.scene.num_envs = args_cli.num_envs
    scene_cfg.scene.env_spacing = 2.0
    scene = InteractiveScene(scene_cfg.scene)

    sim.reset()
    print(f"[INFO]: Setup complete (env={args_cli.env})")
    run_simulator(sim, scene, scene_cfg.urdf)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
