# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from concert_isaac.assets import concert_base_only_simplified


ENVIRONMENTS_DIR = Path(__file__).resolve().parent.parent / "assets" / "usd" / "environments"
DEFAULT_ENV = "cantamessa"


def list_environments() -> list[str]:
    if not ENVIRONMENTS_DIR.is_dir():
        return []
    return sorted(p.name for p in ENVIRONMENTS_DIR.iterdir() if p.is_dir())


def resolve_environment(name: str) -> str:
    usd = ENVIRONMENTS_DIR / name / f"{name}.usd"
    if not usd.is_file():
        available = ", ".join(list_environments()) or "<none>"
        raise FileNotFoundError(
            f"BIM environment '{name}' not found at {usd}. Available: {available}"
        )
    return str(usd)


BIM_USD_PATH = resolve_environment(DEFAULT_ENV)


@configclass
class BimSceneCfg(InteractiveSceneCfg):

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    bim = AssetBaseCfg(
        prim_path="/World/Bim",
        spawn=sim_utils.UsdFileCfg(
            usd_path=BIM_USD_PATH,
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
    )

    robot: ArticulationCfg = concert_base_only_simplified.CONCERT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.806)),
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=750.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )


@configclass
class ConcertBimCfg:

    scene: BimSceneCfg = BimSceneCfg()
    urdf: str = concert_base_only_simplified.CONCERT_URDF
