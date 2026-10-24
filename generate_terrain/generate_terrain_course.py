#!/usr/bin/env python3
"""Generate a MuJoCo test course with mjlab-style terrains for mc_mujoco.

The course is a static object (no free joint) made of tiles laid out along +x,
with one lane per difficulty level along +y:

    entry ramp | flat | rough | slope up | slope down | stairs up | stairs down
               | blocks | waves | slippery | flat (end)

Every tile's border sits at the same height (BASE), so the robot can walk from
one tile to the next without a step. Pits (slope down, stairs down) are dug
below BASE, which is why the whole course is raised above the ground plane and
reached through a gentle ramp.

Heightfields use inline `elevation` data, so the XML has no external files and
needs no extra install step.

Usage:
    python3 generate_terrain_course.py                 # writes terrain_course.xml
    python3 generate_terrain_course.py --levels easy   # single lane
    python3 generate_terrain_course.py --seed 3 -o my_course.xml
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Difficulty levels (tuned for a G1-sized humanoid; adjust for your robot)
# ---------------------------------------------------------------------------


@dataclass
class Level:
    name: str
    rough_amp: float  # m, max height of uniform noise
    slope: float  # rad
    stair_h: float  # m, step height
    block_h: float  # m, max random block height
    wave_amp: float  # m, wave amplitude
    friction: float  # sliding friction of the slippery tile


LEVELS = {
    "easy": Level("easy", 0.03, 0.10, 0.05, 0.03, 0.03, 0.5),
    "medium": Level("medium", 0.05, 0.20, 0.09, 0.05, 0.06, 0.35),
    "hard": Level("hard", 0.08, 0.30, 0.13, 0.08, 0.10, 0.2),
}

TILE = 4.0  # tile side length (m)
PLATFORM = 1.2  # flat platform at the centre of pyramids/stairs (m)
STEP_W = 0.35  # stair tread depth (m)
BLOCK_W = 0.45  # random block cell size (m)
HF_RES = 0.1  # heightfield resolution (m per cell)
RAMP_LEN = 6.0  # entry ramp length (m)
LANE_GAP = 0.0  # gap between lanes (m); 0 = lanes touch
SINK = 0.01  # overlap so stacked faces never coincide (avoids z-fighting)

TILE_ORDER = [
    "flat",
    "rough",
    "slope_up",
    "slope_down",
    "stairs_up",
    "stairs_down",
    "blocks",
    "waves",
    "slippery",
    "flat",
]

COLORS = {
    "flat": "0.55 0.55 0.58 1",
    "rough": "0.45 0.40 0.33 1",
    "slope": "0.35 0.50 0.40 1",
    "stairs": "0.62 0.52 0.40 1",
    "blocks": "0.50 0.45 0.55 1",
    "waves": "0.35 0.45 0.60 1",
    "slippery": "0.70 0.85 0.95 1",
    "support": "0.40 0.40 0.42 1",
}


def max_depth(level: Level) -> float:
    """Deepest point below BASE across all pit tiles of a level."""
    n_steps = int((TILE - PLATFORM) / 2 / STEP_W)
    stairs = n_steps * level.stair_h
    slope = np.tan(level.slope) * (TILE - PLATFORM) / 2
    return max(stairs, slope)


class Course:
    def __init__(self, base: float, rng: np.random.Generator):
        self.base = base
        self.rng = rng
        self.geoms: list[str] = []
        self.hfields: list[str] = []
        self.n = 0

    # -- helpers -----------------------------------------------------------
    def _name(self, prefix: str) -> str:
        self.n += 1
        return f"{prefix}_{self.n}"

    def box(self, cx, cy, z0, z1, sx, sy, rgba, friction=None, name="box"):
        """Axis-aligned box spanning z0..z1 with half sizes sx, sy."""
        if z1 - z0 < 1e-4 or sx <= 0 or sy <= 0:
            return
        fr = f' friction="{friction} 0.005 0.0001"' if friction is not None else ""
        self.geoms.append(
            f'<geom name="{self._name(name)}" type="box" '
            f'pos="{cx:.4f} {cy:.4f} {(z0 + z1) / 2:.4f}" '
            f'size="{sx:.4f} {sy:.4f} {(z1 - z0) / 2:.4f}" rgba="{rgba}"{fr}/>'
        )

    def support(self, cx, cy, top):
        """Solid block under a tile, from the ground up to `top`."""
        self.box(cx, cy, 0.0, top, TILE / 2, TILE / 2, COLORS["support"], name="support")

    def hfield(self, cx, cy, z_bottom, heights: np.ndarray, rgba, name):
        """Heightfield tile. `heights` (m) are relative to z_bottom, >= 0."""
        h_range = float(heights.max())
        if h_range < 1e-4:
            self.box(cx, cy, z_bottom - 0.05, z_bottom, TILE / 2, TILE / 2, rgba)
            return
        nrow, ncol = heights.shape
        norm = heights / h_range
        hf_name = self._name(f"hf_{name}")
        elev = " ".join(f"{v:.4f}" for v in norm.flatten())
        base_thick = 0.05
        self.hfields.append(
            f'<hfield name="{hf_name}" nrow="{nrow}" ncol="{ncol}" '
            f'size="{TILE / 2:.4f} {TILE / 2:.4f} {h_range:.4f} {base_thick}" '
            f'elevation="{elev}"/>'
        )
        self.geoms.append(
            f'<geom name="{self._name(name)}" type="hfield" hfield="{hf_name}" '
            f'pos="{cx:.4f} {cy:.4f} {z_bottom:.4f}" rgba="{rgba}"/>'
        )

    @staticmethod
    def grid():
        n = int(round(TILE / HF_RES)) + 1
        lin = np.linspace(-TILE / 2, TILE / 2, n)
        x, y = np.meshgrid(lin, lin)  # rows = y, cols = x (MuJoCo convention)
        return x, y

    # -- tiles -------------------------------------------------------------
    def flat(self, cx, cy, lv):
        self.box(cx, cy, 0.0, self.base, TILE / 2, TILE / 2, COLORS["flat"], name="flat")

    def slippery(self, cx, cy, lv):
        self.box(
            cx, cy, 0.0, self.base, TILE / 2, TILE / 2, COLORS["slippery"],
            friction=lv.friction, name="slippery",
        )

    def rough(self, cx, cy, lv):
        # Uniform noise on a coarse grid (like mjlab/Isaac random_uniform),
        # upsampled to the heightfield resolution.
        coarse = 0.1
        nc = int(round(TILE / coarse)) + 1
        steps = np.arange(0.0, lv.rough_amp + 1e-9, 0.01)
        noise = self.rng.choice(steps, size=(nc, nc))
        x, y = self.grid()
        ix = np.clip(np.round((x + TILE / 2) / coarse).astype(int), 0, nc - 1)
        iy = np.clip(np.round((y + TILE / 2) / coarse).astype(int), 0, nc - 1)
        h = noise[iy, ix]
        # keep the border flush with neighbours
        border = (np.abs(x) > TILE / 2 - 0.1) | (np.abs(y) > TILE / 2 - 0.1)
        h[border] = 0.0
        self.support(cx, cy, self.base - SINK)
        self.hfield(cx, cy, self.base, h, COLORS["rough"], "rough")

    def _pyramid(self, x, y, lv):
        d = np.maximum(np.abs(x), np.abs(y))  # distance from centre (L-inf)
        run = (TILE - PLATFORM) / 2
        t = np.clip((TILE / 2 - d) / run, 0.0, 1.0)  # 0 at border, 1 on platform
        return t * np.tan(lv.slope) * run

    def slope_up(self, cx, cy, lv):
        x, y = self.grid()
        h = self._pyramid(x, y, lv)
        self.support(cx, cy, self.base - SINK)
        self.hfield(cx, cy, self.base, h, COLORS["slope"], "slope_up")

    def slope_down(self, cx, cy, lv):
        x, y = self.grid()
        peak = self._pyramid(x, y, lv)
        depth = float(peak.max())
        h = depth - peak  # border at BASE, centre at BASE - depth
        bottom = self.base - depth
        self.support(cx, cy, bottom - SINK)
        self.hfield(cx, cy, bottom, h, COLORS["slope"], "slope_down")

    def stairs_up(self, cx, cy, lv):
        self.support(cx, cy, self.base)
        n = int((TILE - PLATFORM) / 2 / STEP_W)
        for k in range(1, n + 1):
            half = TILE / 2 - k * STEP_W
            if k == n:
                half = max(half, PLATFORM / 2)
            z1 = self.base + k * lv.stair_h
            self.box(cx, cy, 0.0, z1, half, half, COLORS["stairs"], name="stair_up")

    def stairs_down(self, cx, cy, lv):
        n = int((TILE - PLATFORM) / 2 / STEP_W)
        bottom = self.base - n * lv.stair_h
        self.support(cx, cy, bottom)
        # ring k (0 = outermost) has its top at BASE - k * stair_h
        for k in range(n):
            outer = TILE / 2 - k * STEP_W
            inner = outer - STEP_W
            top = self.base - k * lv.stair_h
            self._ring(cx, cy, 0.0, top, outer, inner)

    def _ring(self, cx, cy, z0, z1, outer, inner):
        w = (outer - inner) / 2
        c = COLORS["stairs"]
        # +y / -y bands span the full width, +x / -x bands fill the rest
        self.box(cx, cy + inner + w, z0, z1, outer, w, c, name="stair_dn")
        self.box(cx, cy - inner - w, z0, z1, outer, w, c, name="stair_dn")
        self.box(cx + inner + w, cy, z0, z1, w, inner, c, name="stair_dn")
        self.box(cx - inner - w, cy, z0, z1, w, inner, c, name="stair_dn")

    def blocks(self, cx, cy, lv):
        self.support(cx, cy, self.base)
        n = int(round(TILE / BLOCK_W))
        w = TILE / n
        for i in range(n):
            for j in range(n):
                bx = -TILE / 2 + (i + 0.5) * w
                by = -TILE / 2 + (j + 0.5) * w
                # flat landing strip on the entry/exit edges and centre
                if abs(bx) > TILE / 2 - w or max(abs(bx), abs(by)) < PLATFORM / 2:
                    continue
                h = self.rng.uniform(0.0, lv.block_h)
                self.box(cx + bx, cy + by, self.base - SINK, self.base + h,
                         w / 2, w / 2, COLORS["blocks"], name="block")

    def waves(self, cx, cy, lv):
        x, y = self.grid()
        n_waves = 3
        h = lv.wave_amp * 0.5 * (1 - np.cos(2 * np.pi * n_waves * (x + TILE / 2) / TILE))
        self.support(cx, cy, self.base - SINK)
        self.hfield(cx, cy, self.base, h, COLORS["waves"], "waves")

    # -- ramp --------------------------------------------------------------
    def ramp(self, x_end, y_center, width):
        """Inclined box from z=0 at x_end-RAMP_LEN up to BASE at x_end."""
        angle = np.arctan2(self.base, RAMP_LEN)
        length = np.hypot(self.base, RAMP_LEN)
        thick = 0.1
        # centre of the top face midpoint, then move down by half thickness
        mx = x_end - RAMP_LEN / 2
        mz = self.base / 2
        cx = mx + np.sin(angle) * thick / 2
        cz = mz - np.cos(angle) * thick / 2
        # rotation about y by -angle (rising towards +x); MuJoCo euler is in degrees
        self.geoms.append(
            f'<geom name="{self._name("ramp")}" type="box" '
            f'pos="{cx:.4f} {y_center:.4f} {cz:.4f}" '
            f'euler="0 {-np.degrees(angle):.4f} 0" '
            f'size="{length / 2:.4f} {width / 2:.4f} {thick / 2:.4f}" '
            f'rgba="{COLORS["flat"]}"/>'
        )
        return np.degrees(angle)


def build(levels: list[Level], seed: int, x_start: float) -> tuple[str, dict]:
    rng = np.random.default_rng(seed)
    base = round(max(max_depth(lv) for lv in levels) + 0.05, 3)
    course = Course(base, rng)

    lane_w = TILE + LANE_GAP
    tiles_x0 = x_start + RAMP_LEN  # first tile's -x border
    ramp_deg = course.ramp(tiles_x0, (len(levels) - 1) * lane_w / 2, len(levels) * lane_w)

    layout = {}
    for li, lv in enumerate(levels):
        cy = li * lane_w
        for ti, kind in enumerate(TILE_ORDER):
            cx = tiles_x0 + (ti + 0.5) * TILE
            getattr(course, kind)(cx, cy, lv)
            layout.setdefault(lv.name, []).append((kind, cx, cy))

    xml = f"""<mujoco model="terrain_course">
  <!-- Generated by generate_terrain_course.py (seed={seed}).
       Course top height (BASE) = {base} m, entry ramp = {ramp_deg:.1f} deg.
       Lanes along +y: {", ".join(f"{lv.name} (y={i * lane_w:g})" for i, lv in enumerate(levels))}.
       Tiles along +x from x={tiles_x0:g}: {", ".join(TILE_ORDER)} ({TILE:g} m each). -->
  <asset>
    {chr(10).join("    " + h if i else h for i, h in enumerate(course.hfields))}
  </asset>
  <default>
    <!-- named class (like mc_mujoco's longtable.xml) so the robot's global defaults don't leak in.
         group 1 = shown by default in mc_mujoco (group 0 is its hidden collision group) -->
    <default class="terrain">
      <geom condim="3" contype="1" conaffinity="1" group="1" friction="1 0.005 0.0001"/>
    </default>
  </default>
  <worldbody>
    <body name="base_link" childclass="terrain">
      {chr(10).join("      " + g if i else g for i, g in enumerate(course.geoms))}
    </body>
  </worldbody>
</mujoco>
"""
    info = {"base": base, "ramp_deg": ramp_deg, "tiles_x0": tiles_x0, "lane_w": lane_w, "layout": layout}
    return xml, info


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-o", "--output", default="terrain_course.xml")
    p.add_argument("--levels", nargs="+", default=["easy", "medium", "hard"], choices=list(LEVELS))
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--x-start", type=float, default=1.0,
                   help="x where the entry ramp starts (robot spawns at origin by default)")
    args = p.parse_args()

    levels = [LEVELS[n] for n in args.levels]
    xml, info = build(levels, args.seed, args.x_start)
    with open(args.output, "w") as f:
        f.write(xml)

    print(f"Wrote {args.output}")
    print(f"  course height {info['base']} m, ramp {info['ramp_deg']:.1f} deg")
    for name, tiles in info["layout"].items():
        cy = tiles[0][2]
        print(f"  lane '{name}' at y={cy:g}:")
        for kind, cx, _ in tiles:
            print(f"    x={cx:6.2f}  {kind}")


if __name__ == "__main__":
    main()
