"""
Procedural MuJoCo environment generator for SLAM testing.

The scene (ground plane, lighting, obstacles) is built entirely in Python via
mjSpec. The Lite3 robot is included from its authored MJCF (``Lite3.xml``).
Any floor/light that ``Lite3.xml`` authors is removed so this module owns the
full world definition.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, List, Sequence, Tuple

import mujoco
import numpy as np


@dataclass(frozen=True)
class GeneratorConfig:
    """Tunable parameters for procedural world generation.

    Attributes:
        bounds_x: Sampling range for obstacle/node X coordinates in meters.
        bounds_y: Sampling range for obstacle/node Y coordinates in meters.
        node_count: Number of navigation graph nodes to sample.
        node_min_spacing: Minimum pairwise node spacing in meters.
        node_clear_radius: Clearance radius around each node kept obstacle-free.
        edge_clear_width: Corridor width around graph edges kept obstacle-free.
        extra_edge_count: Additional non-intersecting edges after MST connectivity.
        obstacle_count: Target number of procedurally spawned obstacles.
        obstacle_sample_budget: Max rejected+accepted samples before giving up.
        floor_half_extent: Half-size of square plane floor (X/Y) in meters.
        floor_material: MuJoCo material name for the procedural floor.
        light_pos: Directional light position in world coordinates.
        light_dir: Directional light direction vector in world coordinates.
    """
    bounds_x: Tuple[float, float] = (-10.5, 2.5)
    bounds_y: Tuple[float, float] = (-5.8, 5.8)
    node_count: int = 16
    node_min_spacing: float = 2.2
    node_clear_radius: float = 1.2
    edge_clear_width: float = 0.8
    extra_edge_count: int = 6
    obstacle_count: int = 200
    obstacle_sample_budget: int = 3000
    floor_half_extent: float = 20.0
    floor_material: str = "checker_mat"
    light_pos: Tuple[float, float, float] = (-0.5, 0.5, 3.0)
    light_dir: Tuple[float, float, float] = (0.0, 0.0, -1.0)


PASTEL_RGBA: Sequence[Tuple[float, float, float, float]] = (
    (0.94, 0.76, 0.76, 1.0),
    (0.98, 0.86, 0.67, 1.0),
    (0.99, 0.95, 0.72, 1.0),
    (0.79, 0.93, 0.80, 1.0),
    (0.76, 0.87, 0.97, 1.0),
    (0.84, 0.80, 0.95, 1.0),
)


def _segment_intersects(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    """Return True if 2D line segments AB and CD intersect."""
    def orient(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
        return (
            min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
            and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
        )

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    eps = 1e-9
    if abs(o1) < eps and on_segment(a, c, b):
        return True
    if abs(o2) < eps and on_segment(a, d, b):
        return True
    if abs(o3) < eps and on_segment(c, a, d):
        return True
    if abs(o4) < eps and on_segment(c, b, d):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def _point_to_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Compute Euclidean distance from point P to segment AB."""
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = max(0.0, min(1.0, t))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _inside_free_space(
    p: np.ndarray,
    nodes: np.ndarray,
    edges: Sequence[Tuple[int, int]],
    node_clear_radius: float,
    edge_clear_width: float,
) -> bool:
    """Check whether point p lies in node/edge corridors reserved as free space."""
    if np.any(np.linalg.norm(nodes - p[None, :], axis=1) <= node_clear_radius):
        return True
    for i, j in edges:
        if _point_to_segment_distance(p, nodes[i], nodes[j]) <= (0.5 * edge_clear_width):
            return True
    return False


def _sample_nodes(rng: np.random.Generator, cfg: GeneratorConfig, start_xy: Tuple[float, float]) -> np.ndarray:
    """Sample spaced 2D nodes, seeding the first node at robot start."""
    nodes: List[np.ndarray] = [np.array(start_xy, dtype=np.float64)]
    tries = 0
    while len(nodes) < cfg.node_count and tries < 5000:
        tries += 1
        p = np.array(
            [
                rng.uniform(cfg.bounds_x[0], cfg.bounds_x[1]),
                rng.uniform(cfg.bounds_y[0], cfg.bounds_y[1]),
            ],
            dtype=np.float64,
        )
        if all(np.linalg.norm(p - q) >= cfg.node_min_spacing for q in nodes):
            nodes.append(p)
    return np.array(nodes, dtype=np.float64)


def _build_planar_edges(nodes: np.ndarray, cfg: GeneratorConfig) -> List[Tuple[int, int]]:
    """Build a non-crossing planar graph (MST + short extra edges)."""
    n = len(nodes)
    if n < 2:
        return []

    candidates: List[Tuple[float, int, int]] = []
    for i in range(n):
        for j in range(i + 1, n):
            d = float(np.linalg.norm(nodes[i] - nodes[j]))
            candidates.append((d, i, j))
    candidates.sort(key=lambda x: x[0])

    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    edges: List[Tuple[int, int]] = []
    for _, i, j in candidates:
        if find(i) == find(j):
            continue
        intersects = False
        for u, v in edges:
            if len({i, j, u, v}) < 4:
                continue
            if _segment_intersects(nodes[i], nodes[j], nodes[u], nodes[v]):
                intersects = True
                break
        if intersects:
            continue
        edges.append((i, j))
        union(i, j)
        if len(edges) >= n - 1:
            break

    extras_added = 0
    existing = set(edges)
    for _, i, j in candidates:
        key = (i, j)
        if key in existing:
            continue
        intersects = False
        for u, v in edges:
            if len({i, j, u, v}) < 4:
                continue
            if _segment_intersects(nodes[i], nodes[j], nodes[u], nodes[v]):
                intersects = True
                break
        if intersects:
            continue
        edges.append((i, j))
        existing.add(key)
        extras_added += 1
        if extras_added >= cfg.extra_edge_count:
            break
    return edges


def _all_pairs_shortest_paths(
    nodes: np.ndarray,
    edges: Sequence[Tuple[int, int]],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return distance and next-hop matrices (Floyd-Warshall)."""
    n = len(nodes)
    inf = float("inf")
    dist = np.full((n, n), inf, dtype=np.float64)
    nxt = np.full((n, n), -1, dtype=np.int32)
    for i in range(n):
        dist[i, i] = 0.0
        nxt[i, i] = i
    for i, j in edges:
        w = float(np.linalg.norm(nodes[i] - nodes[j]))
        if w < dist[i, j]:
            dist[i, j] = w
            dist[j, i] = w
            nxt[i, j] = j
            nxt[j, i] = i
    for k in range(n):
        for i in range(n):
            dik = dist[i, k]
            if not np.isfinite(dik):
                continue
            for j in range(n):
                cand = dik + dist[k, j]
                if cand < dist[i, j]:
                    dist[i, j] = cand
                    nxt[i, j] = nxt[i, k]
    return dist, nxt


def _reconstruct_shortest_path(u: int, v: int, nxt: np.ndarray) -> List[int]:
    """Reconstruct shortest path from u to v (inclusive)."""
    if nxt[u, v] < 0:
        return []
    path = [u]
    while u != v:
        u = int(nxt[u, v])
        path.append(u)
    return path


def _minimum_weight_perfect_matching(odd_nodes: Sequence[int], dist: np.ndarray) -> List[Tuple[int, int]]:
    """Solve minimum perfect matching on odd-degree nodes via bitmask DP."""
    k = len(odd_nodes)
    if k == 0:
        return []
    index_to_node = list(odd_nodes)

    @lru_cache(maxsize=None)
    def solve(mask: int) -> Tuple[float, Tuple[Tuple[int, int], ...]]:
        if mask == 0:
            return 0.0, tuple()
        i = (mask & -mask).bit_length() - 1
        best_cost = float("inf")
        best_pairs: Tuple[Tuple[int, int], ...] = tuple()
        mask_without_i = mask & ~(1 << i)
        m = mask_without_i
        while m:
            j = (m & -m).bit_length() - 1
            m &= ~(1 << j)
            pair_cost = float(dist[index_to_node[i], index_to_node[j]])
            rest_cost, rest_pairs = solve(mask_without_i & ~(1 << j))
            total = pair_cost + rest_cost
            if total < best_cost:
                best_cost = total
                best_pairs = ((index_to_node[i], index_to_node[j]),) + rest_pairs
        return best_cost, best_pairs

    _, pairs = solve((1 << k) - 1)
    return list(pairs)


def _build_edge_cover_mission(nodes: np.ndarray, edges: Sequence[Tuple[int, int]], start_idx: int = 0) -> List[int]:
    """Compute a short mission that traverses all edges at least once.

    Uses undirected Chinese Postman construction:
      1) pair odd-degree nodes with minimum added shortest-path cost
      2) duplicate those shortest paths to Eulerize the multigraph
      3) run Hierholzer to obtain an Euler trail/circuit mission.
    """
    n = len(nodes)
    if n == 0:
        return []
    if not edges:
        return [start_idx]

    degree = [0] * n
    multiedge_counts: Dict[Tuple[int, int], int] = {}
    for i, j in edges:
        degree[i] += 1
        degree[j] += 1
        key = (i, j) if i < j else (j, i)
        multiedge_counts[key] = multiedge_counts.get(key, 0) + 1

    dist, nxt = _all_pairs_shortest_paths(nodes, edges)
    odd_nodes = [idx for idx, d in enumerate(degree) if (d % 2) == 1]
    for a, b in _minimum_weight_perfect_matching(odd_nodes, dist):
        path = _reconstruct_shortest_path(a, b, nxt)
        for u, v in zip(path, path[1:]):
            key = (u, v) if u < v else (v, u)
            multiedge_counts[key] = multiedge_counts.get(key, 0) + 1

    edge_instances: List[Tuple[int, int]] = []
    for (i, j), count in multiedge_counts.items():
        edge_instances.extend([(i, j)] * count)

    adjacency: List[List[int]] = [[] for _ in range(n)]
    for edge_id, (i, j) in enumerate(edge_instances):
        adjacency[i].append(edge_id)
        adjacency[j].append(edge_id)

    used = [False] * len(edge_instances)
    next_ptr = [0] * n
    stack: List[int] = [start_idx]
    trail: List[int] = []
    while stack:
        v = stack[-1]
        while next_ptr[v] < len(adjacency[v]) and used[adjacency[v][next_ptr[v]]]:
            next_ptr[v] += 1
        if next_ptr[v] >= len(adjacency[v]):
            trail.append(v)
            stack.pop()
            continue
        edge_id = adjacency[v][next_ptr[v]]
        next_ptr[v] += 1
        if used[edge_id]:
            continue
        used[edge_id] = True
        a, b = edge_instances[edge_id]
        nxt_v = b if v == a else a
        stack.append(nxt_v)

    trail.reverse()
    return trail if trail else [start_idx]


def _add_obstacle_geom(
    worldbody: mujoco.MjsBody,
    name: str,
    geom_type: int,
    pos: Tuple[float, float, float],
    size: Tuple[float, float, float],
    rgba: Tuple[float, float, float, float],
    yaw: float,
) -> None:
    """Add one obstacle geom with shared collision/visual defaults."""
    geom = worldbody.add_geom()
    geom.name = name
    geom.type = geom_type
    geom.pos = np.array(pos, dtype=np.float64)
    geom.size = np.array(size, dtype=np.float64)
    geom.rgba = np.array(rgba, dtype=np.float32)
    geom.contype = 1
    geom.conaffinity = 1
    if geom_type in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER):
        geom.quat = np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)], dtype=np.float64)


def _add_ground_plane(worldbody: mujoco.MjsBody, cfg: GeneratorConfig) -> None:
    """Create or overwrite the ground plane using config values."""
    # MuJoCo Python specs in this environment do not expose delete/remove on
    # MjsGeom, so we reuse the Lite3-authored floor when present.
    floor = next((g for g in worldbody.geoms if g.name == "floor"), None)
    if floor is None:
        floor = worldbody.add_geom()
    half = cfg.floor_half_extent
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    floor.size = np.array([half, half, 0.1], dtype=np.float64)
    floor.contype = 1
    floor.conaffinity = 1
    if cfg.floor_material:
        floor.material = cfg.floor_material


def _add_overhead_light(worldbody: mujoco.MjsBody, cfg: GeneratorConfig) -> None:
    """Create or overwrite a fixed directional overhead light."""
    # Reuse the first authored light when present, otherwise add one.
    light = next(iter(worldbody.lights), None)
    if light is None:
        light = worldbody.add_light()
    light.mode = mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    light.pos = np.array(cfg.light_pos, dtype=np.float64)
    light.dir = np.array(cfg.light_dir, dtype=np.float64)


def _populate_obstacles(
    worldbody: mujoco.MjsBody,
    rng: np.random.Generator,
    cfg: GeneratorConfig,
    robot_start_xy: Tuple[float, float],
) -> Tuple[np.ndarray, List[Tuple[int, int]], int]:
    """Populate random obstacles while preserving free-space corridors."""
    nodes = _sample_nodes(rng, cfg, robot_start_xy)
    edges = _build_planar_edges(nodes, cfg)

    obstacle_idx = 0
    attempts = 0
    while obstacle_idx < cfg.obstacle_count and attempts < cfg.obstacle_sample_budget:
        attempts += 1
        x = float(rng.uniform(cfg.bounds_x[0], cfg.bounds_x[1]))
        y = float(rng.uniform(cfg.bounds_y[0], cfg.bounds_y[1]))
        p = np.array([x, y], dtype=np.float64)
        if _inside_free_space(p, nodes, edges, cfg.node_clear_radius, cfg.edge_clear_width):
            continue

        geom_pick = int(rng.integers(0, 3))
        rgba = PASTEL_RGBA[int(rng.integers(0, len(PASTEL_RGBA)))]
        yaw = float(rng.uniform(-math.pi, math.pi))

        if geom_pick == 0:
            sx = float(rng.uniform(0.12, 0.35))
            sy = float(rng.uniform(0.12, 0.35))
            sz = float(rng.uniform(0.22, 0.5))
            _add_obstacle_geom(
                worldbody,
                f"proc_box_{obstacle_idx}",
                mujoco.mjtGeom.mjGEOM_BOX,
                (x, y, sz),
                (sx, sy, sz),
                rgba,
                yaw,
            )
        elif geom_pick == 1:
            radius = float(rng.uniform(0.08, 0.22))
            half_h = float(rng.uniform(0.22, 0.45))
            _add_obstacle_geom(
                worldbody,
                f"proc_cyl_{obstacle_idx}",
                mujoco.mjtGeom.mjGEOM_CYLINDER,
                (x, y, half_h),
                (radius, half_h, 0.0),
                rgba,
                yaw,
            )
        else:
            radius = float(rng.uniform(0.09, 0.24))
            half_h = float(rng.uniform(0.10, 0.35))
            _add_obstacle_geom(
                worldbody,
                f"proc_capsule_{obstacle_idx}",
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                (x, y, radius + half_h),
                (radius, half_h, 0.0),
                rgba,
                yaw,
            )

        obstacle_idx += 1

    return nodes, edges, obstacle_idx


def build_procedural_spec(
    lite3_xml_path: str,
    robot_start_xy: Tuple[float, float],
    seed: int,
    config: GeneratorConfig | None = None,
) -> Tuple[mujoco.MjSpec, dict]:
    """Build a full MuJoCo spec procedurally, including only the Lite3 robot from XML.

    The returned spec contains:
      * everything from ``Lite3.xml`` EXCEPT the authored floor and lights
      * a fresh ground plane (Python-authored)
      * a directional overhead light (Python-authored)
      * procedurally placed obstacles

    Args:
        lite3_xml_path: Absolute path to ``Lite3.xml`` (robot-only MJCF).
        robot_start_xy: XY position reserved as a free-space seed node.
        seed: Deterministic seed for reproducible scenes.
        config: Optional generator overrides.

    Returns:
        (spec, meta) where meta reports seed and counts.
    """
    cfg = config or GeneratorConfig()
    rng = np.random.default_rng(seed)

    spec = mujoco.MjSpec.from_file(lite3_xml_path)
    worldbody = spec.worldbody

    _add_ground_plane(worldbody, cfg)
    _add_overhead_light(worldbody, cfg)
    nodes, edges, obstacle_count = _populate_obstacles(
        worldbody, rng, cfg, robot_start_xy
    )
    mission_node_idx = _build_edge_cover_mission(nodes, edges, start_idx=0)
    node_xy = [[float(p[0]), float(p[1])] for p in nodes]
    mission_xy = [[float(nodes[idx][0]), float(nodes[idx][1])] for idx in mission_node_idx]

    meta = {
        "seed": seed,
        "nodes": len(nodes),
        "edges": len(edges),
        "obstacles": obstacle_count,
        "node_xy": node_xy,
        "mission_node_idx": mission_node_idx,
        "mission_xy": mission_xy,
        "robot_start_pose": [float(robot_start_xy[0]), float(robot_start_xy[1]), 0.0],
    }
    return spec, meta
