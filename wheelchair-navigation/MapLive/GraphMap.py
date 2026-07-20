"""
wheelchair_router.py
--------------------
בונה מפה משוקללת לנגישות כיסאות גלגלים מבוסס OSM.
מחזיר רשימת סמיכויות ל-C++ דרך pybind11.
ה-A* רץ בצד C++ בלבד.

דרישות:
    pip install osmnx networkx shapely
"""

import json
import osmnx as ox
import networkx as nx
import urllib3

ox.settings.requests_kwargs = {"verify": False}
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
ox.settings.timeout = 180


# ─────────────────────────────────────────────
# 1. טעינת קובץ התצורה
# ─────────────────────────────────────────────

def load_config(path: str = "wheelchair_config.json") -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ─────────────────────────────────────────────
# 2. שליפת הגרף מ-OSM
# ─────────────────────────────────────────────

CUSTOM_FILTER = (
    '["highway"]["highway"!~"motorway|motorway_link|trunk|trunk_link'
    '|construction|abandoned|steps"]'
    '["access"!~"private|no"]'
)


def fetch_graph(origin: tuple, destination: tuple, margin: float = 0.005) -> nx.MultiDiGraph:
    min_lat = min(origin[0], destination[0]) - margin
    max_lat = max(origin[0], destination[0]) + margin
    min_lon = min(origin[1], destination[1]) - margin
    max_lon = max(origin[1], destination[1]) + margin

    G = ox.graph_from_bbox(
        bbox=(min_lon, min_lat, max_lon, max_lat),
        network_type="walk",
        custom_filter=CUSTOM_FILTER,
        retain_all=False,
    )
    G = ox.add_edge_speeds(G, fallback=5.0)
    G = ox.add_edge_travel_times(G)
    return G


# ─────────────────────────────────────────────
# 3. פונקציות עזר לתגיות OSM
# ─────────────────────────────────────────────

def _get_tag(data: dict, key: str, default=None):
    val = data.get(key, default)
    if isinstance(val, list):
        val = val[0]
    return val


def _parse_width(raw) -> float | None:
    if raw is None:
        return None
    raw = str(raw).strip().lower()
    try:
        if "cm" in raw:
            return float(raw.replace("cm", "").strip()) / 100
        return float(raw.replace("m", "").strip())
    except ValueError:
        return None


def _parse_incline(raw) -> float | None:
    if raw is None:
        return None
    raw = str(raw).strip().lower().replace("%", "")
    try:
        return abs(float(raw))
    except ValueError:
        return None


def _kerb_height(osm_data: dict) -> float:
    kerb    = _get_tag(osm_data, "kerb")
    mapping = {"flush": 0.0, "lowered": 0.03, "raised": 0.15, "yes": 0.10}
    return mapping.get(str(kerb).lower() if kerb else "", 0.0)


def _has_traffic_light(osm_data: dict) -> bool:
    crossing = str(_get_tag(osm_data, "crossing") or "")
    return "traffic" in crossing or "signal" in crossing


def _first_edge_data(G: nx.MultiDiGraph, u: int, v: int) -> dict:
    edges = G.get_edge_data(u, v)
    if edges:
        return next(iter(edges.values()))
    return {}


# ─────────────────────────────────────────────
# 4. חישוב עלות קשת
# ─────────────────────────────────────────────

INF = float("inf")


def compute_edge_cost(data: dict, length: float, cfg: dict) -> float:
    hard   = cfg["hard_blocks"]
    thresh = cfg["thresholds"]

    highway    = _get_tag(data, "highway", "")
    wheelchair = _get_tag(data, "wheelchair", "")
    kerb       = _get_tag(data, "kerb", "")
    barrier    = _get_tag(data, "barrier", "")

    if highway    in hard.get("highway",    []): return INF
    if wheelchair in hard.get("wheelchair", []): return INF
    if kerb       in hard.get("kerb",       []): return INF
    if barrier    in hard.get("barrier",    []): return INF

    width     = _parse_width(_get_tag(data, "width"))
    min_width = thresh.get("minimum_width_meters", 0.9)
    if width is not None and width < min_width:
        return INF

    max_inc     = thresh.get("max_incline_percent", 8.0)
    inc_penalty = thresh.get("incline_penalty_per_percent", 1.2)
    incline_raw = _get_tag(data, "incline")
    incline_num = _parse_incline(incline_raw)

    if incline_num is not None and incline_num > max_inc:
        return INF

    incline_multiplier = 1.0
    if incline_num is not None and incline_num > 0:
        incline_multiplier = 1.0 + (incline_num * (inc_penalty - 1.0) / max_inc)
    elif incline_num is None and incline_raw is not None:
        inc_str_mult       = cfg.get("incline_string_multipliers", {})
        incline_multiplier = inc_str_mult.get(str(incline_raw).strip().lower(), 1.0)

    surface   = _get_tag(data, "surface")
    surf_mult = cfg.get("surface_multipliers", {})
    if surface is not None:
        surface_multiplier = surf_mult.get(str(surface).strip().lower(), 1.0)
    else:
        fallback           = cfg["defaults"].get("fallback_highways", {})
        surface_multiplier = fallback.get(
            str(highway).strip().lower(),
            cfg["defaults"].get("uncertainty_multiplier", 1.2)
        )

    smoothness            = _get_tag(data, "smoothness")
    smooth_mult           = cfg.get("smoothness_multipliers", {})
    smoothness_multiplier = smooth_mult.get(
        str(smoothness).strip().lower() if smoothness else "", 1.0
    )
    if surface is None and smoothness is None:
        smoothness_multiplier = cfg["defaults"].get("uncertainty_multiplier", 1.2)

    return length * surface_multiplier * smoothness_multiplier * incline_multiplier


# ─────────────────────────────────────────────
# 5. חישוב עלות צומת
# ─────────────────────────────────────────────

def compute_node_cost(data: dict, cfg: dict) -> float:
    cost        = 0.0
    kerb        = _get_tag(data, "kerb")
    kerb_costs  = cfg.get("kerb_node_cost_meters", {})
    if kerb is not None:
        cost += kerb_costs.get(str(kerb).strip().lower(), kerb_costs.get("unknown", 15.0))

    crossing    = _get_tag(data, "crossing")
    cross_costs = cfg.get("crossing_node_cost_meters", {})
    if crossing is not None:
        cost += cross_costs.get(str(crossing).strip().lower(), 10.0)

    return cost


# ─────────────────────────────────────────────
# 6. בניית הגרף הממושקל
# ─────────────────────────────────────────────

def build_weighted_graph(G: nx.MultiDiGraph, cfg: dict) -> nx.DiGraph:
    WG = nx.DiGraph()

    for node_id, node_data in G.nodes(data=True):
        WG.add_node(node_id,
                    x=node_data.get("x", 0),
                    y=node_data.get("y", 0),
                    node_cost=compute_node_cost(node_data, cfg))

    for u, v, _, data in G.edges(keys=True, data=True):
        length    = data.get("length", 1.0)
        edge_cost = compute_edge_cost(data, length, cfg)

        if edge_cost == INF:
            continue

        v_node_cost = WG.nodes[v].get("node_cost", 0.0) if v in WG else 0.0
        total_cost  = edge_cost + max(v_node_cost, 0.0)

        if WG.has_edge(u, v):
            if total_cost < WG[u][v]["effort_cost"]:
                WG[u][v]["effort_cost"] = total_cost
        else:
            WG.add_edge(u, v, effort_cost=total_cost, length=length)

    return WG


# ─────────────────────────────────────────────
# 7. ממשק ל-C++ דרך pybind11
#    מחזיר מפה משוקללת בלבד – ללא A*
# ─────────────────────────────────────────────

def build_graph_for_cpp(
    origin: tuple[float, float],
    destination: tuple[float, float],
    config_path: str = "wheelchair_config.json",
    margin: float = 0.005,
) -> dict:
    """
    נקרא מ-C++ דרך pybind11.
    בונה את המפה המשוקללת ומחזיר אותה כ-dict.
    ה-A* רץ בצד C++ על myMap.

    מחזיר:
        {
          "nodes": { node_id: { lat, lon, kerb_height,
                                is_crossing, has_traffic_light } },
          "edges": [ { source, target, effort_cost, length,
                       width, incline, surface, safe_speed } ],
          "source_node": int,   <- צומת OSM הקרוב ל-origin
          "target_node": int    <- צומת OSM הקרוב ל-destination
        }
    """
    print("[GraphMap] טוען קובץ תצורה...")
    cfg = load_config(config_path)

    print("[GraphMap] מוריד גרף OSM...")
    G = fetch_graph(origin, destination, margin)

    print("[GraphMap] בונה גרף מאמץ משוקלל...")
    WG = build_weighted_graph(G, cfg)

    print("[GraphMap] מאתר צמתי מוצא/יעד...")
    nearest_osm = ox.nearest_nodes(G, X=origin[1],      Y=origin[0])
    target_node = ox.nearest_nodes(G, X=destination[1], Y=destination[0])

    # ── צומת וירטואלי במיקום הכיסא ─────────────────────────────────────────
    VIRTUAL_START_ID = 0  # 0 is safe: OSM node IDs always start from 1
    nearest_data = G.nodes[nearest_osm]
    dist_to_nearest = ox.distance.great_circle(
        origin[0], origin[1],
        nearest_data["y"], nearest_data["x"]
    )
    WG.add_node(VIRTUAL_START_ID, x=origin[1], y=origin[0], node_cost=0.0)
    WG.add_edge(VIRTUAL_START_ID, nearest_osm,
                effort_cost=float(dist_to_nearest),
                length=float(dist_to_nearest))
    source_node = VIRTUAL_START_ID
    print(f"[GraphMap] virtual node: lat={origin[0]} lon={origin[1]} -> OSM={nearest_osm} dist={dist_to_nearest:.1f}m")

    # ── צמתים ──────────────────────────────────────────────────────────────
    nodes_out: dict[int, dict] = {}
    nodes_out[VIRTUAL_START_ID] = {
        "lat":               float(origin[0]),
        "lon":               float(origin[1]),
        "kerb_height":       0.0,
        "is_crossing":       False,
        "has_traffic_light": False,
    }
    for node_id, data in WG.nodes(data=True):
        if node_id == VIRTUAL_START_ID:
            continue
        osm_data = G.nodes.get(node_id, {})
        nodes_out[int(node_id)] = {
            "lat":               float(data.get("y", 0.0)),
            "lon":               float(data.get("x", 0.0)),
            "kerb_height":       _kerb_height(osm_data),
            "is_crossing":       bool(_get_tag(osm_data, "crossing")),
            "has_traffic_light": _has_traffic_light(osm_data),
        }

    # ── קשתות ──────────────────────────────────────────────────────────────
    edges_out: list[dict] = []
    for u, v, edata in WG.edges(data=True):
        raw = _first_edge_data(G, u, v)
        edges_out.append({
            "source":      int(u),
            "target":      int(v),
            "effort_cost": float(edata["effort_cost"]),
            "length":      float(edata.get("length", 0.0)),
            "width":       float(_parse_width(_get_tag(raw, "width")) or -1.0),
            "incline":     float(_parse_incline(_get_tag(raw, "incline")) or 0.0),
            "surface":     str(_get_tag(raw, "surface") or "unknown"),
            "safe_speed":  float(raw.get("speed_kph", 5.0)),
        })

    print(f"[GraphMap] גרף מוכן: {len(nodes_out)} צמתים, {len(edges_out)} קשתות")

    return {
        "nodes":       nodes_out,
        "edges":       edges_out,
        "source_node": int(source_node),
        "target_node": int(target_node),
    }
