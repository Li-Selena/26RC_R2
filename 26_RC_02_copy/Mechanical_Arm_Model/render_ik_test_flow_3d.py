#!/usr/bin/env python3
"""Render the current USB-triggered IK test flow as a 3D arm trajectory.

The constants and step list mirror the firmware implementation in
Applications/R2_user/Src/R2_arm.c and Components/Algorithm.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont


Vec3 = Tuple[float, float, float]


ROOT = Path(__file__).resolve().parent
OUT_PNG = ROOT / "robotarm_ik_test_flow_3d.png"
OUT_HTML = ROOT / "robotarm_ik_test_flow_3d.html"
OUT_SIDE_PNG = ROOT / "robotarm_ik_test_flow_side_views.png"


# Firmware kinematics constants, millimeters/radians.
D1_MM = 98.18
J2_Y_MM = 42.64
L23_MM = 383.929445
LONG_LINK_MM = 270.98076211
SHORT_LINK_MM = 124.55
BEND_RAD = math.pi / 6.0
S2_STOW_OFFSET_Y_MM = 92.6
POWERON_LONG_ALPHA_DEG = -174.0
POWERON_TOOL_PHI_DEG = 180.0
SAFE_ALPHA_DEG = 80.0
J2_MOTOR_SIGN = 1.0
J2_MOTOR_PER_JOINT_RAD = 55.0 / 30.0
J3_MOTOR_SIGN = -1.0
J3_MOTOR_PER_JOINT_RAD = 2.0


TEST_STEPS = [
    {
        "index": "P0",
        "label": "P0 actual power-on",
        "direction": "Y+",
        "plane_label": "-YOZ",
        "alpha_deg": POWERON_LONG_ALPHA_DEG,
        "phi_deg": POWERON_TOOL_PHI_DEG,
        "fold": "concave",
        "command": False,
        "color": "#64748b",
    },
    {
        "index": "S0",
        "label": "S0 J2-only to convex Y+ +45deg",
        "direction": "Y+",
        "plane_label": "Y+",
        "alpha_deg": 45.0,
        "hold_poweron_j3": True,
        "fold": "convex",
        "command": True,
        "color": "#0f766e",
    },
    {
        "index": 1,
        "label": "1 Y+ long +45deg IK",
        "direction": "Y+",
        "plane_label": "Y+",
        "alpha_deg": 45.0,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#16a34a",
    },
    {
        "index": "S1",
        "label": "safe lift Y+ +80deg",
        "direction": "Y+",
        "plane_label": "Y+",
        "alpha_deg": SAFE_ALPHA_DEG,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#22c55e",
    },
    {
        "index": "S2",
        "label": "safe switch X+ +80deg",
        "direction": "X+",
        "plane_label": "X+",
        "alpha_deg": SAFE_ALPHA_DEG,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#f59e0b",
    },
    {
        "index": 2,
        "label": "2 X+ long 0deg IK",
        "direction": "X+",
        "plane_label": "X+",
        "alpha_deg": 0.0,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#ea580c",
    },
    {
        "index": "S3",
        "label": "safe lift X+ +80deg",
        "direction": "X+",
        "plane_label": "X+",
        "alpha_deg": SAFE_ALPHA_DEG,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#f59e0b",
    },
    {
        "index": "S4",
        "label": "safe switch X- +80deg",
        "direction": "X-",
        "plane_label": "X-",
        "alpha_deg": SAFE_ALPHA_DEG,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#a855f7",
    },
    {
        "index": 3,
        "label": "3 X- long -30deg IK",
        "direction": "X-",
        "plane_label": "X-",
        "alpha_deg": -30.0,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#9333ea",
    },
    {
        "index": "S5",
        "label": "safe lift X- +80deg",
        "direction": "X-",
        "plane_label": "X-",
        "alpha_deg": SAFE_ALPHA_DEG,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#a855f7",
    },
    {
        "index": "S6",
        "label": "safe switch Y+ +80deg",
        "direction": "Y+",
        "plane_label": "Y+",
        "alpha_deg": SAFE_ALPHA_DEG,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#22c55e",
    },
    {
        "index": 4,
        "label": "4 return Y+ long +45deg IK",
        "direction": "Y+",
        "plane_label": "Y+",
        "alpha_deg": 45.0,
        "phi_deg": 0.0,
        "fold": "convex",
        "command": True,
        "color": "#059669",
    },
]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def scale(a: Vec3, k: float) -> Vec3:
    return (a[0] * k, a[1] * k, a[2] * k)


def long_to_chord_offset_rad() -> float:
    adjacent = LONG_LINK_MM + SHORT_LINK_MM * math.cos(BEND_RAD)
    opposite = SHORT_LINK_MM * math.sin(BEND_RAD)
    return math.atan2(opposite, adjacent)


def normalize_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def poweron_theta2_rad() -> float:
    return normalize_pi(
        math.radians(POWERON_LONG_ALPHA_DEG)
        - long_to_chord_offset_rad()
        - 0.5 * math.pi
    )


def poweron_theta3_rad() -> float:
    return normalize_pi(math.radians(POWERON_TOOL_PHI_DEG) - poweron_theta2_rad())


def model_j2_to_motor_rad(theta2_rad: float) -> float:
    return J2_MOTOR_SIGN * (theta2_rad - poweron_theta2_rad()) * J2_MOTOR_PER_JOINT_RAD


def model_j3_to_motor_rad(theta3_rad: float) -> float:
    return J3_MOTOR_SIGN * (theta3_rad - poweron_theta3_rad()) * J3_MOTOR_PER_JOINT_RAD


def yaw_for_direction(direction: str) -> float:
    if direction == "Y+":
        return 0.0
    if direction == "X+":
        return -0.5 * math.pi
    if direction == "X-":
        return 0.5 * math.pi
    raise ValueError(f"bad direction: {direction}")


def axes_for_yaw(yaw: float) -> Tuple[Vec3, Vec3, Vec3]:
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    x_axis = (cy, sy, 0.0)
    y_axis = (-sy, cy, 0.0)
    z_axis = (0.0, 0.0, 1.0)
    return x_axis, y_axis, z_axis


def pose_from_step(step: Dict[str, object]) -> Dict[str, object]:
    offset_rad = long_to_chord_offset_rad()
    alpha_rad = math.radians(float(step["alpha_deg"]))
    is_command = bool(step.get("command", True))
    is_concave = str(step.get("fold", "convex")) == "concave"
    hold_poweron_j3 = bool(step.get("hold_poweron_j3", False))

    direction = str(step["direction"])
    yaw_rad = yaw_for_direction(direction)
    x_axis, y_axis, z_axis = axes_for_yaw(yaw_rad)

    o0 = (0.0, 0.0, 0.0)
    o1 = (0.0, 0.0, D1_MM)
    j2 = add(o1, scale(y_axis, J2_Y_MM))

    elbow = add(
        j2,
        add(
            scale(y_axis, LONG_LINK_MM * math.cos(alpha_rad)),
            scale(z_axis, LONG_LINK_MM * math.sin(alpha_rad)),
        ),
    )
    short_alpha_rad = alpha_rad - BEND_RAD
    j4 = add(
        elbow,
        add(
            scale(y_axis, SHORT_LINK_MM * math.cos(short_alpha_rad)),
            scale(z_axis, SHORT_LINK_MM * math.sin(short_alpha_rad)),
        ),
    )

    gamma_rad = alpha_rad - offset_rad
    radial_mm = L23_MM * math.cos(gamma_rad)
    height_mm = L23_MM * math.sin(gamma_rad)
    j4_chord = add(j2, add(scale(y_axis, radial_mm), scale(z_axis, height_mm)))

    theta2_rad = normalize_pi(gamma_rad - 0.5 * math.pi)
    if not is_command:
        theta2_rad = poweron_theta2_rad()
        theta3_rad = poweron_theta3_rad()
        phi_rad = math.radians(POWERON_TOOL_PHI_DEG)
    elif hold_poweron_j3:
        theta3_rad = poweron_theta3_rad()
        phi_rad = normalize_pi(theta2_rad + theta3_rad)
    else:
        phi_rad = math.radians(float(step.get("phi_deg", 0.0)))
        theta3_rad = normalize_pi(phi_rad - theta2_rad)

    y_tool_axis = add(scale(y_axis, math.cos(phi_rad)), scale(z_axis, math.sin(phi_rad)))
    tool = add(j4, scale(y_tool_axis, S2_STOW_OFFSET_Y_MM))

    return {
        "index": step["index"],
        "label": step["label"],
        "direction": direction,
        "plane_label": step.get("plane_label", direction),
        "fold": "concave" if is_concave else "convex",
        "color": step["color"],
        "command": is_command,
        "alpha_deg": math.degrees(alpha_rad),
        "phi_deg": math.degrees(phi_rad),
        "yaw_deg": math.degrees(yaw_rad),
        "theta2_deg": math.degrees(theta2_rad),
        "theta3_deg": math.degrees(theta3_rad),
        "motor1_deg": math.degrees(yaw_rad),
        "motor2_deg": 0.0 if not is_command else math.degrees(model_j2_to_motor_rad(theta2_rad)),
        "motor3_deg": 0.0 if not is_command else math.degrees(model_j3_to_motor_rad(theta3_rad)),
        "tool_xyz": tool,
        "j4_error_mm": distance(j4, j4_chord),
        "points": {
            "O0": o0,
            "O1": o1,
            "J2": j2,
            "Elbow": elbow,
            "J4": j4,
            "S2": tool,
            "J4Chord": j4_chord,
        },
        "segments": [
            ("O0", "O1", "base"),
            ("O1", "J2", "j2_offset"),
            ("J2", "Elbow", "long_link"),
            ("Elbow", "J4", "short_link"),
            ("J4", "S2", "tool_offset"),
            ("J2", "J4Chord", "equivalent_chord"),
        ],
    }


def distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


def hex_to_rgb(hex_color: str) -> Tuple[int, int, int]:
    hex_color = hex_color.lstrip("#")
    return (
        int(hex_color[0:2], 16),
        int(hex_color[2:4], 16),
        int(hex_color[4:6], 16),
    )


def project(point: Vec3, azim_deg: float = -38.0, elev_deg: float = 27.0) -> Tuple[float, float]:
    x, y, z = point
    az = math.radians(azim_deg)
    el = math.radians(elev_deg)
    x1 = x * math.cos(az) - y * math.sin(az)
    y1 = x * math.sin(az) + y * math.cos(az)
    y2 = y1 * math.cos(el) - z * math.sin(el)
    return (x1, -y2)


def all_world_points(poses: Sequence[Dict[str, object]]) -> List[Vec3]:
    pts: List[Vec3] = []
    for pose in poses:
        pts.extend(pose["points"].values())  # type: ignore[union-attr]
    limit = 620.0
    pts.extend(
        [
            (-limit, 0.0, 0.0),
            (limit, 0.0, 0.0),
            (0.0, -limit, 0.0),
            (0.0, limit, 0.0),
            (0.0, 0.0, -80.0),
            (0.0, 0.0, 560.0),
        ]
    )
    return pts


def make_view_transform(
    poses: Sequence[Dict[str, object]],
    width: int,
    height: int,
    left_panel_w: int,
    margin: int,
) -> Tuple[float, float, float]:
    projected = [project(p) for p in all_world_points(poses)]
    min_x = min(p[0] for p in projected)
    max_x = max(p[0] for p in projected)
    min_y = min(p[1] for p in projected)
    max_y = max(p[1] for p in projected)
    plot_w = width - left_panel_w - 2 * margin
    plot_h = height - 2 * margin
    s = min(plot_w / (max_x - min_x), plot_h / (max_y - min_y))
    ox = left_panel_w + margin + (plot_w - (max_x - min_x) * s) * 0.5 - min_x * s
    oy = margin + (plot_h - (max_y - min_y) * s) * 0.5 - min_y * s
    return s, ox, oy


def screen(
    point: Vec3, scale_px: float, ox: float, oy: float, aa_scale: int
) -> Tuple[int, int]:
    px, py = project(point)
    return (int(round((px * scale_px + ox) * aa_scale)), int(round((py * scale_px + oy) * aa_scale)))


def draw_line(
    draw: ImageDraw.ImageDraw,
    a: Vec3,
    b: Vec3,
    view: Tuple[float, float, float],
    aa_scale: int,
    fill: Tuple[int, int, int, int],
    width: int,
) -> None:
    draw.line(
        [screen(a, *view, aa_scale), screen(b, *view, aa_scale)],
        fill=fill,
        width=width * aa_scale,
    )


def draw_dashed_line(
    draw: ImageDraw.ImageDraw,
    a: Vec3,
    b: Vec3,
    view: Tuple[float, float, float],
    aa_scale: int,
    fill: Tuple[int, int, int, int],
    width: int,
    dash_mm: float = 28.0,
) -> None:
    length = distance(a, b)
    if length <= 1.0:
        return
    count = max(1, int(length / dash_mm))
    for i in range(count):
        if i % 2 != 0:
            continue
        t0 = i / count
        t1 = min(1.0, (i + 1) / count)
        p0 = (
            a[0] + (b[0] - a[0]) * t0,
            a[1] + (b[1] - a[1]) * t0,
            a[2] + (b[2] - a[2]) * t0,
        )
        p1 = (
            a[0] + (b[0] - a[0]) * t1,
            a[1] + (b[1] - a[1]) * t1,
            a[2] + (b[2] - a[2]) * t1,
        )
        draw_line(draw, p0, p1, view, aa_scale, fill, width)


def draw_marker(
    draw: ImageDraw.ImageDraw,
    p: Vec3,
    view: Tuple[float, float, float],
    aa_scale: int,
    fill: Tuple[int, int, int, int],
    radius: int,
) -> None:
    x, y = screen(p, *view, aa_scale)
    r = radius * aa_scale
    draw.ellipse((x - r, y - r, x + r, y + r), fill=fill, outline=(255, 255, 255, 240), width=2 * aa_scale)


def load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/calibrib.ttf" if bold else "C:/Windows/Fonts/calibri.ttf"),
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    ]
    for path in candidates:
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def draw_static_png(poses: Sequence[Dict[str, object]]) -> None:
    width, height = 1900, 1820
    aa_scale = 2
    left_panel_w = 760
    margin = 80
    img = Image.new("RGBA", (width * aa_scale, height * aa_scale), (250, 252, 255, 255))
    draw = ImageDraw.Draw(img, "RGBA")
    view = make_view_transform(poses, width, height, left_panel_w, margin)

    title_font = load_font(30 * aa_scale, bold=True)
    header_font = load_font(18 * aa_scale, bold=True)
    text_font = load_font(16 * aa_scale, bold=False)
    small_font = load_font(14 * aa_scale, bold=False)

    # Left information panel.
    panel = (36 * aa_scale, 36 * aa_scale, (left_panel_w - 28) * aa_scale, (height - 36) * aa_scale)
    draw.rounded_rectangle(panel, radius=18 * aa_scale, fill=(255, 255, 255, 245), outline=(210, 219, 235, 255), width=2 * aa_scale)
    draw.text((62 * aa_scale, 62 * aa_scale), "Robot Arm IK Test Flow", fill=(20, 28, 45, 255), font=title_font)
    draw.text((62 * aa_scale, 108 * aa_scale), "P0 is actual motor zero: long link on Y- and 6deg down", fill=(70, 82, 105, 255), font=text_font)
    draw.text((62 * aa_scale, 134 * aa_scale), "S0 only turns J2 to the convex Y+ side; official test points use S2/STOW IK", fill=(70, 82, 105, 255), font=text_font)
    draw.text((62 * aa_scale, 160 * aa_scale), "J1 mapping: Y+=0deg, X+=-90deg, X-=+90deg", fill=(70, 82, 105, 255), font=text_font)
    draw.text((62 * aa_scale, 196 * aa_scale), "Trajectory coordinates and motor targets", fill=(20, 28, 45, 255), font=header_font)

    y = 232
    for pose in poses:
        color = hex_to_rgb(str(pose["color"]))
        draw.rounded_rectangle(
            (62 * aa_scale, y * aa_scale, 94 * aa_scale, (y + 24) * aa_scale),
            radius=7 * aa_scale,
            fill=color + (230,),
        )
        draw.text((70 * aa_scale, (y + 2) * aa_scale), str(pose["index"]), fill=(255, 255, 255, 255), font=small_font)
        draw.text((108 * aa_scale, (y - 1) * aa_scale), str(pose["label"]), fill=(20, 28, 45, 255), font=header_font)
        x, yy, z = pose["tool_xyz"]  # type: ignore[misc]
        row = f"XYZ=({x:7.1f}, {yy:7.1f}, {z:7.1f})"
        row2 = (
            f"plane={str(pose['plane_label']):>4}  fold={str(pose['fold']):>7}  "
            f"alpha={pose['alpha_deg']:6.1f}"
        )
        row2b = f"phi={pose['phi_deg']:6.1f}  J1={pose['yaw_deg']:6.1f}"
        row3 = f"motor=({pose['motor1_deg']:6.1f}, {pose['motor2_deg']:6.1f}, {pose['motor3_deg']:6.1f}) deg"
        draw.text((108 * aa_scale, (y + 27) * aa_scale), row, fill=(54, 65, 83, 255), font=text_font)
        draw.text((108 * aa_scale, (y + 52) * aa_scale), row2, fill=(54, 65, 83, 255), font=text_font)
        draw.text((108 * aa_scale, (y + 77) * aa_scale), row2b, fill=(54, 65, 83, 255), font=small_font)
        draw.text((108 * aa_scale, (y + 99) * aa_scale), row3, fill=(54, 65, 83, 255), font=small_font)
        y += 118

    draw.text((62 * aa_scale, (height - 102) * aa_scale), "Solid segments show the physical long/short bent link.", fill=(20, 28, 45, 255), font=header_font)
    draw.text((62 * aa_scale, (height - 72) * aa_scale), "Dashed segment = equivalent L23 chord used by IK.", fill=(70, 82, 105, 255), font=text_font)
    draw.text((62 * aa_scale, (height - 46) * aa_scale), "P0 is reference; S0 is startup; numbered 1-4 are the requested test poses.", fill=(70, 82, 105, 255), font=text_font)

    # Ground plane and world axes.
    grid_color = (200, 211, 226, 120)
    axis_x = (220, 38, 38, 230)
    axis_y = (22, 125, 70, 230)
    axis_z = (37, 99, 235, 230)
    grid_limit = 600
    for v in range(-600, 601, 100):
        draw_line(draw, (-grid_limit, v, 0), (grid_limit, v, 0), view, aa_scale, grid_color, 1)
        draw_line(draw, (v, -grid_limit, 0), (v, grid_limit, 0), view, aa_scale, grid_color, 1)
    draw_line(draw, (-grid_limit, 0, 0), (grid_limit, 0, 0), view, aa_scale, axis_x, 3)
    draw_line(draw, (0, -grid_limit, 0), (0, grid_limit, 0), view, aa_scale, axis_y, 3)
    draw_line(draw, (0, 0, -80), (0, 0, 560), view, aa_scale, axis_z, 3)

    def axis_label(text: str, p: Vec3, color: Tuple[int, int, int, int]) -> None:
        sx, sy = screen(p, *view, aa_scale)
        draw.text((sx + 8 * aa_scale, sy - 8 * aa_scale), text, fill=color, font=header_font)

    axis_label("X+", (grid_limit, 0, 0), axis_x)
    axis_label("Y+", (0, grid_limit, 0), axis_y)
    axis_label("Z+", (0, 0, 560), axis_z)

    # Trajectory line through S2 target points.
    tool_points = [pose["points"]["S2"] for pose in poses]  # type: ignore[index]
    for a, b in zip(tool_points, tool_points[1:]):
        draw_line(draw, a, b, view, aa_scale, (15, 23, 42, 180), 3)

    # Arm skeletons.
    for pose in poses:
        points = pose["points"]  # type: ignore[assignment]
        rgb = hex_to_rgb(str(pose["color"]))
        body = rgb + (210,)
        faint = rgb + (115,)
        dark = (24, 31, 45, 230)
        draw_line(draw, points["O0"], points["O1"], view, aa_scale, (82, 95, 117, 180), 6)
        draw_line(draw, points["O1"], points["J2"], view, aa_scale, faint, 5)
        draw_line(draw, points["J2"], points["Elbow"], view, aa_scale, body, 8)
        draw_line(draw, points["Elbow"], points["J4"], view, aa_scale, body, 6)
        draw_line(draw, points["J4"], points["S2"], view, aa_scale, dark, 4)
        draw_dashed_line(draw, points["J2"], points["J4Chord"], view, aa_scale, (71, 85, 105, 125), 2)
        for name in ("J2", "Elbow", "J4"):
            draw_marker(draw, points[name], view, aa_scale, rgb + (210,), 5)
        draw_marker(draw, points["S2"], view, aa_scale, rgb + (245,), 9)
        sx, sy = screen(points["S2"], *view, aa_scale)
        draw.text((sx + 12 * aa_scale, sy - 24 * aa_scale), str(pose["index"]), fill=(15, 23, 42, 255), font=header_font)

    img = img.resize((width, height), Image.Resampling.LANCZOS)
    img.convert("RGB").save(OUT_PNG, "PNG")


def side_h(pose: Dict[str, object], point: Vec3) -> float:
    plane = str(pose["plane_label"])
    if plane == "X+":
        return point[0]
    if plane == "X-":
        return -point[0]
    return point[1]


def draw_side_views(poses: Sequence[Dict[str, object]]) -> None:
    width, height = 1900, 980
    aa_scale = 2
    img = Image.new("RGBA", (width * aa_scale, height * aa_scale), (250, 252, 255, 255))
    draw = ImageDraw.Draw(img, "RGBA")
    title_font = load_font(26 * aa_scale, bold=True)
    header_font = load_font(17 * aa_scale, bold=True)
    text_font = load_font(14 * aa_scale, bold=False)

    panels = [
        ("-YOZ power-on reference", "-YOZ"),
        ("Y+ side view, convex after S0", "Y+"),
        ("X+ side view, convex", "X+"),
        ("X- side view, convex", "X-"),
    ]
    panel_w = width / 4.0
    margin = 44

    for panel_i, (title, plane) in enumerate(panels):
        x0 = int(panel_i * panel_w)
        x1 = int((panel_i + 1) * panel_w)
        panel_poses = [p for p in poses if str(p["plane_label"]) == plane]
        if not panel_poses:
            continue

        pts_2d: List[Tuple[float, float]] = []
        for pose in panel_poses:
            points = pose["points"]  # type: ignore[assignment]
            for point in points.values():
                pts_2d.append((side_h(pose, point), point[2]))
        pts_2d.extend([(0.0, 0.0), (540.0, 0.0), (-460.0, 0.0), (0.0, 520.0), (0.0, -170.0)])
        min_h = min(p[0] for p in pts_2d)
        max_h = max(p[0] for p in pts_2d)
        min_z = min(p[1] for p in pts_2d)
        max_z = max(p[1] for p in pts_2d)
        plot_w = panel_w - 2 * margin
        plot_h = height - 150
        scale_px = min(plot_w / max(1.0, max_h - min_h), plot_h / max(1.0, max_z - min_z))
        ox = x0 + margin + (plot_w - (max_h - min_h) * scale_px) * 0.5 - min_h * scale_px
        oy = 108 + (plot_h + (max_z + min_z) * scale_px) * 0.5

        def sp(pose: Dict[str, object], point: Vec3) -> Tuple[int, int]:
            h = side_h(pose, point)
            return (int(round((h * scale_px + ox) * aa_scale)),
                    int(round((oy - point[2] * scale_px) * aa_scale)))

        draw.rounded_rectangle(
            ((x0 + 18) * aa_scale, 28 * aa_scale, (x1 - 18) * aa_scale, (height - 28) * aa_scale),
            radius=14 * aa_scale,
            fill=(255, 255, 255, 245),
            outline=(211, 220, 233, 255),
            width=2 * aa_scale,
        )
        draw.text(((x0 + 38) * aa_scale, 50 * aa_scale), title, fill=(20, 28, 45, 255), font=title_font)
        axis_color = (22, 125, 70, 230) if plane in ("Y+", "-YOZ") else (220, 38, 38, 230)
        z_color = (37, 99, 235, 230)

        # Grid and axes.
        for h in range(-500, 601, 100):
            pa = (h, min_z - 40.0, 0.0)
            pb = (h, max_z + 40.0, 0.0)
            x_a = int(round((h * scale_px + ox) * aa_scale))
            draw.line([(x_a, int(116 * aa_scale)), (x_a, int((height - 60) * aa_scale))],
                      fill=(200, 211, 226, 95), width=aa_scale)
        for z in range(-100, 551, 100):
            y = int(round((oy - z * scale_px) * aa_scale))
            draw.line([(int((x0 + 30) * aa_scale), y), (int((x1 - 30) * aa_scale), y)],
                      fill=(200, 211, 226, 95), width=aa_scale)
        draw.line([(int((0 * scale_px + ox) * aa_scale), int(116 * aa_scale)),
                   (int((0 * scale_px + ox) * aa_scale), int((height - 60) * aa_scale))],
                  fill=z_color, width=3 * aa_scale)
        draw.line([(int((x0 + 30) * aa_scale), int((oy - 0 * scale_px) * aa_scale)),
                   (int((x1 - 30) * aa_scale), int((oy - 0 * scale_px) * aa_scale))],
                  fill=axis_color, width=3 * aa_scale)
        axis_name = "+Y" if plane == "Y+" else ("-Y" if plane == "-YOZ" else plane)
        draw.text(((x1 - 82) * aa_scale, (oy - 26 * aa_scale / aa_scale) * aa_scale), axis_name,
                  fill=axis_color, font=header_font)
        draw.text(((x0 + 50) * aa_scale, 118 * aa_scale), "Z+", fill=z_color, font=header_font)

        for pose in panel_poses:
            points = pose["points"]  # type: ignore[assignment]
            rgb = hex_to_rgb(str(pose["color"]))
            main = rgb + (230,)
            black = (18, 25, 38, 225)
            # Draw chord first, then physical long/short bent link.
            draw.line([sp(pose, points["J2"]), sp(pose, points["J4Chord"])],
                      fill=(118, 132, 150, 150), width=2 * aa_scale)
            draw.line([sp(pose, points["J2"]), sp(pose, points["Elbow"]), sp(pose, points["J4"])],
                      fill=main, width=8 * aa_scale, joint="curve")
            draw.line([sp(pose, points["J4"]), sp(pose, points["S2"])],
                      fill=black, width=3 * aa_scale)
            for name in ("J2", "Elbow", "J4", "S2"):
                x, y = sp(pose, points[name])
                r = (8 if name == "S2" else 5) * aa_scale
                draw.ellipse((x - r, y - r, x + r, y + r),
                             fill=rgb + (245,), outline=(255, 255, 255, 245), width=2 * aa_scale)
            x, y = sp(pose, points["S2"])
            draw.text((x + 8 * aa_scale, y - 22 * aa_scale), str(pose["index"]),
                      fill=(15, 23, 42, 255), font=header_font)

        note = "concave allowed only here" if plane == "-YOZ" else "all shown links are convex"
        draw.text(((x0 + 38) * aa_scale, (height - 62) * aa_scale),
                  note, fill=(71, 85, 105, 255), font=text_font)

    img = img.resize((width, height), Image.Resampling.LANCZOS)
    img.convert("RGB").save(OUT_SIDE_PNG, "PNG")


def rounded_pose_for_json(pose: Dict[str, object]) -> Dict[str, object]:
    points = pose["points"]  # type: ignore[assignment]
    return {
        "index": pose["index"],
        "label": pose["label"],
        "direction": pose["direction"],
        "planeLabel": pose["plane_label"],
        "fold": pose["fold"],
        "color": pose["color"],
        "alphaDeg": round(float(pose["alpha_deg"]), 3),
        "yawDeg": round(float(pose["yaw_deg"]), 3),
        "theta2Deg": round(float(pose["theta2_deg"]), 3),
        "theta3Deg": round(float(pose["theta3_deg"]), 3),
        "phiDeg": round(float(pose["phi_deg"]), 3),
        "motor1Deg": round(float(pose["motor1_deg"]), 3),
        "motor2Deg": round(float(pose["motor2_deg"]), 3),
        "motor3Deg": round(float(pose["motor3_deg"]), 3),
        "command": bool(pose["command"]),
        "points": {
            name: [round(float(v), 3) for v in point]
            for name, point in points.items()
        },
        "segments": pose["segments"],
    }


def write_interactive_html(poses: Sequence[Dict[str, object]]) -> None:
    data = {
        "poses": [rounded_pose_for_json(pose) for pose in poses],
        "constants": {
            "D1_MM": D1_MM,
            "J2_Y_MM": J2_Y_MM,
            "LONG_LINK_MM": LONG_LINK_MM,
            "SHORT_LINK_MM": SHORT_LINK_MM,
            "S2_STOW_OFFSET_Y_MM": S2_STOW_OFFSET_Y_MM,
            "longToChordOffsetDeg": math.degrees(long_to_chord_offset_rad()),
        },
    }
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Arm IK Test Flow 3D</title>
  <style>
    :root {{
      color-scheme: light;
      --ink: #0f172a;
      --muted: #64748b;
      --line: #d8e0ed;
      --panel: #ffffff;
      --bg: #f8fafc;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      min-height: 100vh;
      font-family: Arial, Helvetica, sans-serif;
      color: var(--ink);
      background: var(--bg);
      display: grid;
      grid-template-columns: 560px 1fr;
    }}
    aside {{
      border-right: 1px solid var(--line);
      background: var(--panel);
      padding: 22px;
      overflow: auto;
    }}
    main {{ min-width: 0; display: grid; grid-template-rows: auto 1fr; }}
    header {{
      padding: 16px 20px;
      border-bottom: 1px solid var(--line);
      background: rgba(255, 255, 255, 0.82);
    }}
    h1 {{ margin: 0 0 6px; font-size: 22px; letter-spacing: 0; }}
    p {{ margin: 0; color: var(--muted); line-height: 1.45; }}
    canvas {{
      width: 100%;
      height: 100%;
      min-height: 640px;
      display: block;
      cursor: grab;
    }}
    canvas:active {{ cursor: grabbing; }}
    table {{ width: 100%; border-collapse: collapse; margin-top: 18px; font-size: 13px; }}
    th, td {{ border-bottom: 1px solid var(--line); padding: 7px 4px; text-align: right; white-space: nowrap; }}
    th:first-child, td:first-child {{ text-align: left; }}
    .dot {{ display: inline-block; width: 12px; height: 12px; border-radius: 999px; margin-right: 6px; vertical-align: -1px; }}
    .meta {{ margin-top: 18px; display: grid; gap: 7px; font-size: 13px; color: var(--muted); }}
    @media (max-width: 900px) {{
      body {{ grid-template-columns: 1fr; }}
      aside {{ border-right: 0; border-bottom: 1px solid var(--line); }}
      canvas {{ min-height: 520px; }}
    }}
  </style>
</head>
<body>
  <aside>
    <h1>IK Test Flow</h1>
    <p>Drag the canvas to rotate. P0 is actual motor zero, S0 is the J2-only startup move, and numbered points are the requested IK test poses.</p>
    <div class="meta">
      <div>J1: Y+=0 deg, X+=-90 deg, X-=+90 deg</div>
      <div>S0 flips from concave Y- to convex Y+; after that all poses stay convex</div>
      <div>Numbered commands use S2/STOW; yaw switches happen only at alpha=80 deg</div>
      <div>Solid long/short segments: physical crank structure; dashed segment: IK L23 chord</div>
    </div>
    <table>
      <thead>
        <tr><th>Step</th><th>Plane</th><th>Fold</th><th>X</th><th>Y</th><th>Z</th><th>J1</th><th>alpha</th><th>M2</th><th>M3</th></tr>
      </thead>
      <tbody id="rows"></tbody>
    </table>
  </aside>
  <main>
    <header>
      <h1>Robot Arm Physical Structure and S2 Trajectory</h1>
      <p>Starts from the actual concave Y- power-on pose, uses J2 to flip into the convex Y+ 45deg side, then runs the IK test with safe alpha=80deg waypoints before each yaw switch.</p>
    </header>
    <canvas id="view"></canvas>
  </main>
  <script>
    const DATA = {json.dumps(data, indent=6)};
    const canvas = document.getElementById('view');
    const ctx = canvas.getContext('2d');
    let az = -0.72;
    let el = 0.46;
    let dragging = false;
    let lastX = 0;
    let lastY = 0;

    function rotate(p) {{
      const [x, y, z] = p;
      const caz = Math.cos(az), saz = Math.sin(az);
      const x1 = x * caz - y * saz;
      const y1 = x * saz + y * caz;
      const cel = Math.cos(el), sel = Math.sin(el);
      const y2 = y1 * cel - z * sel;
      const z2 = y1 * sel + z * cel;
      return [x1, -y2, z2];
    }}

    function fit(points) {{
      const pr = points.map(rotate);
      const xs = pr.map(p => p[0]);
      const ys = pr.map(p => p[1]);
      const pad = 74;
      const scale = Math.min((canvas.width - pad * 2) / (Math.max(...xs) - Math.min(...xs)),
                             (canvas.height - pad * 2) / (Math.max(...ys) - Math.min(...ys)));
      const ox = canvas.width / 2 - (Math.max(...xs) + Math.min(...xs)) * 0.5 * scale;
      const oy = canvas.height / 2 - (Math.max(...ys) + Math.min(...ys)) * 0.5 * scale;
      return {{scale, ox, oy}};
    }}

    function worldToScreen(p, view) {{
      const q = rotate(p);
      return [q[0] * view.scale + view.ox, q[1] * view.scale + view.oy, q[2]];
    }}

    function line(a, b, view, color, width, dash=false) {{
      const pa = worldToScreen(a, view);
      const pb = worldToScreen(b, view);
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineCap = 'round';
      if (dash) ctx.setLineDash([9, 9]);
      ctx.beginPath();
      ctx.moveTo(pa[0], pa[1]);
      ctx.lineTo(pb[0], pb[1]);
      ctx.stroke();
      ctx.restore();
    }}

    function marker(p, view, color, r=6) {{
      const q = worldToScreen(p, view);
      ctx.beginPath();
      ctx.arc(q[0], q[1], r, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
    }}

    function label(text, p, view, color='#0f172a') {{
      const q = worldToScreen(p, view);
      ctx.font = '600 14px Arial';
      ctx.fillStyle = color;
      ctx.fillText(text, q[0] + 10, q[1] - 10);
    }}

    function render() {{
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.floor(rect.width * dpr);
      canvas.height = Math.floor(rect.height * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const w = rect.width;
      const h = rect.height;
      ctx.clearRect(0, 0, w, h);

      const allPoints = [];
      DATA.poses.forEach(pose => Object.values(pose.points).forEach(p => allPoints.push(p)));
      const limit = 620;
      allPoints.push([-limit, 0, 0], [limit, 0, 0], [0, -limit, 0], [0, limit, 0], [0, 0, -80], [0, 0, 560]);
      const view = fit(allPoints);
      view.scale /= dpr;
      view.ox /= dpr;
      view.oy /= dpr;

      ctx.fillStyle = '#f8fafc';
      ctx.fillRect(0, 0, w, h);
      for (let v = -600; v <= 600; v += 100) {{
        line([-600, v, 0], [600, v, 0], view, 'rgba(148,163,184,.35)', 1);
        line([v, -600, 0], [v, 600, 0], view, 'rgba(148,163,184,.35)', 1);
      }}
      line([-620, 0, 0], [620, 0, 0], view, '#dc2626', 2.5);
      line([0, -620, 0], [0, 620, 0], view, '#15803d', 2.5);
      line([0, 0, -80], [0, 0, 560], view, '#2563eb', 2.5);
      label('X+', [620, 0, 0], view, '#dc2626');
      label('Y+', [0, 620, 0], view, '#15803d');
      label('Z+', [0, 0, 560], view, '#2563eb');

      for (let i = 0; i < DATA.poses.length - 1; i++) {{
        line(DATA.poses[i].points.S2, DATA.poses[i + 1].points.S2, view, 'rgba(15,23,42,.72)', 3);
      }}

      DATA.poses.forEach(pose => {{
        const p = pose.points;
        line(p.O0, p.O1, view, 'rgba(71,85,105,.65)', 6);
        line(p.O1, p.J2, view, pose.color + 'aa', 5);
        line(p.J2, p.Elbow, view, pose.color, 8);
        line(p.Elbow, p.J4, view, pose.color, 6);
        line(p.J4, p.S2, view, 'rgba(15,23,42,.9)', 4);
        line(p.J2, p.J4Chord, view, 'rgba(71,85,105,.5)', 2, true);
        marker(p.J2, view, pose.color, 4);
        marker(p.Elbow, view, pose.color, 4);
        marker(p.J4, view, pose.color, 5);
        marker(p.S2, view, pose.color, 8);
        label(String(pose.index), p.S2, view);
      }});
    }}

    function fillRows() {{
      const body = document.getElementById('rows');
      body.innerHTML = DATA.poses.map(pose => {{
        const s2 = pose.points.S2;
        return `<tr>
          <td><span class="dot" style="background:${{pose.color}}"></span>${{pose.index}}</td>
          <td>${{pose.planeLabel}}</td>
          <td>${{pose.fold}}</td>
          <td>${{s2[0].toFixed(1)}}</td>
          <td>${{s2[1].toFixed(1)}}</td>
          <td>${{s2[2].toFixed(1)}}</td>
          <td>${{pose.yawDeg.toFixed(1)}}</td>
          <td>${{pose.alphaDeg.toFixed(1)}}</td>
          <td>${{pose.motor2Deg.toFixed(1)}}</td>
          <td>${{pose.motor3Deg.toFixed(1)}}</td>
        </tr>`;
      }}).join('');
    }}

    canvas.addEventListener('pointerdown', ev => {{
      dragging = true;
      lastX = ev.clientX;
      lastY = ev.clientY;
      canvas.setPointerCapture(ev.pointerId);
    }});
    canvas.addEventListener('pointermove', ev => {{
      if (!dragging) return;
      az += (ev.clientX - lastX) * 0.008;
      el += (ev.clientY - lastY) * 0.008;
      el = Math.max(-1.2, Math.min(1.2, el));
      lastX = ev.clientX;
      lastY = ev.clientY;
      render();
    }});
    canvas.addEventListener('pointerup', ev => {{
      dragging = false;
      canvas.releasePointerCapture(ev.pointerId);
    }});
    window.addEventListener('resize', render);
    fillRows();
    render();
  </script>
</body>
</html>
"""
    OUT_HTML.write_text(html, encoding="utf-8")


def validate_convex_after_startup(poses: Sequence[Dict[str, object]]) -> List[str]:
    violations: List[str] = []
    startup_index = None

    for i, pose in enumerate(poses):
        if pose["index"] == "S0":
            startup_index = i
            break

    if startup_index is None:
        return ["missing S0 startup pose"]

    for pose in poses[startup_index:]:
        if pose["fold"] != "convex":
            violations.append(f"{pose['index']} is {pose['fold']}, expected convex")
        if pose["plane_label"] == "-YOZ":
            violations.append(f"{pose['index']} is in -YOZ after startup")

    for a, b in zip(poses[startup_index:], poses[startup_index + 1:]):
        yaw_a = math.radians(float(a["yaw_deg"]))
        yaw_b = math.radians(float(b["yaw_deg"]))
        for n in range(21):
            t = n / 20.0
            yaw = yaw_a + (yaw_b - yaw_a) * t
            if math.cos(yaw) < -1.0e-6:
                violations.append(
                    f"yaw segment {a['index']}->{b['index']} enters -Y half-plane"
                )
                break

    return violations


def print_summary(poses: Sequence[Dict[str, object]]) -> None:
    print(f"long_to_chord_offset_deg={math.degrees(long_to_chord_offset_rad()):.6f}")
    print(f"poweron_long_alpha_deg={POWERON_LONG_ALPHA_DEG:.6f}")
    print(f"poweron_theta2_deg={math.degrees(poweron_theta2_rad()):.6f}")
    print(f"poweron_theta3_deg={math.degrees(poweron_theta3_rad()):.6f}")
    for pose in poses:
        x, y, z = pose["tool_xyz"]  # type: ignore[misc]
        print(
            f"step{pose['index']}: {pose['label']}, "
            f"plane={pose['plane_label']}, "
            f"fold={pose['fold']}, "
            f"xyz=({x:.3f},{y:.3f},{z:.3f}), "
            f"J1={pose['yaw_deg']:.3f}deg, "
            f"alpha={pose['alpha_deg']:.3f}deg, "
            f"J2={pose['theta2_deg']:.3f}deg, "
            f"J3={pose['theta3_deg']:.3f}deg, "
            f"motor=({pose['motor1_deg']:.3f},"
            f"{pose['motor2_deg']:.3f},"
            f"{pose['motor3_deg']:.3f})deg, "
            f"chord_error={pose['j4_error_mm']:.6f}mm"
        )
    violations = validate_convex_after_startup(poses)
    if violations:
        print("post_startup_convex_check=FAIL")
        for violation in violations:
            print(f"  {violation}")
    else:
        print("post_startup_convex_check=OK")
    print(f"wrote {OUT_PNG}")
    print(f"wrote {OUT_SIDE_PNG}")
    print(f"wrote {OUT_HTML}")


def main() -> None:
    poses = [pose_from_step(step) for step in TEST_STEPS]
    draw_static_png(poses)
    draw_side_views(poses)
    write_interactive_html(poses)
    print_summary(poses)


if __name__ == "__main__":
    main()
