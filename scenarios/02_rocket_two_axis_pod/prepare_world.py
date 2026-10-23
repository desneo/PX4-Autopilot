#!/usr/bin/env python3
"""从资产仓地理 world 生成包含多架火箭机两轴吊仓的运行 world。"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_NAME = "geo_32_217857_119_105878"
MODEL_BASENAME = "rocket_two_axis_pod"
TRACKING_BACKENDS = {"sim_truth", "csrt"}
SIM_TRUTH_DYNAMIC_TARGETS = ("xiaomotuo", "xiaobianfu")
TARGET_COMMAND_TOPIC = "/scenario/02/target/command"
TARGET_STATUS_TOPIC = "/scenario/02/target/status"
TRAJECTORY_COMMAND_TOPIC = "/scenario/02/target/trajectory/command"
TRAJECTORY_STATUS_TOPIC = "/scenario/02/target/trajectory/status"
COLLISION_EVENT_TOPIC = "/scenario/02/collision/event"
RADAR_COMMAND_TOPIC = "/scenario/02/radar/command"
RADAR_STATUS_TOPIC = "/scenario/02/radar/status"


def set_text(parent: ET.Element, tag: str, value: object) -> ET.Element:
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def absolute_resources(element: ET.Element, source_file: Path) -> None:
    """把运行副本使用的相对资源改为源仓库中的绝对路径。"""
    resource_tags = {
        "uri",
        "albedo_map",
        "normal_map",
        "roughness_map",
        "metalness_map",
        "emissive_map",
        "environment_map",
    }
    for item in element.iter():
        if item.tag not in resource_tags or not item.text:
            continue
        value = item.text.strip()
        if not value or "://" in value and not value.startswith(("model://", "file://")):
            continue
        if value.startswith("model://"):
            relative = value.removeprefix("model://")
            path = source_file.parent.parent / relative
        elif value.startswith("file://"):
            path = Path(value.removeprefix("file://"))
            if not path.is_absolute():
                path = source_file.parent / path
        else:
            path = Path(value)
            if not path.is_absolute():
                path = source_file.parent / path
        path = path.resolve()
        if not path.is_file():
            raise ValueError(f"资源文件不存在：{path}（来自 {source_file}）")
        item.text = str(path)


def _numbers(text: str | None, count: int | None = None) -> list[float]:
    try:
        values = [float(value) for value in (text or "").split()]
    except ValueError as error:
        raise ValueError("网格包含无效数值") from error
    if (count is not None and len(values) != count) or not all(math.isfinite(v) for v in values):
        raise ValueError("网格包含非有限数值或错误维度")
    return values


def _identity() -> list[list[float]]:
    return [[float(row == column) for column in range(4)] for row in range(4)]


def _multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _node_transform(node: ET.Element) -> list[list[float]]:
    result = _identity()
    for item in node:
        if item.tag in {"instance_geometry", "node", "extra"}:
            continue
        matrix = _identity()
        if item.tag == "matrix":
            values = _numbers(item.text, 16)
            matrix = [[values[column*4+row] for column in range(4)] for row in range(4)]
            if any(abs(matrix[3][i]-float(i == 3)) > 1e-12 for i in range(4)):
                raise ValueError("DAE不支持投影节点变换")
        elif item.tag == "translate":
            for i, value in enumerate(_numbers(item.text, 3)):
                matrix[i][3] = value
        elif item.tag == "scale":
            for i, value in enumerate(_numbers(item.text, 3)):
                matrix[i][i] = value
        elif item.tag == "rotate":
            x, y, z, degrees = _numbers(item.text, 4)
            norm = math.sqrt(x*x+y*y+z*z)
            if norm < 1e-12:
                raise ValueError("DAE旋转轴为零")
            x, y, z = x/norm, y/norm, z/norm
            c, s, t = math.cos(math.radians(degrees)), math.sin(math.radians(degrees)), 1-math.cos(math.radians(degrees))
            rotation = [[t*x*x+c, t*x*y-s*z, t*x*z+s*y],
                        [t*x*y+s*z, t*y*y+c, t*y*z-s*x],
                        [t*x*z-s*y, t*y*z+s*x, t*z*z+c]]
            for i in range(3):
                matrix[i][:3] = rotation[i]
        else:
            raise ValueError(f"DAE不支持节点元素：{item.tag}")
        result = _multiply(result, matrix)
    return result


def mesh_collision_bounds(mesh: ET.Element) -> tuple[list[float], list[float]]:
    """读取当前三角面DAE的实际实例顶点，返回collision系center与half。"""
    uri = (mesh.findtext("uri") or "").removeprefix("file://")
    path = Path(uri)
    if not path.is_absolute() or not path.is_file() or path.suffix.lower() != ".dae":
        raise ValueError(f"网格碰撞必须是存在的绝对路径DAE：{uri}")
    if mesh.find("submesh") is not None:
        raise ValueError("场景网格碰撞不支持DAE submesh选择或重心重置")
    scale = _numbers(mesh.findtext("scale", "1 1 1"), 3)
    if any(value == 0 for value in scale):
        raise ValueError("网格碰撞scale不得为零")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        raise ValueError(f"无法解析碰撞DAE：{path}") from error
    for item in root.iter():
        item.tag = item.tag.rsplit("}", 1)[-1]
    unit = root.find("asset/unit")
    meter = _numbers(unit.get("meter", "1") if unit is not None else "1", 1)[0]
    if meter <= 0:
        raise ValueError("DAE单位必须为正数")
    up = root.findtext("asset/up_axis", "Y_UP").strip()
    if up != "Z_UP":
        raise ValueError(f"当前碰撞DAE只支持Z_UP，不支持{up}")
    axes = {"Z_UP": ((1, 0, 0), (0, 1, 0), (0, 0, 1))}
    geometries = {}
    for geometry in root.findall("library_geometries/geometry"):
        data = geometry.find("mesh")
        if data is None:
            raise ValueError("DAE碰撞仅支持三角面mesh")
        sources = {}
        for source in data.findall("source"):
            array = source.find("float_array")
            accessor = source.find("technique_common/accessor")
            if array is None or accessor is None:
                continue
            values = _numbers(array.text)
            stride = int(accessor.get("stride", "1"))
            offset = int(accessor.get("offset", "0"))
            count = int(accessor.get("count", "0"))
            if stride < 3 or offset < 0 or count <= 0 or offset+(count-1)*stride+3 > len(values):
                # NORMAL等非POSITION数据不是碰撞几何；仅在被引用时拒绝。
                continue
            sources[source.get("id")] = [values[offset+i*stride:offset+i*stride+3] for i in range(count)]
        vertices = {item.get("id"): item.find("input[@semantic='POSITION']").get("source", "").lstrip("#")
                    for item in data.findall("vertices") if item.find("input[@semantic='POSITION']") is not None}
        points = []
        for primitive in data:
            if primitive.tag in {"source", "vertices", "extra"}:
                continue
            if primitive.tag != "triangles":
                raise ValueError(f"DAE碰撞仅支持triangles，不支持{primitive.tag}")
            inputs = primitive.findall("input")
            vertex_input = next((item for item in inputs if item.get("semantic") == "VERTEX"), None)
            if vertex_input is None:
                raise ValueError("DAE三角面缺少VERTEX输入")
            stride = max(int(item.get("offset", "0")) for item in inputs)+1
            vertex_offset = int(vertex_input.get("offset", "0"))
            source_id = vertices.get(vertex_input.get("source", "").lstrip("#"))
            coordinates = sources.get(source_id)
            if not coordinates:
                raise ValueError("DAE碰撞POSITION源无效")
            try:
                indices = [int(value) for value in (primitive.findtext("p") or "").split()]
            except ValueError as error:
                raise ValueError("DAE三角面索引无效") from error
            if len(indices) != int(primitive.get("count", "0"))*3*stride:
                raise ValueError("DAE三角面索引数量不一致")
            for index in indices[vertex_offset::stride]:
                if not 0 <= index < len(coordinates):
                    raise ValueError("DAE三角面顶点索引越界")
                points.append(coordinates[index])
        if not points or not geometry.get("id"):
            raise ValueError("DAE碰撞几何为空或缺少id")
        geometries[geometry.get("id")] = points
    scene_ref = root.find("scene/instance_visual_scene")
    scene_id = scene_ref.get("url", "").lstrip("#") if scene_ref is not None else ""
    scene = root.find(f"library_visual_scenes/visual_scene[@id='{scene_id}']")
    if scene is None:
        raise ValueError("DAE缺少实际visual_scene实例")
    minimum, maximum = [math.inf]*3, [-math.inf]*3
    count = 0

    def visit(node, parent):
        nonlocal count
        matrix = _multiply(parent, _node_transform(node))
        for instance in node.findall("instance_geometry"):
            identifier = instance.get("url", "").lstrip("#")
            if identifier not in geometries:
                raise ValueError("DAE实例引用不存在的几何")
            for point in geometries[identifier]:
                transformed = [sum(matrix[i][j]*point[j] for j in range(3))+matrix[i][3] for i in range(3)]
                converted = [sum(axes[up][i][j]*transformed[j] for j in range(3))*meter*scale[i] for i in range(3)]
                for i, value in enumerate(converted):
                    minimum[i], maximum[i] = min(minimum[i], value), max(maximum[i], value)
                count += 1
        if node.find("node") is not None:
            raise ValueError("当前DAE碰撞仅支持平铺实例节点，不支持嵌套node")

    for node in scene.findall("node"):
        visit(node, _identity())
    if not count or not all(math.isfinite(v) for v in minimum+maximum) or any(a >= b for a, b in zip(minimum, maximum)):
        raise ValueError("DAE碰撞包围盒为空、非有限或退化")
    return ([(a+b)/2 for a, b in zip(minimum, maximum)], [(b-a)/2 for a, b in zip(minimum, maximum)])


def add_mesh_collision_bounds(plugin: ET.Element, world: ET.Element) -> int:
    keys = set()
    count = 0
    for model in world.findall("model"):
        if not model.get("name", "").startswith(MODEL_BASENAME+"_"):
            continue
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    continue
                key = tuple(item.get("name", "") for item in (model, link, collision))
                if not all(key) or key in keys:
                    raise ValueError("mesh碰撞的model/link/collision键必须非空且唯一")
                keys.add(key)
                center, half = mesh_collision_bounds(mesh)
                entry = ET.SubElement(plugin, "mesh_collision")
                for tag, value in zip(("model", "link", "collision"), key):
                    set_text(entry, tag, value)
                for tag, values in (("center", center), ("half_extents", half)):
                    set_text(entry, tag, " ".join(format(value, ".17g") for value in values))
                count += 1
    return count


def load_model(aircraft_id: int, model_path: Path) -> ET.Element:
    root = ET.parse(model_path).getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"SDF 中没有顶层 model：{model_path}")
    if model.get("name") != MODEL_BASENAME:
        raise ValueError(
            f"需要模型 {MODEL_BASENAME}，实际为 {model.get('name')}：{model_path}"
        )
    if model.findtext("static", "false").strip().lower() in {"true", "1"}:
        raise ValueError(f"模型必须是动态模型：{model_path}")

    expected_prefix = f"/uav_{aircraft_id}/pod"
    actual_prefixes = {
        item.text.rsplit("/", 1)[0]
        for item in model.findall(".//plugin/command_topic")
        if item.text and item.text.endswith("/command")
    }
    if actual_prefixes != {expected_prefix}:
        raise ValueError(
            f"飞机 {aircraft_id} 的吊仓话题应为 {expected_prefix}，实际为 {sorted(actual_prefixes)}"
        )

    # 靶机交会使用各物理碰撞体的外接盒；不支持的形状应在启动前报错。
    supported_shapes = {"box", "cylinder", "sphere", "capsule", "ellipsoid", "mesh"}
    for collision in model.findall(".//collision"):
        geometry = collision.find("geometry")
        shapes = list(geometry) if geometry is not None else []
        if len(shapes) != 1 or shapes[0].tag not in supported_shapes:
            raise ValueError(
                f"靶机碰撞检测不支持碰撞体形状：{collision.get('name')}（{model_path}）"
            )

    model = copy.deepcopy(model)
    absolute_resources(model, model_path)
    model.set("name", f"{MODEL_BASENAME}_{aircraft_id}")
    for mesh in model.findall("link/collision/geometry/mesh"):
        mesh_collision_bounds(mesh)
    return model


def add_contact_sensors(model: ET.Element) -> int:
    """为每个物理碰撞体增加 Gazebo 原生接触传感器。"""
    count = 0
    for link in model.findall(".//link"):
        for collision in link.findall("collision"):
            collision_name = collision.get("name")
            if not collision_name:
                continue
            sensor_name = f"scenario02_contact_{count}"
            sensor = ET.SubElement(link, "sensor", name=sensor_name, type="contact")
            contact = ET.SubElement(sensor, "contact")
            set_text(contact, "collision", collision_name)
            set_text(sensor, "always_on", "true")
            set_text(sensor, "update_rate", "250")
            count += 1
    if count == 0:
        raise ValueError(f"飞机模型缺少碰撞体：{model.get('name', '')}")
    return count


def configure_gimbal_tracking(
    model: ET.Element,
    tracking_backend: str,
    target_names: list[str],
) -> None:
    """配置单目标跟踪后端与场景内可锁定的飞机实体。"""
    plugins = model.findall("./plugin[@name='two_axis::GimbalController']")
    if len(plugins) != 1:
        raise ValueError(
            f"飞机模型必须且只能包含一个 two_axis::GimbalController："
            f"{model.get('name', '')}"
        )
    plugin = plugins[0]
    for tag in ("tracking_backend", "sim_truth_target_name"):
        for old in plugin.findall(tag):
            plugin.remove(old)
    set_text(plugin, "tracking_backend", tracking_backend)
    for target_name in target_names:
        candidate = ET.SubElement(plugin, "sim_truth_target_name")
        candidate.text = target_name


def add_collision_systems(world: ET.Element) -> ET.Element:
    """加入接触数据系统和场景碰撞监控器。"""
    for old in world.findall('./plugin[@name="gz::sim::systems::Contact"]'):
        world.remove(old)
    world.insert(
        0,
        ET.Element(
            "plugin",
            filename="gz-sim-contact-system",
            name="gz::sim::systems::Contact",
        ),
    )

    for old in world.findall('./plugin[@name="scenario02::CollisionMonitor"]'):
        world.remove(old)
    plugin = ET.SubElement(
        world,
        "plugin",
        filename="libCollisionMonitor.so",
        name="scenario02::CollisionMonitor",
    )
    set_text(plugin, "event_topic", COLLISION_EVENT_TOPIC)
    set_text(plugin, "aircraft_prefix", f"{MODEL_BASENAME}_")
    set_text(plugin, "airborne_height_m", "1.0")
    add_mesh_collision_bounds(plugin, world)
    return plugin


def add_target_control(world: ET.Element, aircraft_count: int) -> ET.Element:
    """加入右侧安全区控制面板，并接管地图中心与相机跟随。"""
    gui = world.find("gui")
    if gui is None:
        gui = ET.SubElement(world, "gui", fullscreen="false")

    for old in gui.findall('./plugin[@filename="TargetControl"]'):
        gui.remove(old)

    center_pose = "0 0 200 0 1.5707963267948966 1.5707963267948966"
    for old in gui.findall('./plugin[@filename="MapCenter"]'):
        configured_pose = old.findtext("center_pose", "").strip()
        if configured_pose:
            values = configured_pose.split()
            try:
                valid = len(values) == 6 and all(
                    math.isfinite(float(value)) for value in values
                )
            except ValueError:
                valid = False
            if not valid:
                raise ValueError(f"MapCenter center_pose 不是有效六自由度位姿：{configured_pose}")
            center_pose = configured_pose
        gui.remove(old)

    elevation_text = world.findtext("spherical_coordinates/elevation", "0")
    try:
        default_altitude_m = float(elevation_text) + 100.0
    except ValueError as error:
        raise ValueError(f"world 中心海拔不是有效数值：{elevation_text}") from error
    if not math.isfinite(default_altitude_m):
        raise ValueError("world 中心海拔必须为有限数值")

    plugin = ET.SubElement(gui, "plugin", filename="TargetControl", name="场景控制")
    gz_gui = ET.SubElement(plugin, "gz-gui")
    set_text(gz_gui, "title", "场景控制")
    for value_type, key, value in (
        ("string", "state", "floating"),
        # 使用插件自己的固定标题栏，避免 Gazebo GzCard 标题栏的拖动行为。
        ("bool", "showTitleBar", "false"),
        ("bool", "showDockButton", "false"),
        ("bool", "showCloseButton", "false"),
        ("bool", "showCollapseButton", "false"),
        ("bool", "resizable", "false"),
        ("double", "width", "350"),
        ("double", "height", "700"),
        ("double", "z", "2"),
    ):
        property_element = ET.SubElement(
            gz_gui, "property", type=value_type, key=key
        )
        property_element.text = value
    anchors = ET.SubElement(gz_gui, "anchors", target="3D View")
    ET.SubElement(anchors, "line", own="right", target="right")
    ET.SubElement(anchors, "line", own="top", target="top")
    set_text(plugin, "command_topic", TARGET_COMMAND_TOPIC)
    set_text(plugin, "status_topic", TARGET_STATUS_TOPIC)
    set_text(plugin, "radar_command_topic", RADAR_COMMAND_TOPIC)
    set_text(plugin, "radar_status_topic", RADAR_STATUS_TOPIC)
    set_text(plugin, "follow_service", "/gui/follow")
    set_text(plugin, "track_topic", "/gui/track")
    set_text(plugin, "move_to_pose_service", "/gui/move_to/pose")
    set_text(plugin, "center_pose", center_pose)
    set_text(plugin, "aircraft_count", aircraft_count)
    set_text(plugin, "default_altitude_m", f"{default_altitude_m:.6f}")
    return plugin


def add_radar_model(world: ET.Element) -> ET.Element:
    """在 world 中心增加固定的 3 米雷达天线；不添加碰撞体。"""
    old = world.find("./model[@name='scenario02_radar']")
    if old is not None:
        world.remove(old)
    model = ET.SubElement(world, "model", name="scenario02_radar")
    set_text(model, "static", "true")
    set_text(model, "pose", "0 0 0 0 0 0")
    link = ET.SubElement(model, "link", name="radar_link")

    mast = ET.SubElement(link, "visual", name="mast")
    set_text(mast, "pose", "0 0 1.5 0 0 0")
    geometry = ET.SubElement(mast, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    set_text(cylinder, "radius", "0.08")
    set_text(cylinder, "length", "3.0")

    antenna = ET.SubElement(link, "visual", name="antenna")
    set_text(antenna, "pose", "0 0 3.0 0 0 0")
    geometry = ET.SubElement(antenna, "geometry")
    sphere = ET.SubElement(geometry, "sphere")
    set_text(sphere, "radius", "0.32")

    for visual, color in ((mast, "0.25 0.25 0.25 1"), (antenna, "0.1 0.65 0.85 1")):
        material = ET.SubElement(visual, "material")
        set_text(material, "ambient", color)
        set_text(material, "diffuse", color)
    return model


def add_target_trajectory(world: ET.Element) -> ET.Element:
    """加入按仿真时间推进靶机视觉模型的 world 系统插件。"""
    for old in world.findall('./plugin[@name="scenario02::TargetTrajectory"]'):
        world.remove(old)
    plugin = ET.SubElement(
        world,
        "plugin",
        filename="libTargetTrajectory.so",
        name="scenario02::TargetTrajectory",
    )
    set_text(plugin, "command_topic", TRAJECTORY_COMMAND_TOPIC)
    set_text(plugin, "status_topic", TRAJECTORY_STATUS_TOPIC)
    return plugin


def add_aircraft_marker(world: ET.Element, aircraft_id: int, x: float, y: float, altitude: float) -> None:
    """无碰撞的橙黄色定位箭头，指向对应飞机顶部吊舱附近。"""
    model = ET.SubElement(world, "model", name=f"POD_LOCATION_MARKER_{aircraft_id}")
    set_text(model, "static", "true")
    set_text(model, "pose", f"{x:g} {y:g} {altitude:.12g} 0 0 0")
    link = ET.SubElement(model, "link", name="pointer")
    for name, position, size in (
        ("mast", "-0.75 -0.2 0.06 0 0 0", "0.05 0.05 0.60"),
        ("shaft", "-0.5 0 0.35 0 0 0", "0.55 0.035 0.035"),
        ("head_upper", "-0.25 0.05 0.35 0 0 -0.7853981634", "0.16 0.035 0.035"),
        ("head_lower", "-0.25 -0.05 0.35 0 0 0.7853981634", "0.16 0.035 0.035"),
    ):
        visual = ET.SubElement(link, "visual", name=name)
        set_text(visual, "pose", position)
        box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
        set_text(box, "size", size)
        material = ET.SubElement(visual, "material")
        for tag in ("ambient", "diffuse"):
            set_text(material, tag, "1 0.5 0.015 1")
        set_text(visual, "cast_shadows", "false")


def terrain_surfaces(world: ET.Element, points: list[tuple[float, float]]) -> list[tuple[float, tuple[float, float, float]]]:
    """返回当前静态tile_*真实三角面的高度与向上单位法线。"""
    tiles = [model for model in world.findall("model") if model.get("name", "").startswith("tile_")]
    if not tiles:
        if world.findall(".//collision"):
            raise ValueError("world含物理碰撞体但缺少支持的tile_*地形，不能假定地面为0")
        return [(0.0, (0.0, 0.0, 1.0))] * len(points)  # 无地形fixture以原点水平面为参考。
    heights: list[float | None] = [None] * len(points)
    normals: list[tuple[float, float, float] | None] = [None] * len(points)
    for model in tiles:
        if model.findtext("static", "false").strip().lower() not in {"true", "1"}:
            raise ValueError("出生点地形必须为静态tile模型")
        link = model.find("link[@name='terrain']")
        collision = link.find("collision[@name='ground']") if link is not None else None
        mesh = collision.find("geometry/mesh") if collision is not None else None
        if mesh is None:
            raise ValueError(f"地形缺少terrain::ground mesh：{model.get('name')}")
        translation = [0.0, 0.0, 0.0]
        for frame in (model, link, collision):
            pose = frame.find("pose")
            if pose is not None and pose.get("relative_to"):
                raise ValueError("当前地形不支持relative_to坐标系")
            values = _numbers(frame.findtext("pose", "0 0 0 0 0 0"), 6)
            if any(value != 0 for value in values[3:]):
                raise ValueError("当前地形只支持平移的tile/link/collision姿态")
            translation = [a+b for a, b in zip(translation, values[:3])]
        path = Path((mesh.findtext("uri") or "").removeprefix("file://"))
        if not path.is_absolute() or not path.is_file() or path.suffix.lower() != ".dae":
            raise ValueError(f"地形必须是存在的绝对路径DAE：{path}")
        if mesh.find("submesh") is not None or _numbers(mesh.findtext("scale", "1 1 1"), 3) != [1, 1, 1]:
            raise ValueError("当前地形不支持submesh或非单位mesh scale")
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as error:
            raise ValueError(f"无法解析地形DAE：{path}") from error
        for item in root.iter():
            item.tag = item.tag.rsplit("}", 1)[-1]
        unit = root.find("asset/unit")
        if root.findtext("asset/up_axis") != "Z_UP" or unit is None or _numbers(unit.get("meter"), 1) != [1]:
            raise ValueError("当前地形仅支持米制Z_UP DAE")
        geometries = root.findall("library_geometries/geometry")
        nodes = root.findall("library_visual_scenes/visual_scene/node")
        scene_ref = root.find("scene/instance_visual_scene")
        scene = root.find("library_visual_scenes/visual_scene")
        if (len(geometries) != 1 or len(nodes) != 1 or scene_ref is None or scene is None
                or scene_ref.get("url") != "#"+scene.get("id", "")
                or any(item.tag != "instance_geometry" for item in nodes[0])
                or len(nodes[0]) != 1 or nodes[0][0].get("url") != "#"+geometries[0].get("id", "")):
            raise ValueError("当前地形仅支持单geometry、单平铺identity实例的DAE")
        data = geometries[0].find("mesh")
        triangles = data.findall("triangles") if data is not None else []
        if data is None or not triangles or any(item.tag not in {"source", "vertices", "triangles"} for item in data):
            raise ValueError("当前地形仅支持DAE triangles")
        for primitive in triangles:
            inputs = primitive.findall("input")
            vertex = next((item for item in inputs if item.get("semantic") == "VERTEX"), None)
            if vertex is None:
                raise ValueError("地形三角面缺少VERTEX")
            vertices = data.find(f"vertices[@id='{vertex.get('source', '').lstrip('#')}']")
            position = vertices.find("input[@semantic='POSITION']") if vertices is not None else None
            source = data.find(f"source[@id='{position.get('source', '').lstrip('#')}']") if position is not None else None
            array = source.find("float_array") if source is not None else None
            accessor = source.find("technique_common/accessor") if source is not None else None
            if array is None or accessor is None or accessor.get("source") != "#"+array.get("id", ""):
                raise ValueError("地形POSITION源无效")
            values = _numbers(array.text)
            count, stride, offset = (int(accessor.get(k, default)) for k, default in (("count", "0"), ("stride", "1"), ("offset", "0")))
            if count <= 0 or stride != 3 or offset != 0 or len(values) != count*3 or int(array.get("count", "0")) != len(values):
                raise ValueError("地形POSITION数量或stride无效")
            coordinates = [tuple(values[i+j]+translation[j] for j in range(3)) for i in range(0, len(values), 3)]
            input_stride = max(int(item.get("offset", "0")) for item in inputs)+1
            vertex_offset = int(vertex.get("offset", "0"))
            indices = [int(value) for value in (primitive.findtext("p") or "").split()]
            if (input_stride <= 0 or vertex_offset < 0 or int(primitive.get("count", "0")) <= 0
                    or len(indices) != int(primitive.get("count", "0"))*3*input_stride):
                raise ValueError("地形三角面索引数量无效")
            indices = indices[vertex_offset::input_stride]
            if any(index < 0 or index >= len(coordinates) for index in indices):
                raise ValueError("地形三角面顶点索引越界")
            for i in range(0, len(indices), 3):
                a, b, c = (coordinates[index] for index in indices[i:i+3])
                denominator = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
                if abs(denominator) < 1e-12:
                    raise ValueError("地形三角面在XY平面退化")
                ab, ac = ([b[k]-a[k] for k in range(3)], [c[k]-a[k] for k in range(3)])
                normal = (ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], denominator)
                length = math.sqrt(sum(value*value for value in normal))
                normal = tuple(value/length*(1 if denominator > 0 else -1) for value in normal)
                for j, (x, y) in enumerate(points):
                    u = ((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/denominator
                    v = ((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/denominator
                    if min(u, v, 1-u-v) >= -1e-10:
                        height = u*a[2]+v*b[2]+(1-u-v)*c[2]
                        # 共边处浮点高度等价时保留第一实际三角面，避免法线随舍入改变。
                        if heights[j] is None or height > heights[j] + 1e-9:
                            heights[j], normals[j] = height, normal
    if any(height is None for height in heights):
        raise ValueError("出生格点不在已验证地形三角面覆盖范围内")
    return [(float(height), normal) for height, normal in zip(heights, normals)]


def terrain_heights(world: ET.Element, points: list[tuple[float, float]]) -> list[float]:
    return [height for height, _ in terrain_surfaces(world, points)]


def ground_roll_pitch(normal: tuple[float, float, float]) -> tuple[float, float]:
    """yaw=0时让机体+Z沿地面法线；R=Ry(pitch)Rx(roll)。"""
    angles = (math.asin(-normal[1]), math.atan2(normal[0], normal[2]))
    return tuple(0.0 if abs(value) < 1e-15 else value for value in angles)


def spawn_altitude(model_path: Path, override: float | None) -> float:
    if override is not None:
        value = override
    else:
        path = model_path.parent / "parameters.json"
        try:
            parameters = json.loads(path.read_text())
            pose = parameters["simulation"]["spawn_pose"]
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise ValueError(f"无法继承simulation.spawn_pose：{path}；可显式指定--altitude") from error
        if not isinstance(pose, str):
            raise ValueError(f"simulation.spawn_pose应为六自由度字符串：{path}")
        value = _numbers(pose, 6)[2]
    if not math.isfinite(value) or value <= 0:
        raise ValueError("飞机原点相对地形高度必须为有限正数")
    return value


def prepare(
    source_world: Path,
    output: Path,
    models: list[tuple[int, Path]],
    spacing: float = 5.0,
    altitude: float | None = None,
    target_control: bool = False,
    tracking_backend: str = "sim_truth",
) -> Path:
    if not models:
        raise ValueError("至少需要一架飞机")
    if len({aircraft_id for aircraft_id, _ in models}) != len(models):
        raise ValueError("飞机 ID 不能重复")
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("飞机间距必须为有限正数")
    if altitude is not None and (not math.isfinite(altitude) or altitude <= 0):
        raise ValueError("飞机原点相对地形高度必须为有限正数")
    if tracking_backend not in TRACKING_BACKENDS:
        raise ValueError("单目标跟踪后端只支持 sim_truth 或 csrt")

    source_world = source_world.resolve()
    tree = ET.parse(source_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError(f"SDF 中没有 world：{source_world}")
    world.set("name", WORLD_NAME)
    absolute_resources(world, source_world)

    columns = math.ceil(math.sqrt(len(models)))
    positions = [((index % columns + 1)*spacing + .25, (index // columns + 1)*spacing) for index in range(len(models))]
    ground_surfaces = terrain_surfaces(world, positions)
    target_names = [
        *(f"{MODEL_BASENAME}_{aircraft_id}" for aircraft_id, _ in models),
        *SIM_TRUTH_DYNAMIC_TARGETS,
    ]
    for (aircraft_id, model_path), (x, y), (ground_z, normal) in zip(models, positions, ground_surfaces):
        model = load_model(aircraft_id, model_path.resolve())
        configure_gimbal_tracking(model, tracking_backend, target_names)
        add_contact_sensors(model)
        spawn_z = ground_z + spawn_altitude(model_path.resolve(), altitude)
        roll, pitch = ground_roll_pitch(normal)
        set_text(model, "pose", f"{x:g} {y:g} {spawn_z:.12g} {roll:.12g} {pitch:.12g} 0")
        world.append(model)
        add_aircraft_marker(world, aircraft_id, x, y, spawn_z)

    add_collision_systems(world)
    add_radar_model(world)

    scene = world.find('./gui/plugin[@filename="MinimalScene"]')
    if scene is not None:
        clip = scene.find("camera_clip")
        if clip is None:
            clip = ET.SubElement(scene, "camera_clip")
        set_text(clip, "near", 0.005)

    if target_control:
        add_target_control(world, len(models))
        add_target_trajectory(world)

    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def parse_model(value: list[str]) -> tuple[int, Path]:
    aircraft_id = int(value[0])
    if aircraft_id < 1:
        raise ValueError("飞机 ID 必须从 1 开始")
    return aircraft_id, Path(value[1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--model",
        action="append",
        nargs=2,
        metavar=("AIRCRAFT_ID", "MODEL_SDF"),
        required=True,
    )
    parser.add_argument("--spacing", type=float, default=5.0)
    parser.add_argument("--altitude", type=float, help="显式覆盖飞机原点距当地地形的竖直高度（m）；默认继承各模型parameters.json simulation.spawn_pose的Z")
    parser.add_argument(
        "--tracking-backend",
        choices=sorted(TRACKING_BACKENDS),
        default="sim_truth",
        help="单目标跟踪后端（默认：sim_truth）",
    )
    parser.add_argument(
        "--target-control",
        action="store_true",
        help="加入 Gazebo 靶机控制面板和场景航迹插件",
    )
    args = parser.parse_args()
    try:
        models = [parse_model(value) for value in args.model]
        print(
            prepare(
                args.source_world,
                args.output,
                models,
                args.spacing,
                args.altitude,
                args.target_control,
                args.tracking_backend,
            )
        )
    except (OSError, ValueError, ET.ParseError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
