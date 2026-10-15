#!/usr/bin/env python3
"""从资产仓地理 world 生成包含多架火箭机两轴吊仓的运行 world。"""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_NAME = "geo_32_217857_119_105878"
MODEL_BASENAME = "rocket_two_axis_pod"


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

    model = copy.deepcopy(model)
    absolute_resources(model, model_path)
    model.set("name", f"{MODEL_BASENAME}_{aircraft_id}")
    return model


def prepare(
    source_world: Path,
    output: Path,
    models: list[tuple[int, Path]],
    spacing: float = 5.0,
    altitude: float = 0.24,
) -> Path:
    if not models:
        raise ValueError("至少需要一架飞机")
    if len({aircraft_id for aircraft_id, _ in models}) != len(models):
        raise ValueError("飞机 ID 不能重复")
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError("飞机间距必须为有限正数")
    if not math.isfinite(altitude) or altitude < 0.225:
        raise ValueError("飞机原点高度必须至少为 0.225 m")

    source_world = source_world.resolve()
    tree = ET.parse(source_world)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError(f"SDF 中没有 world：{source_world}")
    world.set("name", WORLD_NAME)
    absolute_resources(world, source_world)

    columns = math.ceil(math.sqrt(len(models)))
    for index, (aircraft_id, model_path) in enumerate(models):
        model = load_model(aircraft_id, model_path.resolve())
        x = (index % columns) * spacing
        y = (index // columns) * spacing
        set_text(model, "pose", f"{x:g} {y:g} {altitude:g} 0 0 0")
        world.append(model)

    scene = world.find('./gui/plugin[@filename="MinimalScene"]')
    if scene is not None:
        clip = scene.find("camera_clip")
        if clip is None:
            clip = ET.SubElement(scene, "camera_clip")
        set_text(clip, "near", 0.005)

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
    parser.add_argument("--altitude", type=float, default=0.24)
    args = parser.parse_args()
    try:
        models = [parse_model(value) for value in args.model]
        print(prepare(args.source_world, args.output, models, args.spacing, args.altitude))
    except (OSError, ValueError, ET.ParseError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
