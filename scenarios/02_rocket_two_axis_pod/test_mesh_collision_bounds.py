"""DAE实例、单位与外部缩放的真实局部包围盒，以及mesh启动拒绝。"""
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import prepare_world as prepare


def write_box_dae(path, *, unit=1., up="Z_UP", transform="", lower=(1, 2, 3), upper=(3, 6, 9)):
    points = [(x, y, z) for x in (lower[0], upper[0]) for y in (lower[1], upper[1]) for z in (lower[2], upper[2])]
    indices = [0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1,
               2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3]
    path.write_text(f'''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
      <asset><unit meter="{unit}"/><up_axis>{up}</up_axis></asset>
      <library_geometries><geometry id="box"><mesh><source id="positions">
      <float_array id="positions-array" count="24">{' '.join(str(v) for p in points for v in p)}</float_array>
      <technique_common><accessor source="#positions-array" count="8" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common>
      </source><vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
      <triangles count="12"><input semantic="VERTEX" source="#vertices" offset="0"/><p>{' '.join(map(str,indices))}</p></triangles>
      </mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="Scene">
      <node id="outer">{transform}<instance_geometry url="#box"/></node>
      </visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>''')
    return path


def mesh_element(path, scale="1 1 1"):
    mesh = ET.Element("mesh")
    ET.SubElement(mesh, "uri").text = str(path)
    ET.SubElement(mesh, "scale").text = scale
    return mesh


class MeshCollisionBoundsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "offset.dae"

    def assertVector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=11)

    def test_nonzero_mesh_center_and_size_are_separate(self):
        write_box_dae(self.path)
        center, half = prepare.mesh_collision_bounds(mesh_element(self.path))
        self.assertVector(center, (2, 4, 6))
        self.assertVector(half, (1, 2, 3))

    def test_node_transform_unit_and_nonuniform_mesh_scale(self):
        write_box_dae(self.path, unit=.001, transform="<translate>10 20 30</translate><rotate>0 0 1 90</rotate><scale>2 1 .5</scale>")
        center, half = prepare.mesh_collision_bounds(mesh_element(self.path, "2 3 4"))
        self.assertVector(center, (.012, .072, .132))
        self.assertVector(half, (.004, .006, .006))

    def test_column_major_matrix_transform_and_reflected_scale(self):
        write_box_dae(self.path, transform="<matrix>1 0 0 0 0 1 0 0 0 0 1 0 10 20 30 1</matrix>")
        center, half = prepare.mesh_collision_bounds(mesh_element(self.path, "-2 3 4"))
        self.assertVector(center, (-24, 72, 144))
        self.assertVector(half, (2, 6, 12))

    def test_other_up_axes_are_rejected_before_gazebo_loading(self):
        for up in ("Y_UP", "X_UP", "INVALID"):
            write_box_dae(self.path, up=up)
            with self.subTest(up=up), self.assertRaisesRegex(ValueError, "只支持Z_UP"):
                prepare.mesh_collision_bounds(mesh_element(self.path))

    def test_bad_missing_empty_or_unsupported_mesh_fails_before_start(self):
        for content in ("", "<COLLADA/>", "not xml"):
            self.path.write_text(content)
            with self.subTest(content=content), self.assertRaises(ValueError):
                prepare.mesh_collision_bounds(mesh_element(self.path))
        self.path.unlink()
        with self.assertRaisesRegex(ValueError, "存在的绝对路径DAE"):
            prepare.mesh_collision_bounds(mesh_element(self.path))
        write_box_dae(self.path)
        for scale in ("0 1 1", "nan 1 1", "1 1"):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                prepare.mesh_collision_bounds(mesh_element(self.path, scale))
        mesh = mesh_element(self.path)
        ET.SubElement(mesh, "submesh")
        with self.assertRaisesRegex(ValueError, "submesh"):
            prepare.mesh_collision_bounds(mesh)

    def test_nested_nodes_are_rejected_before_gazebo_loading(self):
        write_box_dae(self.path)
        text = self.path.read_text().replace('<instance_geometry url="#box"/>', '<node id="inner"><instance_geometry url="#box"/></node>')
        self.path.write_text(text)
        with self.assertRaisesRegex(ValueError, "嵌套node"):
            prepare.mesh_collision_bounds(mesh_element(self.path))

    def test_unsupported_node_transform_is_not_silently_ignored(self):
        write_box_dae(self.path, transform="<lookat>0 0 0 1 0 0 0 0 1</lookat>")
        with self.assertRaisesRegex(ValueError, "节点元素"):
            prepare.mesh_collision_bounds(mesh_element(self.path))

    def test_mesh_bounds_key_is_unique_per_actual_model_link_collision(self):
        write_box_dae(self.path)
        world = ET.Element("world")
        for _ in range(2):
            model = ET.SubElement(world, "model", name="rocket_two_axis_pod_1")
            link = ET.SubElement(model, "link", name="camera_link")
            geometry = ET.SubElement(ET.SubElement(link, "collision", name="window"), "geometry")
            geometry.append(mesh_element(self.path))
        with self.assertRaisesRegex(ValueError, "唯一"):
            prepare.add_mesh_collision_bounds(ET.Element("plugin"), world)


if __name__ == "__main__":
    unittest.main()
