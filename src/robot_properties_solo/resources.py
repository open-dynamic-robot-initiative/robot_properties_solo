"""resources

Define the resource files present in this package.

License: BSD 3-Clause License
Copyright (C) 2018-2021, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

from pathlib import Path


class Resources(object):
    def __init__(self, robot_name, robot_family="solo") -> None:
        super().__init__()

        self.package_path = str(Path(__file__).resolve().parent.absolute())

        self.robot_name = str(robot_name)
        self.robot_family = str(robot_family)

        self.resources_dir = Path(self.package_path) / "resources"

        self.dgm_yaml_path = str(
            self.resources_dir
            / "dynamic_graph_manager"
            / ("dgm_parameters_" + self.robot_name + ".yaml")
        )

        self.urdf_path = str(
            self.resources_dir / "urdf" / (self.robot_name + ".urdf")
        )
        self.srdf_path = str(
            self.resources_dir / "srdf" / (self.robot_family + ".srdf")
        )
        self.meshes_path = str(Path(self.package_path).parent)

        self.imp_ctrl_yaml_path = str(
            self.resources_dir / "impedance_ctrl.yaml"
        )

        self.resources_dir = str(self.resources_dir)
