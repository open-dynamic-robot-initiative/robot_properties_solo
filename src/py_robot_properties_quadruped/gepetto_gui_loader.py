"""

"""

import numpy as np
from config import QuadrupedConfig
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
from py_gepetto_gui_helper.robot_visual import RobotVisual


def create_scene():
    """
    Just create a scene for the quadruped to be in
    """
    return GepettoGuiScene("quadruped_scene", "quadruped_window")


def load_quadruped_in_gepetto_gui(gepetto_scene, robot_name):
    """
    Load the quadruped meshes in the scene
    """
    config = QuadrupedConfig()
    return RobotVisual(gepetto_scene, robot_name, config.urdf_path,
                       config.meshes_path)


if __name__ == "__main__":
    gui_scene = create_scene()
    quadruped_visual = load_quadruped_in_gepetto_gui(gui_scene, "quadruped")
    config = QuadrupedConfig()
    quadruped_visual.display(config.q0)
    

