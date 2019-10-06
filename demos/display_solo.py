""" Basic loading and visualization for the Solo robot using gepetto viewer. """

import time
from robot_properties_solo.config import SoloConfig
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.frame import Frame


if __name__ == "__main__":

    gepetto_gui_scene = GepettoGuiScene("solo_scene", "solo_window")
    config = SoloConfig()
    solo_visual = RobotVisual(gepetto_gui_scene, "solo", config.urdf_path,
                              config.meshes_path)
    solo_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gepetto_gui_scene)

    # Example of moving the robot forward and updating the display every time.
    q = config.q0.copy()
    for i in range(10):
        q[0] += 0.05
        solo_visual.display(q)
        time.sleep(0.2)
