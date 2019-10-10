"""

"""
import eigenpy
eigenpy.switchToNumpyMatrix()
import time
from config import SoloConfig
from py_gepetto_gui_helper.gepetto_gui_scene import GepettoGuiScene
from py_gepetto_gui_helper.robot_visual import RobotVisual
from py_gepetto_gui_helper.frame import Frame

def create_scene():
    """
    Just create a scene for the solo to be in
    """
    return GepettoGuiScene("solo_scene", "solo_window")


def load_solo_in_gepetto_gui(gepetto_scene, robot_name):
    """
    Load the solo meshes in the scene
    """
    config = SoloConfig()
    return RobotVisual(gepetto_scene, robot_name, config.urdf_path,
                       config.meshes_path)

def display_solo_in_gepetto_gui(launch_gepetto_gui_exec=False):
    """
    Uses the function above to load the urdf model of Solo in gepetto gui
    and load it in the initial configuration
    """

    if launch_gepetto_gui_exec:
        # create a new window
        gepetto_gui_process = GepettoGuiScene.open_gepetto_gui()

    # create a scene in it
    gui_scene = create_scene()
    # load the robot
    solo_visual = load_solo_in_gepetto_gui(gui_scene, "solo")
    # place the robot in initial configuration
    config = SoloConfig()
    solo_visual.display(config.q0)
    # place the world frame
    world_frame = Frame(gui_scene)

    if launch_gepetto_gui_exec:
        # close the window after little while
        time.sleep(5)
        GepettoGuiScene.close_gepetto_gui(gepetto_gui_process)

    return gui_scene, solo_visual, world_frame

if __name__ == "__main__":
    display_solo_in_gepetto_gui()
    

