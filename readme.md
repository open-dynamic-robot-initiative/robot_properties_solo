Robot Properties Solo
---------------------

### What it is

URDF and ROS integration of the Solo BLMC robot.

### Installation

1. Install Pinocchio if you have not done so already.

   The simplest way to do so is using Conda:

   ```
   conda install -c conda-forge pinocchio
   ```

   Alternatively you can use any of the ways listed here: https://stack-of-tasks.github.io/pinocchio/download.html.

2. Install bullet_utils:

  ```
  git clone git@github.com:machines-in-motion/bullet_utils.git
  cd bullet_utils
  pip3 install .
  ```

3. Install robot_properties_solo:

  ```
  git clone git@github.com:open-dynamic-robot-initiative/robot_properties_solo.git
  cd robot_properties_solo
  pip3 install .
  ```

### Examples

**Loading Solo12 in PyBullet**

```
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot

env = BulletEnvWithGround(p.GUI)
robot = env.add_robot(Solo12Robot)
```

**Run simulation on MPI cluster**

```
conda create -n solo python=3.7
source activate solo
conda install -c conda-forge pinocchio 

git clone git@github.com:machines-in-motion/bullet_utils.git
cd bullet_utils
pip3 install .

git clone git@github.com:open-dynamic-robot-initiative/robot_properties_solo.git
cd robot_properties_solo
pip3 install .
```

### Authors

- Felix Grimmiger
- Maximilien Naveau
- Avadesh Meduri
- Julian Viereck
- Huaijiang Zhu

### Copyrights

Copyright(c) 2018-2021 Max Planck Gesellschaft, New York University

### License

BSD 3-Clause License


