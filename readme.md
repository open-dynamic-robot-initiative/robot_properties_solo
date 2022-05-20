Robot Properties Solo
---------------------

### What it is

Collection of configuration files for the solo robot.

### Installation

1. Install Pinocchio if you have not done so already.

   The simplest way to do so is using Conda:

   ```
   conda install -c conda-forge pinocchio
   ```

   Alternatively you can use any of the ways listed here: https://stack-of-tasks.github.io/pinocchio/download.html.

2. Install bullet_utils:

  If you use ssh to clone the repository.

  ```
  git clone git@github.com:machines-in-motion/bullet_utils.git
  cd bullet_utils
  pip3 install .
  ```

  Or if you use https to clone the repository.

  ```
  git clone https://github.com/machines-in-motion/bullet_utils.git
  cd bullet_utils
  pip3 install .
  ```

3. Install robot_properties_solo:

  If you use ssh to clone the repository.

  ```
  git clone git@github.com:open-dynamic-robot-initiative/robot_properties_solo.git
  cd robot_properties_solo
  pip3 install .
  ```

  Or if you use https to clone the repository.

  ```
  git clone https://github.com/open-dynamic-robot-initiative/robot_properties_solo.git
  cd bullet_utils
  pip3 install .
  ```

4. (Optional) In order to run the demo python files in the `demos/` folder that start with `demo_display_solo*`, please install [gepetto-viewer](https://github.com/Gepetto/gepetto-viewer)
  It is easier to install it using conda.
  ```
  conda install gepetto-viewer gepetto-viewer-corba -c conda-forge
  ```


### Examples

Below are a few examples. You find more in the `demos/` folder.

**Loading Solo12 in PyBullet**

```
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot

env = BulletEnvWithGround(p.GUI)
robot = env.add_robot(Solo12Robot)
```

**Run simulation on Max-Planck Institute cluster**

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

### License and Copyrights

Copyright(c) 2018-2021 Max Planck Gesellschaft, New York University

BSD 3-Clause License
