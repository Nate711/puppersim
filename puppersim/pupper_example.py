# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Lint as: python3
r"""An example that the Pupper stands.

"""
from absl import app
from absl import flags
import os
import time
import gin
import math
import numpy as np

from pybullet_envs.minitaur.envs_v2 import env_loader
import pybullet as p
import puppersim

import os


flags.DEFINE_bool("render", True, "Whether to render the example.")
flags.DEFINE_bool("profile", False, "Whether to print timing results for different parts of the code.")
flags.DEFINE_bool("run_on_robot", False, "Whether to run on robot or in simulation.")

FLAGS = flags.FLAGS
CONFIG_DIR = puppersim.getPupperSimPath()
_NUM_STEPS = 100000
_ENV_RANDOM_SEED = 13

MAX_TORQUE = 1.7 # Maximum torque in [Nm]. For DJI Pupper limited to 7A, the maximum torque pushing against resistance is ~1.7Nm.

def _load_config(render=False):
  if FLAGS.run_on_robot:
    config_file = os.path.join(CONFIG_DIR, "config", "pupper_robot.gin")
  else:
    config_file = os.path.join(CONFIG_DIR, "config", "pupper.gin")

  gin.parse_config_file(config_file)
  gin.bind_parameter("SimulationParameters.enable_rendering", render)


def run_example(num_max_steps=_NUM_STEPS):
  """Runs the example.

  Args:
    num_max_steps: Maximum number of steps this example should run for.
  """
  env = env_loader.load()
  env.seed(_ENV_RANDOM_SEED)

  env._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
  env._pybullet_client.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

  # BUG: the motor limits defined by the robot override those of the motor model here
  # https://github.com/bulletphysics/bullet3/blob/48dc1c45da685c77d3642545c4851b05fb3a1e8b/examples/pybullet/gym/pybullet_envs/minitaur/robots/quadruped_base.py#L131

  env._robot._motor_model._torque_lower_limits = -np.array([MAX_TORQUE]*12)
  env._robot._motor_model._torque_upper_limits = np.array([MAX_TORQUE]*12)

  print("env.action_space=",env.action_space)
  obs = env.reset()
  last_control_step = time.time()
  for i in range(num_max_steps):
    delta_time = env.robot.GetTimeSinceReset()
    # 1Hz signal
    phase = delta_time * 2 * np.pi 
    # joint angles corresponding to a standing position
    action = np.array([0, 0.6,-1.2,0, 0.6,-1.2,0, 0.6,-1.2,0, 0.6,-1.2])
    # modulate the default joint angles by a sinusoid to make the robot do pushups
    action = (np.sin(phase) * 0.6 + 0.8) * action
    # NOTE: We do not fix the loop rate so be careful if using a policy that is solely dependent on time

    before_step_timestamp = time.time()
    obs, reward, done, _ = env.step(action)
    after_step_timestamp = time.time()
    if FLAGS.profile:
      print("loop_dt: ", time.time() - last_control_step, "env.step(): ", after_step_timestamp - before_step_timestamp)
    else:
      print("obs: ", obs)
      print("act: ", action)
      print("time: ", delta_time)

    last_control_step = time.time()


def main(_):
  _load_config(FLAGS.render)
  run_example()


if __name__ == "__main__":
  app.run(main)
