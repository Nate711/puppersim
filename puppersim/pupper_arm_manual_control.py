import pybullet as p
import time

# import pybullet_data
import data as pd

p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setAdditionalSearchPath(pd.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

pupper = p.loadURDF("pupper_arm.urdf", useFixedBase=True)


gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(pupper, -1, linearDamping=0, angularDamping=0)
p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=135, cameraPitch=-30, cameraTargetPosition=[0,0,0])

#press 'G' or use the following line to hide the GUI
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

for j in range(p.getNumJoints(pupper)):
  p.changeDynamics(pupper, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(pupper, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    jointIds.append(j)
    paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))

p.setRealTimeSimulation(1)
while (1):
  p.setGravity(0, 0, p.readUserDebugParameter(gravId))
  for i in range(len(paramIds)):
    c = paramIds[i]
    targetPos = p.readUserDebugParameter(c)
    p.setJointMotorControl2(pupper, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
  time.sleep(0.01)
