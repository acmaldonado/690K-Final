import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("/home/andy/Desktop/pybullet-test/franka_panda_description-master/robots/panda_arm.urdf",startPos, startOrientation, useFixedBase=1)
visBallID = p.createCollisionShape(p.GEOM_SPHERE)
collBallID = p.createVisualShape(p.GEOM_SPHERE)
p.createMultiBody(baseMass=1, baseInertialFramePosition=[0,0,0], baseCollisionShapeIndex=collBallID, baseVisualShapeIndex=visBallID, basePosition=[0,0,0])
maxForce = 300
mode = p.TORQUE_CONTROL
joints = [i for i in range(p.getNumJoints(boxId))]
command = np.zeros(p.getNumJoints(boxId))
command[0] = maxForce
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    if i%100 == 0:
        command[0] = -command[0]
        print(p.getJointState(boxId, 0))
        print('switch!')
    if i == 500:
        command[4] = maxForce
        print('begin!')
    p.setJointMotorControlArray(boxId, joints, controlMode=mode, forces=command)
    
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()