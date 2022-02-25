import pybullet as p
import time

IS_GUI = False
# physical params
dt = 1/240
g = 10
L = 0.8
m = 1
q0 = 0.78
maxTime = 10
t = 0
# joint index
jIdx = 1
# containters for logging and plots
idx = 0
maxIdx = int(maxTime / dt)
log_time = [x for x in range(idx,maxIdx)]
log_pos = [x for x in range(idx,maxIdx)]
log_pos[0] = q0

if (IS_GUI):
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)

p.setGravity(0, 0, -g)
bodyId = p.loadURDF("./pendulum.urdf")

p.changeDynamics(bodyUniqueId=bodyId,
                linkIndex=jIdx,
                restitution=0,
                spinningFriction=0,
                rollingFriction=0,
                lateralFriction=0,
                frictionAnchor=0,
                anisotropicFriction=0)

p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        targetPosition = q0,
                        controlMode = p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = -0.155)

while idx < maxIdx:
    p.stepSimulation()
    pos = p.getJointState(bodyId, jIdx)[0]
    t += dt
    log_pos[idx] = pos
    log_time[idx] = t
    idx += 1
    if (IS_GUI):
        time.sleep(dt)
p.disconnect()

import matplotlib.pyplot as plt
plt.plot(log_time, log_pos)
plt.grid(True)
plt.show()