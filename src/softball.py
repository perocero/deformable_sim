import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,-2],planeOrn)
kukaId = p.loadURDF("")
ballId = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition = [0,0,-1], scale = 0.5, mass = 40, useNeoHookean = 1, NeoHookeanMu = 1600, NeoHookeanLambda = 2000, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)
#400 600 0.01
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")

while p.isConnected():

  p.stepSimulation()  
  #there can be some artifacts in the visualizer window, 
  #due to reading of deformable vertices in the renderer,
  #while the simulators updates the same vertices
  #it can be avoided using
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #but then things go slower
  p.setGravity(0,0,-10)
  #sleep(1./240.)
  
#p.resetSimulation()
#p.stopStateLogging(logId)
