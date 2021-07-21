import pybullet as p
import struct
import numpy as np
import gym

def readLogFile(filename, verbose=True):
  f = open(filename, 'rb')

  print('Opened'),
  print(filename)

  keys = f.readline().decode('utf8').rstrip('\n').split(',')
  fmt = f.readline().decode('utf8').rstrip('\n')

  # The byte number of one record
  sz = struct.calcsize(fmt)
  # The type number of one record
  ncols = len(fmt)

  if verbose:
    print('Keys:'),
    print(keys)
    print('Format:'),
    print(fmt)
    print('Size:'),
    print(sz)
    print('Columns:'),
    print(ncols)

  # Read data
  wholeFile = f.read()
  # split by alignment word
  chunks = wholeFile.split(b'\xaa\xbb')
  log = list()
  for chunk in chunks:
    if len(chunk) == sz:
      values = struct.unpack(fmt, chunk)
      record = list()
      for i in range(ncols):
        record.append(values[i])
      log.append(record)

  return log


#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
#p.loadURDF("tray/tray.urdf", [0, 0, 0])
#p.loadURDF("block.urdf", [0, 0, 2])
ballId = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition = [0,0,2], scale = 0.1, mass = 4, useNeoHookean = 1, NeoHookeanMu = 1600, NeoHookeanLambda = 2000, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)
p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)

log = readLogFile("block_grasp_log.bin")

recordNum = len(log)
itemNum = len(log[0])
objectNum = p.getNumBodies()

print('record num:'),
print(recordNum)
print('item num:'),
print(itemNum)

def getExtendedObservation():
    #camEyePos = [0.03,0.236,0.54]
    camEyePos = [0.1,0,0]
    distance = 0.56
    pitch= 0
    yaw = 0
    roll= 0
    upAxisIndex = 2
    camInfo = p.getDebugVisualizerCamera()
    #print("width,height")
    #print(camInfo[0])
    #print(camInfo[1])
    #print("viewMatrix")
    #print(camInfo[2])
    #print("projectionMatrix")
    #print(camInfo[3])
    #viewMat = camInfo[2]
    viewMat = p.computeViewMatrixFromYawPitchRoll(camEyePos,distance,yaw, pitch,roll,upAxisIndex)
    #viewMat = [
        #-0.5120397806167603, 0.7171027660369873, -0.47284144163131714, 0.0, -0.8589617609977722,
        #-0.42747554183006287, 0.28186774253845215, 0.0, 0.0, 0.5504802465438843,
       # 0.8348482847213745, 0.0, 0.1925382763147354, -0.24935829639434814, -0.4401884973049164, 1.0
    #]
    #projMatrix = camInfo[3]#[0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0]
    projMatrix = [
        0.75, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0,
        -0.02000020071864128, 0.0
    ]

    img_arr = p.getCameraImage(width=341,
                               height=256,
                               viewMatrix=viewMat,
                               projectionMatrix=projMatrix)
    rgb = img_arr[2]
    np_img_arr = np.reshape(rgb, (256, 341, 4))
    _observation = np_img_arr
    return _observation

def Step(stepIndex):
  for objectId in range(objectNum):
    record = log[stepIndex * objectNum + objectId]
    Id = record[2]
    pos = [record[3], record[4], record[5]]
    orn = [record[6], record[7], record[8], record[9]]
    p.resetBasePositionAndOrientation(Id, pos, orn)
    numJoints = p.getNumJoints(Id)
    for i in range(numJoints):
      jointInfo = p.getJointInfo(Id, i)
      qIndex = jointInfo[3]
      if qIndex > -1:
        p.resetJointState(Id, i, record[qIndex - 7 + 17])


stepIndexId = p.addUserDebugParameter("stepIndex", 0, recordNum / objectNum - 1, 0)

while True:
  stepIndex = int(p.readUserDebugParameter(stepIndexId))
  getExtendedObservation()
  Step(stepIndex)
  p.stepSimulation()
  Step(stepIndex)
