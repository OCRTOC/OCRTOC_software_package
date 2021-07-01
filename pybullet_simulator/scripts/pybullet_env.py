import time
import numpy as np
import pybullet
import pybullet_data
import math
import cv2

class Floor:
    def __init__(self, basePosition=[0,0,0], baseRPY=[0,0,0]):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.id = pybullet.loadURDF('plane.urdf', basePosition=basePosition, baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0):
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, lateralFriction=lateralFriction, spinningFriction=spinningFriction)
        print("Floor friction updated!")
        print("lateralFriction:", pybullet.getDynamicsInfo(self.id, -1)[1])
        print("spinningFriction:", pybullet.getDynamicsInfo(self.id, -1)[7])

class VisualSphere:
    def __init__(self, basePosition=[0,0,0], baseRPY=[0,0,0], radius=0.1, rgbaColor=[1,0,0,0.6]):
        visualShapeId = pybullet.createVisualShape(shapeType=pybullet.GEOM_SPHERE, radius=radius, rgbaColor=rgbaColor)
        self.objectUniqueId = pybullet.createMultiBody(baseVisualShapeIndex=visualShapeId, basePosition=basePosition, baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def resetPosition(self, position, quaternion=[0,0,0,1]):
        pybullet.resetBasePositionAndOrientation(self.objectUniqueId, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.objectUniqueId)

class VisualBox:
    def __init__(self, basePosition=[0,0,1], baseRPY=[0,0,0], size=[1,1,1], rgbaColor=[1,0,0,0.6]):
        visualShapeId = pybullet.createVisualShape(shapeType=pybullet.GEOM_BOX, halfExtents=np.array(size)/2, rgbaColor=rgbaColor)
        self.objectUniqueId = pybullet.createMultiBody(baseVisualShapeIndex=visualShapeId, basePosition=basePosition, baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def resetBasePosition(self, position, quaternion=[0,0,0,1]):
        pybullet.resetBasePositionAndOrientation(self.objectUniqueId, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.objectUniqueId)

class Sphere:
    def __init__(self, basePosition=[0,0,1], baseRPY=[0,0,0], radius=0.1, mass=1, rgbaColor=[1,0,0,1]):
        collisionShapeIndex = pybullet.createCollisionShape(shapeType=pybullet.GEOM_SPHERE, radius=radius,)
        visualShapeIndex = pybullet.createVisualShape(shapeType=pybullet.GEOM_SPHERE, radius=radius, rgbaColor=rgbaColor)
        self.objectUniqueId = pybullet.createMultiBody(baseMass=mass,
                                           baseCollisionShapeIndex=collisionShapeIndex,
                                           baseVisualShapeIndex=visualShapeIndex,
                                           basePosition=basePosition,
                                           baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def resetBasePosition(self, position, quaternion=[0,0,0,1]):
        pybullet.resetBasePositionAndOrientation(self.objectUniqueId, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.objectUniqueId)

class Cylinder:
    def __init__(self, basePosition=[0,0,1], baseRPY=[0,0,0], radius=0.1, length=1, mass=1, rgbaColor=[0.8,0.8,0.8,1], useFixedBase=False):
        mass = 0 if useFixedBase else mass
        collisionShapeIndex = pybullet.createCollisionShape(shapeType=pybullet.GEOM_CYLINDER, radius=radius, height=length)
        visualShapeIndex = pybullet.createVisualShape(shapeType=pybullet.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=rgbaColor)
        self.objectUniqueId = pybullet.createMultiBody(baseMass=mass,
                                           baseCollisionShapeIndex=collisionShapeIndex,
                                           baseVisualShapeIndex=visualShapeIndex,
                                           basePosition=basePosition,
                                           baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))

    def resetBasePosition(self, position, quaternion=[0,0,0,1]):
        pybullet.resetBasePositionAndOrientation(self.objectUniqueId, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.objectUniqueId)

class Box:
    def __init__(self):
        print('robot id:', self.id)
        self.changeFriction(lateralFriction=1.0, spinningFriction=1.0)
        self.enableFrictionAnchor()

    @classmethod
    def fromParam(cls, basePosition=[0,0,1], baseRPY=[0,0,0], size=[1, 1, 1], mass=1, rgbaColor=[0.8,0.8,0.8,1], useFixedBase=False):
        mass = 0 if useFixedBase else mass
        collisionShapeIndex = pybullet.createCollisionShape(shapeType=pybullet.GEOM_BOX, halfExtents=np.array(size) / 2)
        visualShapeIndex = pybullet.createVisualShape(shapeType=pybullet.GEOM_BOX, halfExtents=np.array(size) / 2,
                                                      rgbaColor=rgbaColor,specularColor=[0.0, 0.0, 0.0])
        cls.id = pybullet.createMultiBody(baseMass=mass,
                                           baseCollisionShapeIndex=collisionShapeIndex,
                                           baseVisualShapeIndex=visualShapeIndex,
                                           basePosition=basePosition,
                                           baseOrientation=pybullet.getQuaternionFromEuler(baseRPY))
        return cls()


    @classmethod
    def fromID(cls, id):
        cls.id = id
        return cls()

    def getBasePosition(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[0]

    def getBaseOrientation(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[1]

    def resetBasePosition(self, position, quaternion=[0,0,0,1]):
        pybullet.resetBasePositionAndOrientation(self.id, position, quaternion)

    def remove(self):
        pybullet.removeBody(self.id)

    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0):
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, lateralFriction=lateralFriction, spinningFriction=spinningFriction)
        print("Box friction updated!")
        print("lateralFriction:", pybullet.getDynamicsInfo(self.id, -1)[1])
        print("spinningFriction:", pybullet.getDynamicsInfo(self.id, -1)[7])

    def enableFrictionAnchor(self):
        print("Box friction anchor enabled!")
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, frictionAnchor=True)

class Table:
    def __init__(self, basePosition, baseRPY, useFixedBase, globalScaling=1.0):
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        table_urdf_path = pybullet_data.getDataPath() + "/table/table.urdf"
        self.id = pybullet.loadURDF(table_urdf_path,
                                    basePosition=basePosition,
                                    baseOrientation=pybullet.getQuaternionFromEuler(baseRPY),
                                    useMaximalCoordinates=0,
                                    useFixedBase=useFixedBase,
                                    flags=0,
                                    globalScaling=globalScaling,
                                    physicsClientId=0)


    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0, rollingFriction=0.0):
        print("Current table dynamic: ", pybullet.getDynamicsInfo(self.id, -1))
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction)
        print("Updated table dynamic: ", pybullet.getDynamicsInfo(self.id, -1))

class StepStone:
    def __init__(self, basePosition, baseRPY, useFixedBase=True, globalScaling=1.0):
        urdf = '/home/xin/codes/robots_playground/pybullet_data/step_stone/step_stone.urdf'
        self.id = pybullet.loadURDF(urdf,
                                    basePosition=basePosition,
                                    baseOrientation=pybullet.getQuaternionFromEuler(baseRPY),
                                    useMaximalCoordinates=0,
                                    useFixedBase=useFixedBase,
                                    flags=0,
                                    globalScaling=globalScaling,
                                    physicsClientId=0)


    def changeFriction(self, lateralFriction=1.0, spinningFriction=1.0, rollingFriction=0.0):
        print("Current table dynamic: ", pybullet.getDynamicsInfo(self.id, -1))
        pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=-1, lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction)
        print("Updated table dynamic: ", pybullet.getDynamicsInfo(self.id, -1))

class SimEnv:
    def __init__(self, sim_rate=1000, g=9.81, real_time_sim=True, GUI=True):
        self.sim_rate = sim_rate
        self.sim_time_step = 1.0 / self.sim_rate
        self.real_time_sim = real_time_sim
        self.sim_count = 0
        self.sim_time = 0.0

        if GUI is True:
            self.physics_client = pybullet.connect(pybullet.GUI_SERVER)
        else:
            self.physics_client = pybullet.connect(pybullet.SHARED_MEMORY_SERVER)
        pybullet.setTimeStep(self.sim_time_step)
        pybullet.setGravity(0, 0, -g)

        self.floor = Floor(basePosition=[0,0,-0.0])
        self.floor.changeFriction(lateralFriction=1.0, spinningFriction=1.0)



        self.configureDebugVisualizer(COV_ENABLE_GUI=False,
                                      COV_ENABLE_RGB_BUFFER_PREVIEW=False,
                                      COV_ENABLE_DEPTH_BUFFER_PREVIEW=False,
                                      COV_ENABLE_SEGMENTATION_MARK_PREVIEW=False)


    def loadURDF(self, fileName, basePosition, baseRPY, useFixedBase, globalScaling=1.0):
        modelID = pybullet.loadURDF(fileName=fileName,
                                    basePosition=basePosition,
                                    baseOrientation=pybullet.getQuaternionFromEuler(baseRPY),
                                    useMaximalCoordinates=0,
                                    useFixedBase=useFixedBase,
                                    flags=0,
                                    globalScaling=globalScaling,
                                    physicsClientId=0)
        return modelID

    def configureDebugVisualizer(self, COV_ENABLE_GUI=False, COV_ENABLE_RGB_BUFFER_PREVIEW=False, COV_ENABLE_DEPTH_BUFFER_PREVIEW=False, COV_ENABLE_SEGMENTATION_MARK_PREVIEW=False):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, COV_ENABLE_GUI)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, COV_ENABLE_RGB_BUFFER_PREVIEW)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, COV_ENABLE_DEPTH_BUFFER_PREVIEW)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, COV_ENABLE_SEGMENTATION_MARK_PREVIEW)

    def reset(self):
        pass

    def resetLightPosition(self, lightPosition=[1,1,1]):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False, lightPosition=lightPosition)

    def resetShadowMapResolution(self, resolution=2**12):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False, shadowMapResolution=resolution)


    def resetCamera(self, cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0,0,0.5]):
        pybullet.resetDebugVisualizerCamera(cameraDistance=cameraDistance,
                                            cameraYaw=cameraYaw,
                                            cameraPitch=cameraPitch,
                                            cameraTargetPosition=cameraTargetPosition)

    def resetGUIView(self, distance=1.5, yaw=45, pitch=-30, target_position=[0,0,0.5]):
        pybullet.resetDebugVisualizerCamera(cameraDistance=distance,
                                            cameraYaw=yaw,
                                            cameraPitch=pitch,
                                            cameraTargetPosition=target_position)

    def debug(self):
        # pause simulation by space key event
        keys = pybullet.getKeyboardEvents()
        space_key = ord(' ')
        if space_key in keys and keys[space_key] & pybullet.KEY_WAS_TRIGGERED:
            print("*" * 100 + "Simulation Paused! Press 'Space' to resume!" + "*" * 100)
            while True:
                keys = pybullet.getKeyboardEvents()
                if space_key in keys and keys[space_key] & pybullet.KEY_WAS_TRIGGERED:
                    break

    @staticmethod
    def get_arrow_key_states():
        keys = pybullet.getKeyboardEvents()
        x, y, yaw = 0, 0, 0
        if pybullet.B3G_UP_ARROW in keys and keys[pybullet.B3G_UP_ARROW] & pybullet.KEY_IS_DOWN:
            x += 1
        if pybullet.B3G_DOWN_ARROW in keys and keys[pybullet.B3G_DOWN_ARROW] & pybullet.KEY_IS_DOWN:
            x -= 1

        if pybullet.B3G_SHIFT in keys and keys[pybullet.B3G_SHIFT] & pybullet.KEY_IS_DOWN:
            if pybullet.B3G_LEFT_ARROW in keys and keys[pybullet.B3G_LEFT_ARROW] & pybullet.KEY_IS_DOWN:
                yaw += 1
            if pybullet.B3G_RIGHT_ARROW in keys and keys[pybullet.B3G_RIGHT_ARROW] & pybullet.KEY_IS_DOWN:
                yaw -= 1
        else:
            if pybullet.B3G_LEFT_ARROW in keys and keys[pybullet.B3G_LEFT_ARROW] & pybullet.KEY_IS_DOWN:
                y += 1
            if pybullet.B3G_RIGHT_ARROW in keys and keys[pybullet.B3G_RIGHT_ARROW] & pybullet.KEY_IS_DOWN:
                y -= 1

        return (x, y, 0), (0, 0, yaw)


    def step(self):
        pybullet.stepSimulation()
        if self.real_time_sim:
            time.sleep(self.sim_time_step)
        self.sim_count += 1
        self.sim_time = self.sim_count * self.sim_time_step


    def spin(self):
        while True:
            self.step()
            pass

    def getCameraImage(self, height=256, weight=256, rate=1):
        pybullet.getCameraImage(256, 256, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)

    @staticmethod
    def addDebugText(text, textPosition, textColorRGB=[0,0,0], textSize=2, lifeTime=0):
        textID = pybullet.addUserDebugText(text=text, textPosition=textPosition, textColorRGB=textColorRGB, textSize=textSize, lifeTime=lifeTime)
        return textID

    @staticmethod
    def addDebugPoint(point, color=[0,0,0], lineWidth=1, lifeTime=0):
        dis, x_unit, y_unit = 0.01, np.array([1,0,0]), np.array([0,1,0])
        line_x = pybullet.addUserDebugLine(point + dis * x_unit, point - dis * x_unit, lineColorRGB=color, lineWidth=lineWidth, lifeTime=lifeTime)
        line_y = pybullet.addUserDebugLine(point + dis * y_unit, point - dis * y_unit, lineColorRGB=color, lineWidth=lineWidth, lifeTime=lifeTime)
        return (line_x, line_y)

    @staticmethod
    def addDebugLine(startPoint, endPoint, color=[0,0,0], lineWidth=1, lifeTime=0):
        return pybullet.addUserDebugLine(startPoint, endPoint, lineColorRGB=color, lineWidth=lineWidth, lifeTime=lifeTime)

    @staticmethod
    def addDebugFrame(origin, quaternion, axisLength=0.2, lineWidth=1, lifeTime=0):
        import utils.transformations as tf
        transformation_matrix = np.dot(tf.translation_matrix(origin), tf.quaternion_matrix(quaternion))
        x = np.dot(transformation_matrix, np.array([axisLength, 0, 0, 1]))[:3]
        y = np.dot(transformation_matrix, np.array([0, axisLength, 0, 1]))[:3]
        z = np.dot(transformation_matrix, np.array([0, 0, axisLength, 1]))[:3]
        x_axis = pybullet.addUserDebugLine(origin, x, lineColorRGB=[1, 0, 0], lineWidth=lineWidth, lifeTime=lifeTime)
        y_axis = pybullet.addUserDebugLine(origin, y, lineColorRGB=[0, 1, 0], lineWidth=lineWidth, lifeTime=lifeTime)
        z_axis = pybullet.addUserDebugLine(origin, z, lineColorRGB=[0, 0, 1], lineWidth=lineWidth, lifeTime=lifeTime)
        return [x_axis, y_axis, z_axis]

    @staticmethod
    def addDebugRectangle(position, quaternion=[0,0,0,1], length=0.2, width=0.1, color=[0,0,0], lineWidth=1, lifeTime=0):
        point1, quaternion1 = pybullet.multiplyTransforms(position, quaternion, [+length / 2, +width / 2, 0], [0, 0, 0, 1])
        point2, quaternion2 = pybullet.multiplyTransforms(position, quaternion, [-length / 2, +width / 2, 0], [0, 0, 0, 1])
        point3, quaternion3 = pybullet.multiplyTransforms(position, quaternion, [-length / 2, -width / 2, 0], [0, 0, 0, 1])
        point4, quaternion4 = pybullet.multiplyTransforms(position, quaternion, [+length / 2, -width / 2, 0], [0, 0, 0, 1])
        line1 = SimEnv.addDebugLine(point1, point2, color, lineWidth, lifeTime)
        line2 = SimEnv.addDebugLine(point2, point3, color, lineWidth, lifeTime)
        line3 = SimEnv.addDebugLine(point3, point4, color, lineWidth, lifeTime)
        line4 = SimEnv.addDebugLine(point4, point1, color, lineWidth, lifeTime)
        return [line1, line2, line3, line4]


    @staticmethod
    def addDebugTrajectory(X, Y, Z, color=[0,0,0], lineWidth=1, lifeTime=0):
        trajectoryId = []
        for i in range(len(X)-1):
            pointFrom = [X[i], Y[i], Z[i]]
            pointTo = [X[i+1], Y[i+1], Z[i+1]]
            lineId = pybullet.addUserDebugLine(pointFrom, pointTo, lineColorRGB=color, lineWidth=lineWidth, lifeTime=lifeTime)
            trajectoryId.append(lineId)
        return trajectoryId

    @staticmethod
    def addDebugTrajectory3D(XYZ, color=[0,0,0], lineWidth=1, lifeTime=0):
        if XYZ.shape[0] != 3:
            XYZ = XYZ.T
        X, Y, Z = XYZ[0], XYZ[1], XYZ[2]
        return SimEnv.addDebugTrajectory(X,Y,Z, color=color, lineWidth=lineWidth, lifeTime=lifeTime)

    def addDebugTrajectory2D(XYZ, color=[0,0,0], lineWidth=1, lifeTime=0):
        if XYZ.shape[0] != 3:
            XYZ = XYZ.T
        X, Y, Z = XYZ[0], XYZ[1], XYZ[2]*0.0
        return SimEnv.addDebugTrajectory(X,Y,Z, color=color, lineWidth=lineWidth, lifeTime=lifeTime)


    @staticmethod
    def removeDebugItems(*args):
        '''
        remove one or multiple debug items
        :param args: int, list, tuple
            id of the items to be removed
        '''
        if len(args) == 0:
            pybullet.removeAllUserDebugItems()
        else:
            for arg in args:
                if isinstance(arg, int):
                    pybullet.removeUserDebugItem(arg)
                elif isinstance(arg, list) or isinstance(arg, tuple):
                    for item in arg:
                        SimEnv.removeDebugItems(item)

class SimRobot:
    def __init__(self, verbose=True):

        if verbose:
            self.showRobotInfo()

        # self.enableTorqueControl()
        self.enablePositionControl()


        # self.addDebugLinkFrames()

        # if jointPositions is None:
        #     self.resetJointStates(np.zeros(self.getNumJoints()))
        # else:
        #     self.resetJointStates(jointPositions)



    @classmethod
    def fromURDF(cls, urdfFileName, basePosition=[0,0,0], baseRPY=[0,0,0], useFixedBase=True, verbose=True):
        id = pybullet.loadURDF(fileName=urdfFileName,
                                    basePosition=basePosition,
                                    baseOrientation=pybullet.getQuaternionFromEuler(baseRPY),
                                    useFixedBase=useFixedBase,
                                    flags=pybullet.URDF_USE_INERTIA_FROM_FILE)
        cls.id = id
        return cls(verbose=verbose)

    @classmethod
    def fromID(cls, id, verbose=True):
        cls.id = id
        return cls(verbose=verbose)

    @classmethod
    def fromConfig(cls, config, verbose=True):
        cls.config = config
        cls.fromURDF(urdfFileName=config['urdf_path'], basePosition=config['base_position'])
        return cls(verbose=verbose)

    @classmethod
    def fromIdConfig(cls, id, config_path, verbose=True):
        cls.id = id
        import yaml
        with open(config_path, 'r') as stream:
            config = yaml.safe_load(stream)
        cls.config = config
        return cls(verbose=verbose)


    @classmethod
    def loadFromURDF(cls, urdf_path, config_path, verbose=True):
        cls.config = cls.loadYaml(config_path)
        cls.id = pybullet.loadURDF(fileName=urdf_path,
                                    basePosition=cls.config['base_position'],
                                    baseOrientation=pybullet.getQuaternionFromEuler(cls.config['base_rpy']),
                                    useFixedBase=cls.config['fixed_base'],
                                    flags=pybullet.URDF_USE_INERTIA_FROM_FILE)

        return cls(verbose=verbose)

    @classmethod
    def loadFromID(cls, id, config_path, verbose=True):
        cls.config = cls.loadYaml(config_path)
        cls.id = id
        return cls(verbose=verbose)

    @classmethod
    def loadYaml(cls, yaml_path):
        import yaml
        with open(yaml_path, 'r') as stream:
            data = yaml.safe_load(stream)
        return data


    def showRobotInfo(self):
        print('*' * 100 + '\nPyBullet Robot Info ' + '\n' + '*' * 100)
        print('robot ID:              ', self.id)
        print('robot name:            ', self.getRobotName())
        print('robot total mass:      ', self.getTotalMass())
        print('base link name:        ', self.getBaseName())
        print('num of joints:         ', self.getNumJoints())
        print('num of actuated joints:', self.getNumActuatedJoints())
        print('joint names:           ', len(self.getJointNames()), self.getJointNames())
        print('joint indexes:         ', len(self.getJointIndexes()), self.getJointIndexes())
        print('actuated joint names:  ', len(self.getActuatedJointNames()), self.getActuatedJointNames())
        print('actuated joint indexes:', len(self.getActuatedJointIndexes()), self.getActuatedJointIndexes())
        print('link names:            ', len(self.getLinkNames()), self.getLinkNames())
        print('link indexes:          ', len(self.getLinkIndexes()), self.getLinkIndexes())
        print('joint dampings:        ', self.getJointDampings())
        print('joint frictions:       ', self.getJointFrictions())
        print('*' * 100 + '\nPyBullet Robot Info ' + '\n' + '*' * 100)

    def resetJointStates(self, jointPositions, jointVelocities=None):
        if jointVelocities is None:
            jointVelocities = np.zeros(self.getNumActuatedJoints())
        for jointIndex, jointPosition, jointVelocity in zip(self.getActuatedJointIndexes(), jointPositions, jointVelocities):
            pybullet.resetJointState(self.id, jointIndex, jointPosition, jointVelocity)

    def enablePositionControl(self):
        self.maxForce = 500
        pybullet.setJointMotorControlArray(self.id,
                                           range(self.getNumJoints()),
                                           pybullet.POSITION_CONTROL,
                                           targetPositions=self.getJointPositions(),
                                           forces=self.maxForce*np.ones(self.getNumJoints()))
        self.controlMode = 'positionContwrol'
        print(self.controlMode, 'enabled!')

    def enableTorqueControl(self):
        pybullet.setJointMotorControlArray(self.id,
                                           self.getJointIndexes(),
                                           pybullet.VELOCITY_CONTROL,
                                           forces=[0.0]*self.getNumJoints())
        self.controlMode = 'torqueControl'
        print(self.controlMode, 'enabled!')

    def getRobotName(self):
        return pybullet.getBodyInfo(self.id)[1].decode()

    def getTotalMass(self):
        totalMass = 0
        for linkId in range(-1, self.getNumJoints()):
            totalMass += self.getLinkMass(linkId)
        return totalMass

    def getLinkMass(self, link_id):
        return pybullet.getDynamicsInfo(self.id, link_id)[0]

    def getBaseName(self):
        return pybullet.getBodyInfo(self.id)[0].decode()

    def getNumJoints(self):
        return pybullet.getNumJoints(self.id)

    def getNumActuatedJoints(self):
        n = 0
        for i in range(self.getNumJoints()):
            if self.getJointType(i) is not pybullet.JOINT_FIXED:
                n += 1
        return n

    def getJointType(self, jointIndex):
        return pybullet.getJointInfo(self.id, jointIndex)[2]

    def getJointNames(self):
        try:
            return self.joint_names
        except AttributeError:
            self.joint_names = []
            for i in range(self.getNumJoints()):
                joint_name = self.getJointName(i)
                self.joint_names.append(joint_name)
            return self.joint_names

    def getJointName(self, jointIndex):
        return pybullet.getJointInfo(self.id, jointIndex)[1].decode()

    def getJointNameArray(self, jointIndexArray):
        return [self.getJointName(jointIndex) for jointIndex in jointIndexArray]

    def getJointIndex(self, jointName):
        return self.getJointNameIndexMap()[jointName]

    def getJointIndexArray(self, jointNameArray):
        return [self.getJointIndex(jointName) for jointName in jointNameArray]

    def getJointIndexes(self):
        return list(range(self.getNumJoints()))

    def getJointNameIndexMap(self):
        try:
            return self.joint_name_index_map
        except AttributeError:
            joint_indexes = self.getJointIndexes()
            joint_names = self.getJointNames()
            self.joint_name_index_map = dict(zip(joint_names, joint_indexes))
        return self.joint_name_index_map

    def getActuatedJointNames(self):
        try:
            return self.actuated_joint_names
        except AttributeError:
            self.actuated_joint_names = []
            for i in range(self.getNumJoints()):
                if self.getJointType(i) is not pybullet.JOINT_FIXED:
                    self.actuated_joint_names.append(self.getJointName(i))
            return self.actuated_joint_names

    def getActuatedJointIndexes(self):
        try:
            return self.actuated_joint_indexes
        except AttributeError:
            self.actuated_joint_indexes = []
            for joint_name in self.getActuatedJointNames():
                self.actuated_joint_indexes.append(self.getJointIndex(joint_name))
            return self.actuated_joint_indexes

    def getParentLinkName(self, linkIndex):
        parent_link_index = self.getParentLinkIndex(linkIndex)
        if parent_link_index == -1:
            parent_link_name = 'panda_link0'
        else:
            parent_link_name = self.getLinkName(parent_link_index)
        return parent_link_name

    def getParentLinkIndex(self, linkIndex):
        return pybullet.getJointInfo(self.id, linkIndex)[16]

    def getLinkName(self, linkIndex):
        return pybullet.getJointInfo(self.id, linkIndex)[12].decode()

    def getLinkNames(self):
        try:
            return self.linkNames
        except AttributeError:
            self.linkNames = []
            for i in range(self.getNumJoints()):
                linkName = self.getLinkName(i)
                self.linkNames.append(linkName)
            return self.linkNames

    def getLinkIndex(self, linkName):
        return self.getLinkNameIndexMap()[linkName]

    def getLinkIndexes(self):
        return list(range(self.getNumJoints()))

    def getLinkNameIndexMap(self):
        try:
            return self.linkNameIndexMap
        except AttributeError:
            linkIndexes = self.getLinkIndexes()
            linkNames = self.getLinkNames()
            self.linkNameIndexMap = dict(zip(linkNames, linkIndexes))
            return self.linkNameIndexMap

    def getJointDamping(self, jointIndex):
        return pybullet.getJointInfo(self.id, jointIndex)[6]

    def getJointDampings(self):
        joint_dampings = []
        for i in range(self.getNumJoints()):
            joint_dampings.append(self.getJointDamping(i))
        return joint_dampings

    def getJointFriction(self, jointIndex):
        return pybullet.getJointInfo(self.id, jointIndex)[7]

    def getJointFrictions(self):
        joint_frictions = []
        for i in range(self.getNumJoints()):
            joint_frictions.append(self.getJointFriction(i))
        return joint_frictions


    def getJointLowerLimit(self, jointIndex):
        return pybullet.getJointInfo(self.id, jointIndex)[8]

    def getJointLowerLimits(self):
        jointLowerLimits = []
        for i in self.getJointIndexes():
            jointLowerLimits.append(self.getJointLowerLimit(i))
        return jointLowerLimits

    def getJointUpperLimit(self, jointIndex):
        return pybullet.getJointInfo(self.id, jointIndex)[9]

    def getJointUpperLimits(self):
        jointUpperLimits = []
        for i in self.getJointIndexes():
            jointUpperLimits.append(self.getJointUpperLimit(i))
        return jointUpperLimits

    def getBaseState(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState

    def getBasePosition(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[0]

    def getBaseOrientation(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[1]

    def getLinkState(self, linkName):
        linkIndex = self.getLinkIndex(linkName=linkName)
        linkState = pybullet.getLinkState(bodyUniqueId=self.id, linkIndex=linkIndex, computeLinkVelocity=True, computeForwardKinematics=True)
        return linkState

    def getLinkPosition(self, linkName):
        linkState = self.getLinkState(linkName=linkName)
        return linkState[4]

    def getLinkOrientation(self, linkName):
        linkState = self.getLinkState(linkName=linkName)
        return linkState[5]

    def getLinkQuaternion(self, linkName):
        linkState = self.getLinkState(linkName=linkName)
        return linkState[5]

    def getLinkRotation(self, linkName):
        quat = self.getLinkQuaternion(linkName)
        rot = pybullet.getMatrixFromQuaternion(quat)
        return np.reshape(rot,(3,3))

    def getLinkEuler(self, linkName):
        quat = self.getLinkQuaternion(linkName)
        euler = pybullet.getEulerFromQuaternion(quat)
        return euler

    def getLinkLinearVelocity(self, linkName):
        linkState = self.getLinkState(linkName=linkName)
        return linkState[6]

    def getLinkAngularVelocity(self, linkName):
        linkState = self.getLinkState(linkName=linkName)
        return linkState[7]

    def calculateInverseKinematics(self, linkName, position, quaternion):
        lowerLimits = np.array(self.getJointLowerLimits())
        upperLimits = np.array(self.getJointUpperLimits())
        jointRanges = upperLimits - lowerLimits
        restPoses = (lowerLimits + upperLimits)/2.0
        restPoses = self.getActuatedJointPositions()
        # print('lower:', self.getJointLowerLimits())
        # print('upper:', self.getJointUpperLimits())
        # print('range:', jointRanges)
        # print('restPoses:', restPoses)
        jointPositions = pybullet.calculateInverseKinematics(self.id, self.getLinkIndex(linkName), position, quaternion, maxNumIterations=100)
        return jointPositions


    # Set control commands
    def setJointPositions(self, jointNamePositionDict):
        for jointName, jointPosition in jointNamePositionDict.items():
            pybullet.setJointMotorControl2(self.id, self.getJointIndex(jointName), pybullet.POSITION_CONTROL, targetPosition=jointPosition)

    def setJointPositionsWithPDGains(self, jointNamePositionDict, PGain=0.001, DGain=0.01):
        for jointName, jointPosition in jointNamePositionDict.items():
            pybullet.setJointMotorControl2(self.id, self.getJointIndex(jointName), pybullet.POSITION_CONTROL, targetPosition=jointPosition, positionGain=PGain, velocityGain=DGain)

    def setJointPositionsWithForceLimit(self, jointStateDict):
        for joint_tuple in jointStateDict.items():
            jointName, joint_dict = joint_tuple
            max_force = joint_dict['max_force']
            jointPosition = joint_dict['joint_target']
            pybullet.setJointMotorControl2(self.id, self.getJointIndex(jointName), pybullet.POSITION_CONTROL, targetPosition=jointPosition, force=max_force)


    def setActuatedJointPositions(self, jointPositions):
        if isinstance(jointPositions, dict):
            actuatedJointPositions = [jointPositions[jointName] for jointName in self.getActuatedJointNames()]
            pybullet.setJointMotorControlArray(self.id, self.getActuatedJointIndexes(), pybullet.POSITION_CONTROL, targetPositions=actuatedJointPositions)
        elif isinstance(jointPositions, np.ndarray) or isinstance(jointPositions, list) or isinstance(jointPositions, tuple):
            pybullet.setJointMotorControlArray(self.id, self.getActuatedJointIndexes(), pybullet.POSITION_CONTROL, targetPositions=jointPositions)

    def getJointConfig(self):
        jointPositions = np.array([state[0] for state in pybullet.getJointStates(self.id, self.getJointIndexes())])
        return dict(zip(self.getJointNames(), jointPositions))

    def getJointPositions(self):
        jointPositions = np.array([state[0] for state in pybullet.getJointStates(self.id, self.getJointIndexes())])
        return jointPositions

    def getActuatedJointConfig(self):
        actuatedJointPositions = self.getActuatedJointPositions()
        return dict(zip(self.getActuatedJointNames(), actuatedJointPositions))

    def getActuatedJointNamePositions(self):
        actuatedJointPositions = self.getActuatedJointPositions()
        return dict(zip(self.getActuatedJointNames(), actuatedJointPositions))

    def getActuatedJointPositions(self):
        actuatedJointPositions = np.array([state[0] for state in pybullet.getJointStates(self.id, self.getActuatedJointIndexes())])
        return actuatedJointPositions

    def getActuatedJointNameVelocities(self):
        actuatedJointVelocities = self.getActuatedJointVelocities()
        return dict(zip(self.getActuatedJointNames(), actuatedJointVelocities))

    def getActuatedJointVelocities(self):
        actuatedJointVelocities = np.array([state[1] for state in pybullet.getJointStates(self.id, self.getActuatedJointIndexes())])
        return actuatedJointVelocities

class Manipulator(SimRobot):
    def __init__(self, verbose):
        SimRobot.__init__(self, verbose)

        self.dt = 0.001
        self.arm_joint_names = self.config['arm_joint_names']
        self.arm_link_names = self.config['arm_link_names']
        self.gripper_joint_names = self.config['gripper_joint_names']
        self.joint_max_forces = self.config['joint_max_forces']
        self.gripper_link_names = self.config['gripper_link_names']
        self.tool_frame_name = self.config['tool_frame_name']
        self.joint_names = self.arm_joint_names + self.gripper_joint_names
        self.arm_home_config = self.config['arm_home_config']
        self.gripper_home_config = self.config['gripper_home_config']
        self.arm_home_joint_positions = [self.arm_home_config[joint_name] for joint_name in self.arm_joint_names]
        self.gripper_home_joint_positions = [self.gripper_home_config[joint_name] for joint_name in self.gripper_joint_names]
        self.home_joint_positions = self.arm_home_joint_positions + self.gripper_home_joint_positions

        self.tf_broadcaster_initialized = False

        self.changeGripperDynamics(lateralFriction=1, spinningFriction=0.8, rollingFriction=0.0, frictionAnchor=True)

        # self.realsense = Camera()

    def changeGripperDynamics(self, lateralFriction=1.0, spinningFriction=1.0,  rollingFriction=0.1, frictionAnchor=True):
        for gripper_link_name in self.gripper_link_names:
            print(gripper_link_name + " friction updated!")
            pybullet.changeDynamics(bodyUniqueId=self.id, linkIndex=self.getLinkIndex(gripper_link_name), lateralFriction=lateralFriction, spinningFriction=spinningFriction, rollingFriction=rollingFriction,  frictionAnchor=frictionAnchor)

    def getMoveJointNames(self):
        return self.gripper_joint_names + self.arm_joint_names

    def getGripperJointNames(self):
        return self.gripper_joint_names

    def getArmJointNum(self):
        return len(self.arm_joint_names)

    def getGripperJointNum(self):
        return len(self.gripper_joint_names)

    def getRealsensePosition(self):
        return self.getLinkPosition('realsense_camera_frame')

    def getRealsenseOrientation(self):
        return self.getLinkOrientation('realsense_camera_frame')

    def getMoveJointPositions(self):
        actuated_joint_name_positions = self.getActuatedJointNamePositions()
        move_joint_positions = [actuated_joint_name_positions[joint_name] for joint_name in (self.gripper_joint_names + self.arm_joint_names)]
        return move_joint_positions

    def getMoveJointVelocities(self):
        actuated_joint_name_velocities = self.getActuatedJointNameVelocities()
        move_joint_velocities = [actuated_joint_name_velocities[joint_name] for joint_name in (self.gripper_joint_names + self.arm_joint_names)]
        return move_joint_velocities

    def getArmJointStateMsg(self):
        import rospy
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.getMoveJointNames()
        msg.position = self.getMoveJointPositions()
        msg.velocity = self.getMoveJointVelocities()
        return msg

    def generateJointTrajectory(self, times, values):
        from scipy.interpolate import CubicSpline, interp1d
        # CubicSpline
        # jointTrajectory = CubicSpline(x=np.array(times), y=np.array(values), axis=0, bc_type=((1, np.zeros_like(values[0])), (1, np.zeros_like(values[-1]))))
        # # inter1d
        jointTrajectory = interp1d(x=np.array(times), y=np.array(values), axis=0, fill_value=(values[0], values[-1]), bounds_error=False)
        return jointTrajectory

    def get_current_joint_position(self):
        current_config = self.getActuatedJointConfig()
        current_joint_position = [current_config[joint_name] for joint_name in self.joint_names]
        return current_joint_position

    def get_current_arm_joint_position(self):
        current_config = self.getActuatedJointConfig()
        arm_joint_position = [current_config[joint_name] for joint_name in self.arm_joint_names]
        return arm_joint_position

    def get_current_gripper_joint_position(self):
        current_config = self.getActuatedJointConfig()
        gripper_joint_position = [current_config[joint_name] for joint_name in self.gripper_joint_names]
        return gripper_joint_position

    def goHome(self, T=1.0):
        current_pos = self.get_current_arm_joint_position()
        desired_pos = self.arm_home_joint_positions
        joint_trajectory = self.generateJointTrajectory(times=[0,T], values=[current_pos, desired_pos])
        for t in np.arange(0, T, self.dt):
            q = joint_trajectory(t)
            self.setJointPositions(dict(zip(self.arm_joint_names,q)))
            time.sleep(self.dt)

    def goZero(self, T=1.0):
        current_pos = self.get_current_arm_joint_position()
        desired_pos = np.zeros_like(current_pos)
        joint_trajectory = self.generateJointTrajectory(times=[0, T], values=[current_pos, desired_pos])
        for t in np.arange(0, T, self.dt):
            q = joint_trajectory(t)
            self.setJointPositions(dict(zip(self.arm_joint_names,q)))
            time.sleep(self.dt)

    def gotoArmJointConfig(self, desired_position, T=1.0):
        current_pos = self.get_current_arm_joint_position()
        desired_pos = desired_position
        joint_trajectory = self.generateJointTrajectory(times=[0, T], values=[current_pos, desired_pos])
        for t in np.arange(0, T, self.dt):
            q = joint_trajectory(t)
            self.setJointPositions(dict(zip(self.arm_joint_names,q)))
            time.sleep(self.dt)

    def executeArmJointTrajectory(self, trajectory):
        joint_names = trajectory.joint_names
        pos, vel, acc, eff, times = [], [], [], [], []
        for point in trajectory.points:
            pos.append(point.positions)
            vel.append(point.velocities)
            acc.append(point.accelerations)
            eff.append(point.effort)
            times.append(point.time_from_start.secs + point.time_from_start.nsecs*1e-9)
        pos_trajectory = self.generateJointTrajectory(times=times, values=pos)
        for t in np.arange(0, times[-1], self.dt*5):
            joint_state_dict = dict()
            for i in range(len(self.arm_joint_names)):
                joint_state_dict[self.arm_joint_names[i]] = {'joint_target': pos_trajectory(t)[i], 'max_force': self.joint_max_forces[i]}
            self.setJointPositionsWithForceLimit(joint_state_dict)
            time.sleep(self.dt)

    def gotoCartesianTarget(self, position=[0.58, -0.13, 1.05], rpy=[-np.pi ,0 ,-np.pi /2], T=1.0):
        jointPositions = self.calculateInverseKinematics(linkName=self.tool_frame_name,
                                                          position=position,
                                                          quaternion=pybullet.getQuaternionFromEuler(rpy))
        armJointPositions = jointPositions[:self.getArmJointNum()]
        self.gotoArmJointConfig(armJointPositions, T=T)


    def gotoCartesianTarget2(self, position=[0.58, -0.13, 1.05], quaternion=[0,0,0,1], T=1.0):
        jointPositions = self.calculateInverseKinematics(linkName=self.tool_frame_name,
                                                          position=position,
                                                          quaternion=quaternion)
        armJointPositions = jointPositions[:self.getArmJointNum()]
        self.gotoArmJointConfig(armJointPositions, T=T)


    def printTool0Pose(self):
        print('tcp pos:', self.getLinkPosition(self.tool_frame_name))
        print('tcp rpy:', self.getLinkEuler(self.tool_frame_name))

    def getToolPosition(self):
        return self.getLinkPosition(self.tool_frame_name)

    def getToolOrientation(self):
        return self.getLinkOrientation(self.tool_frame_name)

    def robotiq85GripperIK(self, gripper_opening_length):
        gripper_main_control_joint_angle = 0.715 - np.math.asin((gripper_opening_length - 0.010) / 0.1143)  # angle calculation
        gripper_main_control_joint_name = "robotiq_85_left_knuckle_joint"
        mimic_joint_names = ["robotiq_85_right_knuckle_joint",
                             "robotiq_85_left_inner_knuckle_joint",
                             "robotiq_85_left_finger_tip_joint",
                             "robotiq_85_right_inner_knuckle_joint",
                             "robotiq_85_right_finger_tip_joint"]
        mimic_multiplier = [1, 1, -1, 1, -1]
        gripper_joint_config = {gripper_main_control_joint_name: gripper_main_control_joint_angle}
        for i in range(len(mimic_joint_names)):
            gripper_joint_config[mimic_joint_names[i]] = gripper_main_control_joint_angle * mimic_multiplier[i]

        gripper_joint_pos = [gripper_joint_config[joint_name] for joint_name in self.gripper_joint_names]
        return gripper_joint_pos

    def robotiq85GripperFK(self, gripper_joint_pos):
        gripper_main_control_joint_angle = gripper_joint_pos[0]
        gripper_opening_length = 0.1143*np.math.sin(0.715-gripper_main_control_joint_angle) + 0.010
        return gripper_opening_length

    def pandaGripperIK(self, gripper_opening_length):
        gripper_joint_pos = [gripper_opening_length/2, gripper_opening_length/2]
        return gripper_joint_pos

    def pandaGripperFK(self, gripper_joint_pos):
        gripper_opening_length = gripper_joint_pos[0] + gripper_joint_pos[1]
        return gripper_opening_length

    def gripperControl(self, gripper_opening_length, T=1.0):
        des_gripper_pos = self.pandaGripperIK(gripper_opening_length)
        cur_gripper_pos = self.get_current_gripper_joint_position()
        gripper_joint_trajectory = self.generateJointTrajectory(times=[0, T], values=[cur_gripper_pos, des_gripper_pos])

        # move
        for t in np.arange(0, T, self.dt):
            q = gripper_joint_trajectory(t)
            # self.setJointPositions(dict(zip(self.gripper_joint_names, q)))
            self.setJointPositionsWithPDGains(dict(zip(self.gripper_joint_names, q)), PGain=0.002, DGain=0.01)
            # self.setJointPositionsWithForceLimit(dict(zip(self.gripper_joint_names, q)), maxForce=100)
            time.sleep(self.dt)

    def getGripperOpeningLength(self):
        gripper_opening_length = self.robotiq85GripperFK(self.get_current_gripper_joint_position())
        return gripper_opening_length

    def setDesiredGripperOpeningLength(self, length):
        self.desired_gripper_opening_length = length

    def broadcast_tfs(self):
        import rospy
        import tf
        if self.tf_broadcaster_initialized==False:
            self.tf_broadcaster = tf.TransformBroadcaster()
            self.tf_broadcaster_initialized = True
            print ('tf_broadcaster_initialized!')
        else:
            for i in range(self.getNumJoints()):
                current_link_name = self.getLinkName(i)
                parent_link_name = self.getParentLinkName(i)
                if parent_link_name == 'panda_link0':
                    tf_pre = np.dot(tf.transformations.translation_matrix(self.getBasePosition()), tf.transformations.quaternion_matrix(self.getBaseOrientation()))
                else:
                    tf_pre = np.dot(tf.transformations.translation_matrix(self.getLinkPosition(parent_link_name)), tf.transformations.quaternion_matrix(self.getLinkOrientation(parent_link_name)))
                tf_cur = np.dot(tf.transformations.translation_matrix(self.getLinkPosition(current_link_name)), tf.transformations.quaternion_matrix(self.getLinkOrientation(current_link_name)))
                pre_tf_cur = np.dot(np.linalg.inv(tf_pre),tf_cur)
                self.tf_broadcaster.sendTransform(translation=tf.transformations.translation_from_matrix(pre_tf_cur),
                                                 rotation=tf.transformations.quaternion_from_matrix(pre_tf_cur),
                                                 time=rospy.Time.now(),
                                                 child=current_link_name,
                                                 parent=parent_link_name)

class Panda(Manipulator):
    pass

class Camera:
    def __init__(self, name, width=1280, height=720, nearVal = 0.01, farVal = 3.0, camera_focus_distance = 0.1, fov = 43.6):
        self.name = name
        self.width = width
        self.height = height

        # view matrix parameters
        self.camera_focus_distance = camera_focus_distance

        # projection matrix parameters
        self.fov = fov
        self.aspect = float(self.width) / self.height
        self.nearVal = nearVal
        self.farVal = farVal
        self.projectionMatrix = self.calProjectionMatrix()
        self.intrinsicMatrix = self.calIntrinsicMatrix()
        self.matrix_computed = False

        # ros bridge
        from cv_bridge import CvBridge
        self.bridge = CvBridge()

    def init_ros_publiser(self):
        import rospy
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        self.color_camera_info_publisher = rospy.Publisher('/' + self.name + '/color/camera_info', CameraInfo, queue_size=10)
        if self.name == 'realsense':
            self.color_image_raw_publisher = rospy.Publisher('/' + self.name + '/color/image_raw', Image, queue_size=10)
            self.depth_image_raw_publisher = rospy.Publisher('/' + self.name + '/aligned_depth_to_color/image_raw', Image, queue_size=10)
        elif self.name == 'kinect':
            self.color_image_raw_publisher = rospy.Publisher('/' + self.name + '/color/image_rect_color', Image, queue_size=10)
            self.depth_image_raw_publisher = rospy.Publisher('/' + self.name + '/depth_to_color/image_raw', Image, queue_size=10)
        self.point_cloud_raw_publisher = rospy.Publisher('/' + self.name + '/depth/points', PointCloud2, queue_size=10)

    def ros_publish_image(self):
        self.color_camera_info_publisher.publish(self.getCameraInfoMsg())
        self.color_image_raw_publisher.publish(self.getCameraColorImageRawMsg())
        self.depth_image_raw_publisher.publish(self.getCameraDepthImageRawMsg())
        self.point_cloud_raw_publisher.publish(self.getCameraPointCloudRawMsg())

    def loadURDF(self, urdfFileName, basePosition=[0,0,0], baseRPY=[0,0,0], useFixedBase=True, verbose=True):
        id = pybullet.loadURDF(fileName=urdfFileName,
                                    basePosition=basePosition,
                                    baseOrientation=pybullet.getQuaternionFromEuler(baseRPY),
                                    useFixedBase=useFixedBase,
                                    flags=pybullet.URDF_USE_INERTIA_FROM_FILE)
        self.id = id
        if verbose:
            self.showRobotInfo()

    def showRobotInfo(self):
        print('*' * 100 + '\nPyBullet Robot Info ' + '\n' + '*' * 100)
        print('robot name:            ', pybullet.getBodyInfo(self.id)[1].decode())
        print('robot ID:              ', self.id)
        print('*' * 100 + '\nPyBullet Robot Info ' + '\n' + '*' * 100)

    def getBasePosition(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[0]

    def getBaseOrientation(self):
        baseState = pybullet.getBasePositionAndOrientation(self.id)
        return baseState[1]

    def calViewMatrix(self, position, quaternion):
        cameraEyePosition = position
        cameraQuaternion = quaternion
        cameraRotationMatrix = pybullet.getMatrixFromQuaternion(cameraQuaternion)
        cameraRotationMatrix = np.array(cameraRotationMatrix).reshape((3,3))
        cameraTargetTransform = pybullet.multiplyTransforms(positionA=cameraEyePosition, orientationA=cameraQuaternion, positionB=[self.camera_focus_distance,0,0],orientationB=[0,0,0,1])
        cameraTargetPosition = cameraTargetTransform[0]
        cameraUpVector = cameraRotationMatrix[:,-1]
        viewMatrix = pybullet.computeViewMatrix(cameraEyePosition=cameraEyePosition,
                                                    cameraTargetPosition=cameraTargetPosition,
                                                    cameraUpVector=cameraUpVector)
        return viewMatrix

    def calProjectionMatrix(self):
        projectionMatrix = pybullet.computeProjectionMatrixFOV(self.fov, self.aspect, self.nearVal, self.farVal)
        return projectionMatrix

    def calIntrinsicMatrix(self):
        f = math.sqrt(self.width * self.width / 4.0 + self.height * self.height / 4.0) / 2.0 / math.tan(self.fov / 2.0 / 180.0 * math.pi)
        return (f, 0.0, self.width / 2.0 - 0.5, 0.0, f, self.height / 2.0 - 0.5, 0.0, 0.0, 1.0)

    def getCameraImage(self, position, quaternion):
        self.viewMatrix = self.calViewMatrix(position=position, quaternion=quaternion)
        width, height, rgbPixels, depthPixels, segmentationMaskBuffer = pybullet.getCameraImage(width=self.width,
                                                                        height=self.height,
                                                                        viewMatrix=self.viewMatrix,
                                                                        projectionMatrix=self.projectionMatrix,
                                                                        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                                                                        flags=pybullet.ER_NO_SEGMENTATION_MASK)
        return (width, height, rgbPixels, depthPixels, segmentationMaskBuffer)

    def updateCameraImage(self, position, quaternion):
        self.viewMatrix = self.calViewMatrix(position=position, quaternion=quaternion)
        width, height, rgbPixels, depthPixels, segmentationMaskBuffer = pybullet.getCameraImage(width=self.width,
                                                                                                height=self.height,
                                                                                                viewMatrix=self.viewMatrix,
                                                                                                projectionMatrix=self.projectionMatrix,
                                                                                                renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                                                                                                flags=pybullet.ER_NO_SEGMENTATION_MASK)
        self.rgbPixels = rgbPixels
        self.depthPixels = depthPixels

    def getCameraInfoMsg(self):
        import rospy
        from sensor_msgs.msg import JointState, CameraInfo, Image
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = rospy.Time.now()
        if self.name == 'realsense':
            camera_info_msg.header.frame_id = 'realsense_color_optical_frame'
        elif self.name == 'kinect':
            camera_info_msg.header.frame_id = 'rgb_camera_link'
        camera_info_msg.height = self.height
        camera_info_msg.width = self.width

        camera_info_msg.K = self.calIntrinsicMatrix()
        camera_info_msg.P = self.projectionMatrix[:12]
        return camera_info_msg

    def getCameraColorImageRawMsg(self):
        import rospy
        import cv2
        image = cv2.cvtColor(np.uint8(self.rgbPixels), code=cv2.COLOR_RGBA2RGB)
        msg = self.bridge.cv2_to_imgmsg(image)
        msg.header.stamp = rospy.Time.now()
        msg.encoding = 'rgb8'
        if self.name == 'realsense':
            msg.header.frame_id = 'realsense_color_optical_frame'
        elif self.name == 'kinect':
            msg.header.frame_id = 'rgb_camera_link'
        return msg

    def getCameraDepthImageRawMsg(self):
        import rospy
        import cv2
        depth_raw_image = self.farVal * self.nearVal / (self.farVal - (self.farVal - self.nearVal) * self.depthPixels)
        depth_raw_image = (depth_raw_image * 1000).astype(np.uint16)
        msg = self.bridge.cv2_to_imgmsg(depth_raw_image)
        msg.header.stamp = rospy.Time.now()
        if self.name == 'realsense':
            msg.header.frame_id = 'realsense_color_optical_frame'
        elif self.name == 'kinect':
            msg.header.frame_id = 'rgb_camera_link'
        return msg

    def getCameraPointCloudRawMsg(self):
        import rospy
        from sensor_msgs.msg import PointCloud2, PointField
        points, colors = self.generate_scene_pointcloud()
        points = points.astype(np.float32)
        msg = PointCloud2()
        msg.header.stamp = rospy.Time().now()

        C = np.zeros((colors[:, 0].size, 4), dtype=np.uint8)

        C[:, 0] = colors[:, 2].astype(np.uint8)
        C[:, 1] = colors[:, 1].astype(np.uint8)
        C[:, 2] = colors[:, 0].astype(np.uint8)

        C = C.view("uint32")
        C = C.view("float32")
        pointsColor = np.zeros((points.shape[0], 1), \
        dtype={
            "names": ( "x", "y", "z", "rgba" ),
            "formats": ( "f4", "f4", "f4", "f4" )} )

        points = points.astype(np.float32)

        pointsColor["x"] = points[:, 0].reshape((-1, 1))
        pointsColor["y"] = points[:, 1].reshape((-1, 1))
        pointsColor["z"] = points[:, 2].reshape((-1, 1))
        pointsColor["rgba"] = C
        if self.name == 'realsense':
            msg.header.frame_id = "realsense_depth_optical_frame"
        elif self.name == 'kinect':
            msg.header.frame_id = "depth_camera_link"
        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb',12, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = int(np.isfinite(points).all())
        msg.data = pointsColor.tostring()
        return msg

    def generate_scene_pointcloud(self):
        '''Generate point cloud from depth image and color image

        Args:
            depth(str / np.array): Depth image path or depth.
            rgb(str / np.array): RGB image path or RGB values.
            intrinsics(np.array): Camera intrinsics matrix.
            depth_scale(float): The depth factor.

        Returns:
            np.array(float), np.array(int): points and colors
        '''
        intrinsics = np.array(self.calIntrinsicMatrix()).reshape((3, 3))
        depth_scale = 1.0
        depths = self.farVal * self.nearVal / (self.farVal - (self.farVal - self.nearVal) * self.depthPixels)
        colors = cv2.cvtColor(np.uint8(self.rgbPixels), code=cv2.COLOR_RGBA2RGB)

        fx, fy = intrinsics[0,0], intrinsics[1,1]
        cx, cy = intrinsics[0,2], intrinsics[1,2]

        xmap, ymap = np.arange(colors.shape[1]), np.arange(colors.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)

        points_z = depths / depth_scale
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        mask = (points_z > 0)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask]
        colors = colors[mask]
        return points, colors

if __name__ == '__main__':
    sim_env = SimEnv()
    sim_env.resetGUIView(distance=1.3, yaw=90, pitch=-15, target_position=[0,0,0.8])

    # main loop
    while True:
        sim_env.step()

