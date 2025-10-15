import csv
import threading
import time
import numpy as np
from OptiTrack.OptiTrackStreaming import OptiTrackStreamingManager

# ----- Numeric range remapping ----- #
targetMin = 150
targetMax = 850
originalMin = 0
originalMax = 1

class ParticipantMotion:
    def __init__(self, defaultParticipantNum: int, otherRigidBodyNum: int, motionInputSystem: str = "optitrack", mocapServer: str = "", mocapLocal: str = "", idList: list = []) -> None:

        self.defaultParticipantNum = defaultParticipantNum
        self.otherRigidBodyNum = otherRigidBodyNum
        self.motionInputSystem = motionInputSystem
        self.udpManager = None
        self.recordedMotion = {}
        self.recordedGripperValue = {}
        self.recordedMotionLength = []
        self.InitBendingSensorValues = []
        self.idList = idList

        n = 2
        fp = 10
        fs = 700

        # ----- Initialize participants' motion input system ----- #
        if motionInputSystem == "optitrack":
            self.optiTrackStreamingManager = OptiTrackStreamingManager(defaultParticipantNum=defaultParticipantNum, otherRigidBodyNum=self.otherRigidBodyNum, mocapServer=mocapServer, mocapLocal=mocapLocal, idList=self.idList)

            # ----- Start streaming from OptiTrack ----- #
            streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
            streamingThread.setDaemon(True)
            streamingThread.start()

    def SetInitialBendingValue(self):
        """
        Set init bending value
        """

        if self.gripperInputSystem == "bendingsensor":
            self.InitBendingSensorValues = []

            for i in range(self.bendingSensorNum):
                self.InitBendingSensorValues.append(self.bendingSensors[i].bendingValue)

    def LocalPosition(self, loopCount: int = 0):
        """
        Local position

        Parameters
        ----------
        loopCount: (Optional) int
            For recorded motion.
            Count of loop.

        Returns
        ----------
        participants' local position: dict
        {'participant1': [x, y, z]}
        unit: [m]
        """

        dictPos = {}
        if self.motionInputSystem == "optitrack":
            dictPos = self.optiTrackStreamingManager.position

        return dictPos

    def LocalRotation(self, loopCount: int = 0):
        """
        Local rotation

        Parameters
        ----------
        loopCount: (Optional) int
            For recorded motion.
            Count of loop.

        Returns
        ----------
        participants' local rotation: dict
        {'participant1': [x, y, z, w] or [x, y, z]}
        """

        dictRot = {}
        if self.motionInputSystem == "optitrack":
            dictRot = self.optiTrackStreamingManager.rotation

        return dictRot
