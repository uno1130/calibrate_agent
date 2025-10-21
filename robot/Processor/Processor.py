import time
import winsound
import csv
import os
import glob
import socket
import json
import threading
from ctypes import windll
import numpy as np
import matplotlib.pyplot as plt
from Participant.ParticipantMotion import ParticipantMotion
from Robot.CAMotion import CAMotion
from Robot.xArmTransform import xArmTransform
from xarm.wrapper import XArmAPI
from Analysis.ReturnPoint import ReturnPointAnalyzer


class ProcessorClass:
    def __init__(self) -> None:
        #* Load settings
        with open("config/settings.json", "r") as json_file:
            settings = json.load(json_file)

        # record
        self.isExportData = settings["record"]["isExportData"]
        self.recordPath = settings["record"]["recordPath"]
        self.recordNum = settings["record"]["recordNum"]
        self.robotNum = settings["record"]["robotNum"]
        self.angleRecord = settings["record"]["angleRecord"]

        # fusion
        self.weightListPos = settings["fusion"]["weightListPos"]
        self.weightListRot = settings["fusion"]["weightListRot"]

        # motive
        self.motiveserverIpAddress = settings["motive"]["motiveServerIPAddress"]
        self.motivelocalIpAddress = settings["motive"]["motiveLocalIPAddress"]
        self.differenceLimit = settings["fusion"]["differenceLimit"]
        self.participantNum = settings["motive"]["participantNum"]
        self.otherRigidBodyNum = settings["motive"]["otherRigidBodyNum"]
        self.motionDataInputMode = "optitrack"
        self.idList = settings["motive"]["idList"]
        self.eulerOrder = settings["fusion"]["eulerOrder"]
        if self.eulerOrder == 1:
            self.eulerLeft = [0, np.pi/2, np.pi]
            self.eulerRight = [0, (-1) * np.pi/2, 0]
        elif self.eulerOrder == 2:
            self.eulerLeft = [0, 0, 0]
            self.eulerRight = [(-1)*np.pi, 0, 0]

        # setting
        self.armMode = settings["setting"]["armMode"]
        self.sleepTime = settings["setting"]["sleepTime"]
        self.loopTime = settings["setting"]["loopTime"]
        self.frameRate = settings["setting"]["frameRate"]

        # xArm
        if self.armMode in ["xArm1", "xArm2", "xArm3"]:
            self.xArmIpAddress_left = settings[self.armMode]["IPAddress_left"]
            self.initialpos_left = settings[self.armMode]["initialpos_left"]
            self.initialrot_left = settings[self.armMode]["initialrot_left"]
            self.initAngleList_left = settings[self.armMode]["initAngleList_left"]

            self.xArmIpAddress_right = settings[self.armMode]["IPAddress_right"]
            self.initialpos_right = settings[self.armMode]["initialpos_right"]
            self.initialrot_right = settings[self.armMode]["initialrot_right"]
            self.initAngleList_right = settings[self.armMode]["initAngleList_right"]
        
        self.returnPos = []
        self.timeList = []

    def mainloop(self):
        isEnablexArm = True
        self.loopCount = 0
        self.totalLoopCount = 0
        self.taskTime = []
        self.errorCount = 0
        taskStartTime = 0
        velocityList = []


        caMotion = CAMotion(defaultParticipantNum=self.participantNum, otherRigidBodyNum=self.otherRigidBodyNum,differenceLimit=self.differenceLimit)
        transform_left = xArmTransform(initpos=self.initialpos_left, initrot=self.initialrot_left, initangle=self.initAngleList_left)
        transform_right = xArmTransform(initpos=self.initialpos_right, initrot=self.initialrot_right, initangle=self.initAngleList_right)
        participantMotion = ParticipantMotion(defaultParticipantNum=self.participantNum, otherRigidBodyNum=self.otherRigidBodyNum, motionInputSystem=self.motionDataInputMode, mocapServer=self.motiveserverIpAddress, mocapLocal=self.motivelocalIpAddress, idList=self.idList)
        returnAnalyzer = ReturnPointAnalyzer()

        #* Decide the ratio
        weightListPosfloat = list(map(float, self.weightListPos[0:]))
        weightListRotfloat = list(map(float, self.weightListRot[0:]))
        weightList = [weightListPosfloat, weightListRotfloat]

        #* Initialize the xArm
        initial = input("'s': initialize xArm, 'q': quit\n")

        if initial == "q":
            raise KeyboardInterrupt

        if initial == "s" and isEnablexArm:
            arm_1 = XArmAPI(self.xArmIpAddress_left)
            arm_2 = XArmAPI(self.xArmIpAddress_right)
            self.InitializeAll(arm_1, transform_left)
            self.InitializeAll(arm_2, transform_right)


        #! flag
        isMoving = False

        try:
            while True:
                if isMoving:
                    #* Get the position and orientation
                    localPosition = participantMotion.LocalPosition(loopCount=self.loopCount)
                    localRotation = participantMotion.LocalRotation(loopCount=self.loopCount)
                    relativePosition = caMotion.GetRelativePosition(position=localPosition)
                    relativeRotation = caMotion.GetRelativeRotation(rotation=localRotation)
                    # print(localPosition, localRotation)

                    elapsed_time = time.perf_counter() - taskStartTime
                    if elapsed_time >= self.loopTime:
                        raise KeyboardInterrupt

                    #! Calculate the integration
                    robotpos, robotrot = caMotion.participant2robot(relativePosition, relativeRotation, weightList, self.eulerLeft, self.eulerRight)


                    #* Send to xArm
                    if isEnablexArm:
                        arm_1.set_servo_cartesian(transform_left.Transform(relativepos=robotpos["robot1"], relativerot=robotrot["robot1"], isLimit=False))
                        arm_2.set_servo_cartesian(transform_right.Transform(relativepos=robotpos["robot2"], relativerot=robotrot["robot2"], isLimit=False))

                    if time.perf_counter() - taskStartTime > 0.5:
                        velocity = self.velocity(relativePosition["participant1"], time.perf_counter())
                        if velocity:
                            velocityList.append(velocity)


                    #* fix framerate
                    self.fix_framerate((time.perf_counter() - loop_start_time), 1/self.frameRate)
                    self.loopCount += 1
                    self.totalLoopCount += 1
                    loop_start_time = time.perf_counter()


                #* If the flag is False
                else:
                    keycode = input('\n "q": quit, "r": Clean error and init arm, "s": start control \n')

                    if keycode == "q":
                        #* Stop the program
                        if isEnablexArm:
                            arm_1.disconnect()
                            arm_2.disconnect()
                        self.PrintProcessInfo()

                        windll.winmm.timeEndPeriod(1)
                        break

                    elif keycode == "r":
                        #* Clean error
                        if isEnablexArm:
                            self.InitializeAll(arm_1, transform_left)
                            self.InitializeAll(arm_2, transform_right)

                    elif keycode == "s":
                        time.sleep(self.sleepTime)
                        winsound.Beep(400,300)
                        winsound.Beep(700,300)
                        winsound.Beep(1000,300)

                        #* Determination of initial position
                        caMotion.SetOriginPosition(participantMotion.LocalPosition())
                        caMotion.SetInversedMatrix(participantMotion.LocalRotation())

                        #! Roop start flag
                        isMoving = True
                        taskStartTime = loop_start_time = self.stopTime_before = time.perf_counter()


        except KeyboardInterrupt:
            returnVelocity = returnAnalyzer.analyze(velocityList)
            returnVelocity_mean = np.mean(returnVelocity)
            returnVelocity_max = np.max(returnVelocity)
            returnVelocity_min = np.min(returnVelocity)
            print("\nKeyboardInterrupt >> Stop: mainloop")

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo(returnVelocity_mean, returnVelocity_max, returnVelocity_min)

            winsound.Beep(1000,300)
            winsound.Beep(700,300)
            winsound.Beep(400,300)
            
            plt.plot(velocityList)
            plt.title("Velocity Profile")
            plt.xlabel("Frame")
            plt.ylabel("Velocity (m/s)")
            plt.show()

            plt.plot(returnVelocity)
            plt.title("Return Velocities")
            plt.ylabel("Velocity (m/s)")
            plt.show()

            #* Disconnect
            if isEnablexArm:
                arm_1.disconnect()
                arm_2.disconnect()

            windll.winmm.timeEndPeriod(1)

        #* Exception handling
        except:
            print("----- Exception has occurred -----")
            windll.winmm.timeEndPeriod(1)
            import traceback
            traceback.print_exc()


    def PrintProcessInfo(self, returnVelocity_mean, returnVelocity_max, returnVelocity_min):
        print("----- Process info -----")
        print("Total loop count > ", self.loopCount)
        for ttask in self.taskTime:
            print("Task time\t > ", "{:.2f}".format(ttask), "[s]")
            print("Frame Rate\t > ", "{:.2f}".format(self.loopCount/ttask), "[fps]")
        print("Error count\t > ", self.errorCount)
        print("Mean Return Velocity\t > ", "{:.2f}".format(returnVelocity_mean), "[m/s]")
        print("Max Return Velocity\t > ", "{:.2f}".format(returnVelocity_max), "[m/s]")
        print("Min Return Velocity\t > ", "{:.2f}".format(returnVelocity_min), "[m/s]")
        print("------------------------")


    def InitializeAll(self, robotArm, transform, isSetInitPosition=True, isSetInitAngle=True):
        robotArm.connect()
        if robotArm.warn_code != 0:
            robotArm.clean_warn()
        if robotArm.error_code != 0:
            robotArm.clean_error()
        robotArm.motion_enable(enable=True)
        robotArm.set_mode(0)  # set mode: position control mode
        robotArm.set_state(state=0)  # set state: sport state
        # if isSetInitAngle:
        #     init_angle_list = transform.GetInitialAngle()
        #     robotArm.set_servo_angle(angle=init_angle_list, is_radian=False, wait=True)
        if isSetInitPosition:
            initX, initY, initZ, initRoll, initPitch, initYaw = transform.GetInitialTransform()
            robotArm.set_position(x=initX, y=initY, z=initZ, roll=initRoll, pitch=initPitch, yaw=initYaw, wait=True)
        else:
            robotArm.reset(wait=True)
        print("Initialized > xArm")

        robotArm.set_mode(1)
        robotArm.set_state(state=0)


    def fix_framerate(self, process_duration, looptime):
        sleeptime = looptime - process_duration
        if sleeptime < 0:
            pass
        else:
            time.sleep(sleeptime)


    def play_beep(self, frequency: list, duration: list):
        for freq, dur in zip(frequency, duration):
            winsound.Beep(freq, dur)


    def velocity(self, position, time):
        if not self.returnPos:
            self.returnPos.append(position)
            self.timeList.append(time)
        else:
            velocity = np.linalg.norm(position - self.returnPos[0]) / (time - self.timeList[0])
            self.returnPos = []
            self.timeList = []
            return velocity
