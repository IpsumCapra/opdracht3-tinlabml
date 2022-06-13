import time as tm
import traceback as tb
import math as mt
import sys as ss
import os
import socket as sc

ss.path += [os.path.abspath(relPath) for relPath in ('..',)]

finity = 20.0   # Needs to be float to obtain ditto numpy array

lidarInputDim = 16
sonarInputDim = 3

sampleFileName = 'default.samples'

def getTargetVelocity (steeringAngle):
    return (90 - abs (steeringAngle)) / 60

import socket_wrapper as sw


class HardcodedClient:
    def __init__(self):
        self.steeringAngle = 0

        with open(sampleFileName, 'w') as self.sampleFile:
            with sc.socket(*sw.socketType) as self.clientSocket:
                self.clientSocket.connect(sw.address)
                self.socketWrapper = sw.SocketWrapper(self.clientSocket)
                self.halfApertureAngle = False

                while True:
                    self.input()
                    self.sweep()
                    self.output()
                    self.logTraining()
                    tm.sleep(0.02)

    def input(self):
        sensors = self.socketWrapper.recv()

        if not self.halfApertureAngle:
            self.halfApertureAngle = sensors['halfApertureAngle']
            self.sectorAngle = 2 * self.halfApertureAngle / lidarInputDim
            self.halfMiddleApertureAngle = sensors['halfMiddleApertureAngle']

        if 'lidarDistances' in sensors:
            self.lidarDistances = sensors['lidarDistances']
        else:
            self.sonarDistances = sensors['sonarDistances']

    def lidarSweep(self):
        nearestObstacleDistance = finity
        nearestObstacleAngle = 0

        nextObstacleDistance = finity
        nextObstacleAngle = 0

        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            lidarDistance = self.lidarDistances[lidarAngle]

            if lidarDistance < nearestObstacleDistance:
                nextObstacleDistance = nearestObstacleDistance
                nextObstacleAngle = nearestObstacleAngle

                nearestObstacleDistance = lidarDistance
                nearestObstacleAngle = lidarAngle

            elif lidarDistance < nextObstacleDistance:
                nextObstacleDistance = lidarDistance
                nextObstacleAngle = lidarAngle

        targetObstacleDistance = (nearestObstacleDistance + nextObstacleDistance) / 2

        self.steeringAngle = (nearestObstacleAngle + nextObstacleAngle) / 2
        self.targetVelocity = getTargetVelocity(self.steeringAngle)

    def sonarSweep(self):
        obstacleDistances = [finity for sectorIndex in range(3)]
        obstacleAngles = [0 for sectorIndex in range(3)]

        for sectorIndex in (-1, 0, 1):
            sonarDistance = self.sonarDistances[sectorIndex]
            sonarAngle = 2 * self.halfMiddleApertureAngle * sectorIndex

            if sonarDistance < obstacleDistances[sectorIndex]:
                obstacleDistances[sectorIndex] = sonarDistance
                obstacleAngles[sectorIndex] = sonarAngle

        if obstacleDistances[-1] > obstacleDistances[0]:
            leftIndex = -1
        else:
            leftIndex = 0

        if obstacleDistances[1] > obstacleDistances[0]:
            rightIndex = 1
        else:
            rightIndex = 0

        self.steeringAngle = (obstacleAngles[leftIndex] + obstacleAngles[rightIndex]) / 2
        self.targetVelocity = getTargetVelocity(self.steeringAngle)

    def sweep(self):
        if hasattr(self, 'lidarDistances'):
            self.lidarSweep()
        else:
            self.sonarSweep()

    def output(self):
        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': self.targetVelocity
        }

        self.socketWrapper.send(actuators)

    def logLidarTraining(self):
        sample = [finity for entryIndex in range(lidarInputDim + 1)]

        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            sectorIndex = round(lidarAngle / self.sectorAngle)
            sample[sectorIndex] = min(sample[sectorIndex], self.lidarDistances[lidarAngle])

        sample[-1] = self.steeringAngle
        print(*sample, file=self.sampleFile)

    def logSonarTraining(self):
        sample = [finity for entryIndex in range(sonarInputDim + 1)]

        for entryIndex, sectorIndex in ((2, -1), (0, 0), (1, 1)):
            sample[entryIndex] = self.sonarDistances[sectorIndex]

        sample[-1] = self.steeringAngle
        print(*sample, file=self.sampleFile)

    def logTraining(self):
        if hasattr(self, 'lidarDistances'):
            self.logLidarTraining()
        else:
            self.logSonarTraining()


HardcodedClient()
