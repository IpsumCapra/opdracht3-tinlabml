import time as tm
import socket_wrapper as sw
import sys as ss
import os
import socket as sc
import pygame

ss.path += [os.path.abspath(relPath) for relPath in ('..',)]

finity = 20.0  # Needs to be float to obtain ditto numpy array

lidarInputDim = 16
sonarInputDim = 3

sampleFileName = 'default.samples'


def getTargetVelocity(steeringAngle):
    return (90 - abs(steeringAngle)) / 60


pygame.init()
display = pygame.display.set_mode((300, 300))

angleStep = 1


class ManualControl:
    def __init__(self):

        self.manual = True if not 'n' in input("Manual? [Y/n]").lower() else False

        self.halfApertureAngle = None
        self.sectorAngle = None
        self.halfMiddleApertureAngle = None
        self.lidarDistances = None

        self.steeringAngle = 0
        self.brake = True

        with open(sampleFileName, 'w') as self.sampleFile:
            with sc.socket(*sw.socketType) as self.clientSocket:
                self.clientSocket.connect(sw.address)
                self.socketWrapper = sw.SocketWrapper(self.clientSocket)

                while True:
                    event = pygame.event.get()
                    self.input()
                    if not self.manual:
                        self.lidarSweep()
                    self.output(event)
                    if not self.brake:
                        self.logLidarTraining()
                    tm.sleep(0.04)

    def input(self):
        sensors = self.socketWrapper.recv()

        self.halfApertureAngle = sensors['halfApertureAngle']
        self.sectorAngle = 2 * self.halfApertureAngle / lidarInputDim
        self.halfMiddleApertureAngle = sensors['halfMiddleApertureAngle']

        self.lidarDistances = sensors['lidarDistances']

    def output(self, event):
        keys = pygame.key.get_pressed()

        if self.manual:
            # Go left.
            if keys[pygame.K_a]:
                self.steeringAngle = max(self.steeringAngle + angleStep, -90)
            # Go right.
            elif keys[pygame.K_d]:
                self.steeringAngle = min(self.steeringAngle - angleStep, 90)

        for e in event:
            if e.type == pygame.KEYDOWN and keys[pygame.K_TAB]:
                self.brake = not self.brake

        self.targetVelocity = 0.5 if self.manual else getTargetVelocity(self.steeringAngle)

        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': self.targetVelocity if not self.brake else 0
        }

        self.socketWrapper.send(actuators)

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

    def logLidarTraining(self):
        sample = [finity for entryIndex in range(lidarInputDim + 1)]

        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            sectorIndex = round(lidarAngle / self.sectorAngle)
            sample[sectorIndex] = min(sample[sectorIndex], self.lidarDistances[lidarAngle])

        lowest = sample[:]
        lowest.sort()
        lowest = lowest[:]

        for i in range(len(sample)):
            if sample[i] not in lowest:
                sample[i] = finity

        sample[-1] = self.steeringAngle
        print(*sample, file=self.sampleFile)


ManualControl()
