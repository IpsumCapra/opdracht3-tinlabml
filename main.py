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
        self.steeringAngle = 0
        self.brake = True

        with open(sampleFileName, 'w') as self.sampleFile:
            with sc.socket(*sw.socketType) as self.clientSocket:
                self.clientSocket.connect(sw.address)
                self.socketWrapper = sw.SocketWrapper(self.clientSocket)

                while True:
                    event = pygame.event.get()
                    self.output(event)
                    self.logLidarTraining()
                    tm.sleep(0.04)

    def output(self, event):
        keys = pygame.key.get_pressed()

        # Go left.
        if keys[pygame.K_a]:
            self.steeringAngle = max(self.steeringAngle + angleStep, -90)
        # Go right.
        elif keys[pygame.K_d]:
            self.steeringAngle = min(self.steeringAngle - angleStep, 90)
        # else:
        #     if self.steeringAngle > 0:
        #         self.steeringAngle -= angleStep
        #     elif self.steeringAngle < 0:
        #         self.steeringAngle += angleStep

        for e in event:
            if e.type == pygame.KEYDOWN and keys[pygame.K_TAB]:
                self.brake = not self.brake

        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': 0.5 if not self.brake else 0
        }

        self.socketWrapper.send(actuators)

    def logLidarTraining(self):
        pass


ManualControl()
