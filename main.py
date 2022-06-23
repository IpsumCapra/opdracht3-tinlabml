import time as tm
import socket_wrapper as sw
import sys as ss
import os
import socket as sc
import pygame

# Program parameters.
ss.path += [os.path.abspath(relPath) for relPath in ('..',)]

finity = 20.0  # Needs to be float to obtain ditto numpy array

lidarInputDim = 16
sonarInputDim = 3

sampleFileName = 'default.samples'

angleStep = 1


def getTargetVelocity(steeringAngle):
    return (90 - abs(steeringAngle)) / 60


# Start pygame for manual control.
pygame.init()
display = pygame.display.set_mode((300, 300))


class ManualControl:
    def __init__(self):
        # Initial defines.
        # Determine whether to allow user control, or use hardcoded driving.
        self.manual = True if not 'n' in input("Manual? [Y/n]").lower() else False

        self.halfApertureAngle = None
        self.sectorAngle = None
        self.halfMiddleApertureAngle = None
        self.lidarDistances = None

        self.steeringAngle = 0
        self.brake = True

        # Connect to world.
        with open(sampleFileName, 'w') as self.sampleFile:
            with sc.socket(*sw.socketType) as self.clientSocket:
                self.clientSocket.connect(sw.address)
                self.socketWrapper = sw.SocketWrapper(self.clientSocket)

                # Car driving loop.
                while True:
                    # Get key presses.
                    event = pygame.event.get()

                    self.input()

                    # Either use user input to drive, or drive hardcoded.
                    if not self.manual:
                        self.lidarSweep()
                    self.output(event)

                    # Only log information while actually driving.
                    if not self.brake:
                        self.logLidarTraining()
                    tm.sleep(0.04)

    # Process world sensor input.
    def input(self):
        sensors = self.socketWrapper.recv()

        self.halfApertureAngle = sensors['halfApertureAngle']
        self.sectorAngle = 2 * self.halfApertureAngle / lidarInputDim
        self.halfMiddleApertureAngle = sensors['halfMiddleApertureAngle']

        self.lidarDistances = sensors['lidarDistances']

    # Output to world.
    def output(self, event):

        # Get pressed keys, check manual steering angle, only if enabled.
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

        # Set target velocity according to driving mode (auto/manual)
        self.targetVelocity = 0.5 if self.manual else getTargetVelocity(self.steeringAngle)

        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': self.targetVelocity if not self.brake else 0 # Only drive if brake is off.
        }

        self.socketWrapper.send(actuators)

    # Standard lidar sweep function from hardcoded_client.py
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

    # Altered logging from hardcoded_client.py
    def logLidarTraining(self):
        sample = [finity for entryIndex in range(lidarInputDim + 1)]

        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            sectorIndex = round(lidarAngle / self.sectorAngle)
            sample[sectorIndex] = min(sample[sectorIndex], self.lidarDistances[lidarAngle])

        # Alter information to only store two closest items. Other items are set to finity. (Like in AIClient)
        lowest = sample[:]
        lowest.sort()
        lowest = lowest[:2]

        for i in range(len(sample)):
            if sample[i] not in lowest:
                sample[i] = finity

        sample[-1] = self.steeringAngle
        print(*sample, file=self.sampleFile)

# Run manual control.
ManualControl()
