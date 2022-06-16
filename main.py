import time as tm
import socket as sc
import socket_wrapper as sw
import numpy as np
import os
import pickle
from sklearn.neural_network import MLPRegressor

finity = 20.0  # Needs to be float to obtain ditto numpy array

lidarInputDim = 16

sampleFileName = 'datasets/default.samples'

X = np.loadtxt("datasets/dataSet", delimiter=' ')

modelSaveFile = 'model.sav'


def getTargetVelocity(steeringAngle):
    return (90 - abs(steeringAngle)) / 60


class AIClient:
    def __init__(self):
        self.steeringAngle = 0

        createModel = input("Create new model? [y/N]")

        if 'y' not in createModel.lower():
            try:
                self.neuralNet = pickle.load(open(modelSaveFile, 'rb'))
            except Exception:
                raise FileNotFoundError
        else:
            self.neuralNet = MLPRegressor(random_state=1, max_iter=100000)
            self.neuralNet.fit(X[:, :-1], X[:, -1])
            pickle.dump(self.neuralNet, open(modelSaveFile, 'wb'))
            print(f"Training finished in {self.neuralNet.n_iter_} cycles.")

        with open(sampleFileName, 'w') as self.sampleFile:
            with sc.socket(*sw.socketType) as self.clientSocket:
                self.clientSocket.connect(sw.address)
                self.socketWrapper = sw.SocketWrapper(self.clientSocket)
                self.halfApertureAngle = False

                while True:
                    self.input()
                    self.lidarSweep()
                    self.output()
                    tm.sleep(0.02)

    def input(self):
        sensors = self.socketWrapper.recv()

        if not self.halfApertureAngle:
            self.halfApertureAngle = sensors['halfApertureAngle']
            self.sectorAngle = 2 * self.halfApertureAngle / lidarInputDim
            self.halfMiddleApertureAngle = sensors['halfMiddleApertureAngle']

        self.lidarDistances = sensors['lidarDistances']

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

        sample = [finity for eI in range(lidarInputDim)]
        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            sectorIndex = round(lidarAngle / self.sectorAngle)
            sample[sectorIndex] = min(sample[sectorIndex], self.lidarDistances[lidarAngle])

        numpySample = np.array(sample).reshape(1, -1)

        self.steeringAngle = self.neuralNet.predict(numpySample)[0]
        print(self.steeringAngle)
        self.targetVelocity = getTargetVelocity(self.steeringAngle)

    def output(self):
        print(self.steeringAngle)
        actuators = {
            'steeringAngle': self.steeringAngle * 2,
            'targetVelocity': self.targetVelocity
        }

        self.socketWrapper.send(actuators)


AIClient()
