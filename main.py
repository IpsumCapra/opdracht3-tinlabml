import time as tm
import socket as sc
import socket_wrapper as sw
import numpy as np
import os
import pickle
from sklearn.neural_network import MLPRegressor

finity = 20.0  # Needs to be float to obtain ditto numpy array

lidarInputDim = 16

sampleFileName = 'datasets/default.samplez'

X = np.loadtxt("datasets/default.samplez", delimiter=' ')

modelSaveFile = 'model.sav'


def getTargetVelocity(steeringAngle):
    return (90 - abs(steeringAngle)) / 60


class AIClient:
    def __init__(self):
        self.steeringAngle = 0

        createModel = input("Create new model? [y/N]: ")

        if 'y' not in createModel.lower():
            try:
                self.neuralNet = pickle.load(open(modelSaveFile, 'rb'))
            except Exception:
                raise FileNotFoundError
            print("Loaded model.")
        else:
            print("Training...")
            self.neuralNet = MLPRegressor(learning_rate_init=0.005,
                                          n_iter_no_change=50,
                                          verbose=True,
                                          random_state=1,
                                          hidden_layer_sizes=(128, 128, 128),
                                          max_iter=100000)
            self.neuralNet.fit(X[:, :-1], X[:, -1])
            print(self.neuralNet.best_loss_)
            pickle.dump(self.neuralNet, open(modelSaveFile, 'wb'))
            print(f"Training finished in {self.neuralNet.n_iter_} cycles.")

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
        sample = [finity for eI in range(lidarInputDim)]
        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            sectorIndex = round(lidarAngle / self.sectorAngle)
            sample[sectorIndex] = min(sample[sectorIndex], self.lidarDistances[lidarAngle])

        lowest = sample[:].sort()[:4]

        for i in range(len(sample)):
            if sample[i] not in lowest:
                sample[i] = finity

        numpySample = np.array(sample).reshape(1, -1)

        self.steeringAngle = self.neuralNet.predict(numpySample)[0]

        self.targetVelocity = getTargetVelocity(self.steeringAngle)

    def output(self):
        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': self.targetVelocity
        }

        self.socketWrapper.send(actuators)


AIClient()
