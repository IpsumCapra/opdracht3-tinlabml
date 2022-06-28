import time as tm
import socket as sc
import socket_wrapper as sw
import numpy as np
import os
import pickle
from sklearn.neural_network import MLPRegressor

# Lidar settings.
finity = 20.0  # Needs to be float to obtain ditto numpy array

lidarInputDim = 16

# File settings.
sampleFileName = 'datasets/defaultz.samples'

X = np.loadtxt("datasets/defaultz.samples", delimiter=' ')

modelSaveFile = 'model.sav'


# Calculate target velocity, based on steering angle.
def getTargetVelocity(steeringAngle):
    return (90 - abs(steeringAngle)) / 60


class AIClient:
    def __init__(self):
        self.steeringAngle = 0

        # Create model dialogue, load a model, or create a new one.
        createModel = input("Create new model? [y/N]: ")

        if 'y' not in createModel.lower():
            # Load neural network into system.
            try:
                self.neuralNet = pickle.load(open(modelSaveFile, 'rb'))
            except Exception:
                raise FileNotFoundError
            print("Loaded model.")
        else:
            # Train a new one based on passed sample set.
            print("Training...")
            self.neuralNet = MLPRegressor(learning_rate_init=0.005,
                                          n_iter_no_change=30,
                                          verbose=True,
                                          hidden_layer_sizes=120,
                                          alpha=5,
                                          max_iter=100000)
            self.neuralNet.fit(X[:, :-1], X[:, -1])
            print(self.neuralNet.best_loss_)
            pickle.dump(self.neuralNet, open(modelSaveFile, 'wb'))
            print(f"Training finished in {self.neuralNet.n_iter_} cycles.")

        # Start connection to the world.
        with sc.socket(*sw.socketType) as self.clientSocket:
            self.clientSocket.connect(sw.address)
            self.socketWrapper = sw.SocketWrapper(self.clientSocket)
            self.halfApertureAngle = False

            # Code to drive the car.
            while True:
                self.input()
                self.lidarSweep()
                self.output()
                tm.sleep(0.02)

    # Collect LIDAR information.
    def input(self):
        sensors = self.socketWrapper.recv()

        if not self.halfApertureAngle:
            self.halfApertureAngle = sensors['halfApertureAngle']
            self.sectorAngle = 2 * self.halfApertureAngle / lidarInputDim
            self.halfMiddleApertureAngle = sensors['halfMiddleApertureAngle']

        self.lidarDistances = sensors['lidarDistances']

    # Process LIDAR information, send to neural network, set target velocity, and steering angle.
    def lidarSweep(self):
        lidarInfo = [finity for eI in range(lidarInputDim)]
        for lidarAngle in range(-self.halfApertureAngle, self.halfApertureAngle):
            sectorIndex = round(lidarAngle / self.sectorAngle)
            lidarInfo[sectorIndex] = min(lidarInfo[sectorIndex], self.lidarDistances[lidarAngle])

        # Single out two closest values.
        lowest = lidarInfo[:]
        lowest.sort()
        lowest = lowest[:2]

        for i in range(len(lidarInfo)):
            if lidarInfo[i] not in lowest:
                lidarInfo[i] = finity

        # Turn into data acceptable by neural network.
        numpySample = np.array(lidarInfo).reshape(1, -1)

        self.steeringAngle = self.neuralNet.predict(numpySample)[0]
        self.targetVelocity = getTargetVelocity(self.steeringAngle)

    # Send outputs to server (world).
    def output(self):
        actuators = {
            'steeringAngle': self.steeringAngle,
            'targetVelocity': self.targetVelocity
        }

        self.socketWrapper.send(actuators)


# Start the AI Client.
AIClient()
