import csv
import numpy as np
from filterpy.kalman import predict, update
import matplotlib.pyplot as plt


V = np.zeros(shape=(25, 2))
Z = np.zeros(shape=(25, 2))
Q = np.array([[0.0001, 0.00002], [0.00002, 0.0001]])
R = np.array([[0.01, 0.005], [0.005, 0.02]])
x = x2 = np.array([[0.,0.]]).T
H = B = P = np.eye(2)


# P = 0.0001*P         #use this when estimate case 2

with open('data.csv', newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=' ')
    for i, row in enumerate(reader):
        a = row[0].split(',')
        V[i] = [float(a[0]), float(a[1])]
        Z[i] = [float(a[2]), float(a[3])]

result = []
for i in range(25):
    u = np.array([[V[i][0]], [V[i][1]]])
    z = np.array([[Z[i][0]], [Z[i][1]]])
    x, P = predict(x, P, F=1, Q=Q, u=u, B=B)
    x, P = update(x, P, z, R, H)
    result.append((x[:2]).tolist())

plt.plot(Z.T[0], Z.T[1], 'ro')
x1, x2 = zip(*result)
plt.plot(x1, x2, 'b-')
# plt.plot(x21, x22, '-')

plt.show()