import numpy as np
from scipy.linalg import solve_continuous_are

A = np.array([[  0.0,  1.0, 0.0],
              [ 40.9,  0.0, 0.0],
              [-40.9,  0.0, 0.0]])

Q=np.array([[1.62,     0,    0],
           [   0, 0.101,    0],
           [   0,     0, 0.04]])

B = np.array([[0.0], [-556.0], [53187.0]])    # shape (3,1)

R=np.array([[100000.0]])

x0 = np.array([[0.1],[0.0],[0.0]])


C=np.hstack([B, A@B, A@A@B])
if np.linalg.matrix_rank(C)==3:
    P = solve_continuous_are(A,B,Q,R)
    K=np.linalg.inv(R)@B.T@P
    u = -K @ x0
    eigs = np.linalg.eigvals(A - B@K)
    slow = eigs[np.argmin(np.abs(eigs.real))]
    tau  = 1.0 / abs(slow.real)
    speed = 3874 * 0.1 * tau

    print(f"peak torque: {u}")
    print(f"slow pole time constant: {tau}")
    print(f"arm speed needed: {speed}")
    print(f"K: {K}")

else:
    print(f"not controllable, rank {np.linalg.matrix_rank(C)}")
