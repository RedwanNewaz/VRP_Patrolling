import numpy as np
import matplotlib.pyplot as plt
from gurobipy import Model, GRB, quicksum

def vrproutes(n, xc,yc, cost_func):
    rnd = np.random
    rnd.seed(0)
    plt.plot(xc[0], yc[0], c='r', marker='s')
    plt.scatter(xc[1:], yc[1:], c='b')
    N = [i for i in range(1, n+1)]
    V = [0] + N
    A = [(i, j) for i in V for j in V if i != j]
    # c = {(i, j): np.hypot(xc[i]-xc[j], yc[i]-yc[j]) for i, j in A}
    c = {(i, j): cost_func(i, j) for i, j in A}
    print(c)
    Q = 20
    q = {i: rnd.randint(1, 10) for i in N}
    # print(q)
    # {1: 2, 2: 5, 3: 7, 4: 3, 5: 9, 6: 3, 7: 3, 8: 3, 9: 1, 10: 2}
    # q = {1: 9, 2: 1, 3: 6, 4: 5, 5: 3, 6: 2, 7: 1, 8: 8, 9: 1, 10: 1}
    mdl = Model('CVRP')

    x = mdl.addVars(A, vtype=GRB.BINARY)
    u = mdl.addVars(N, vtype=GRB.CONTINUOUS)

    mdl.modelSense = GRB.MINIMIZE
    mdl.params.LogToConsole = False
    mdl.setObjective(quicksum(x[i, j]*c[i, j] for i, j in A))

    mdl.addConstrs(quicksum(x[i, j] for j in V if j != i) == 1 for i in N)
    mdl.addConstrs(quicksum(x[i, j] for i in V if i != j) == 1 for j in N)
    mdl.addConstrs((x[i, j] == 1) >> (u[i]+q[j] == u[j])
                   for i, j in A if i != 0 and j != 0)
    mdl.addConstrs(u[i] >= q[i] for i in N)
    mdl.addConstrs(u[i] <= Q for i in N)

    mdl.Params.MIPGap = 0.1
    mdl.Params.TimeLimit = 60  # seconds
    mdl.optimize()

    active_arcs = [a for a in A if x[a].x > 0.99]
    return active_arcs
    # for i, j in active_arcs:
    #     plt.plot([xc[i], xc[j]], [yc[i], yc[j]], c='g', zorder=0)
    # plt.plot(xc[0], yc[0], c='r', marker='s')
    # plt.scatter(xc[1:], yc[1:], c='b')
    # plt.show()