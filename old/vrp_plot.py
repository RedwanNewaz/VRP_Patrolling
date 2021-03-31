import json
import matplotlib.pyplot as plt
import numpy as np
with open('result.json') as file:
    data = json.load(file)

import networkx as nx

G = nx.DiGraph()

# G.add_edge('A', 'B')
# G.add_edge('B', 'C')
# G.add_edge('C', 'A')
# #

if __name__ == '__main__':
    coords = []
    for key, values in data.items():
        # print(key, list(values.keys())[0])
        G.add_edge(key, list(values.keys())[0])
        #
        # for i, (wp, item) in enumerate(values.items()):
        #     coords += item
        #     if i > 0:
        #         G.add_edge(item[i-1], item[i])

    for cycle in nx.simple_cycles(G):
        print(cycle)

    #     for wp, item in values.items():
    #         coords += item
    # coord = np.array(coords)
    # N = len(coord)
    # depo = np.array([-17, 0])
    #
    # for i in range(N-1):
    #     plt.clf()
    #     plt.scatter(coord[:, 0], coord[:, 1])
    #     plt.scatter(coord[-1, 0], coord[-1, 1])
    #     plt.scatter(depo[0], depo[1], color='g')
    #     plt.plot(coord[:i+1, 0], coord[:i+1, 1])
    #     plt.pause(1)
    #
    # plt.show()


    # (39, 54)