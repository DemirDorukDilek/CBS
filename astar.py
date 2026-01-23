import numpy as np
import networkx as nx
import heapq
from itertools import product
np.genfromtxt("./mapcik.txt")

net = nx.read_edgelist("./mapcik.txt", edgetype=int)


def astar(start=(1,3)):
    OPEN = []

    heapq.heappush(OPEN,(0,start))

    CLOSED = []

    while OPEN:
        q = heapq.heappop(OPEN)
        u,v = q

        Nu = np.fromiter(G.adj[u], dtype=np.int64)
        Nv = np.fromiter(G.adj[v], dtype=np.int64)

        A, B = np.meshgrid(Nu, Nv, indexing="ij")
        pairs = np.column_stack((A.ravel(), B.ravel()))

        for s in pairs:
            



import networkx as nx
from itertools import product, combinations

G = nx.read_edgelist("data.txt", nodetype=int)

nodes = [1, 2, 4] # N node

CN = {u: {u, *G.adj[u]} for u in nodes}
pair_products = {
    (u, v): product(CN[u], CN[v])
    for u, v in combinations(nodes, 2)
}

# örnek tüketim:
u, v = 1, 2
print(list(pair_products[(u, v)]))

