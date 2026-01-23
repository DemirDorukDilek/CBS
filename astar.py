import numpy as np
import networkx as nx
import heapq
from itertools import product
np.genfromtxt("./mapcik.txt")

net = nx.read_edgelist("./mapcik.txt", edgetype=int)


def astar(start=(1,3),goal=(12,10)):
    OPEN = []

    heapq.heappush(OPEN,(0,start,None))

    CLOSED = []

    while OPEN:
        q = heapq.heappop(OPEN)

        for s in product(*([u, *G.adj[u]] for u in q[1])):
            if s == goal:
                pass
                # TODO return
            g = q[0]+1
            h = None # TODO h
            f = g+h
            # TODO make skips
            heapq.heappush(OPEN,(f,s,q)) # TODO check correctensy
        
        heapq.heappush(CLOSED,q)
            