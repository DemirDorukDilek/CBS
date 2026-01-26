import numpy as np
import networkx as nx
import heapq
from itertools import product
import os

data = "./mapcik.txt"
if os.path.isfile(data):
    mapcik = [
    (1, 2),
    (1, 4),
    (2, 1),
    (2, 5),
    (3, 4),
    (3, 7),
    (4, 1),
    (4, 3),
    (4, 5),
    (4, 8),
    (5, 2),
    (5, 4),
    (5, 6),
    (5, 9),
    (6, 5),
    (6, 10),
    (7, 3),
    (7, 8),
    (8, 4),
    (8, 7),
    (8, 9),
    (8, 11),
    (9, 5),
    (9, 8),
    (9, 10),
    (9, 12),
    (10, 6),
    (10, 9),
    (11, 8),
    (11, 12),
    (12, 9),
    (12, 11),
]
    net = nx.Graph()
    net.add_edges_from(data)
else:
    net = nx.read_edgelist("./mapcik.txt", edgetype=int, nodetype=int)



class Anode:
    def __init__(self,state,cost=None,parent=None):
        self.state=state
        self.cost=cost
        self.parent=parent

def astar(net,start=(1,3),goal=(12,10),heuristic=None):
    OPEN = []

    heapq.heappush(OPEN,Anode(start,0,None))

    CLOSED = []

    while OPEN:
        q = heapq.heappop(OPEN)
        heapq.heappush(CLOSED,q)
        for s in product(*([u, *net.adj[u]] for u in q.state)):
            if s == goal:
                ptr = q
                path = [goal]
                while ptr:
                    path.append(ptr.state)
                    ptr = ptr.parent
                return path.reverse()
            g = q.cost+1
            h = heuristic(s,goal)
            f = g+h
            # TODO make skips
            heapq.heappush(OPEN,Anode(s,f,q)) # TODO check correctensy
  
astar(net)
# TODO make SIC
