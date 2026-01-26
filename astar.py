import numpy as np
import networkx as nx
import heapq
from itertools import product
np.genfromtxt("./mapcik.txt")

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
