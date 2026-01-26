import numpy as np
import networkx as nx
import heapq
from itertools import product
import os

data = "./mapcik.txt"
if not os.path.isfile(data):
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
    net = nx.DiGraph()
    net.add_edges_from(mapcik)
else:
    net = nx.read_edgelist("./mapcik.txt", edgetype=int, nodetype=int)


def reverse_dijkstra(graph, goals):
    rg = graph.reverse()
    h_values = []
    for goal in goals:
        h_values.append(nx.single_source_shortest_path_lenght(rg,goal))
    
    return h_values



class Anode:
    def __init__(self,state,cost=None,parent=None):
        self.state=state
        self.cost=cost
        self.parent=parent

def astar(net,start=(1,3),goal=(12,10),heuristic=None):
    OPEN = []
    node = Anode(start,0,None)
    OPENd = {start:node.cost}
    heapq.heappush(OPEN,node)

    CLOSED = {}

    h_values = reverse_dijkstra(net,goal)
    print(h_values)

    while OPEN:
        q = heapq.heappop(OPEN)
        OPENd.pop(q.state)
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
            n = Anode(s,f,q)
            if OPENd.get(n.state,float("inf")) > n.cost and CLOSED.get(n.state,float("inf")) > n.cost:
                heapq.heappush(OPEN,) # TODO check correctensy
  
astar(net)
# TODO make SIC
