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


class Anode:
    __slots__ = ('state','f','g','parent')
    def __init__(self,state,f=None,g=0,parent=None):
        self.state=state
        self.f=f
        self.g = g
        self.parent=parent
    def __lt__(self,o):
        return self.f<o.f
    def __repr__(self):
        return str(self.state)

def get_successor(state,adj):
    return product(*((u,)+tuple(adj[u]) for u in state))

def MAPF_valid(os,ns):
    return len(os) == len(set(ns)) and len(os) == len({frozenset(x) for x in zip(os,ns)})

def is_goal(state,goal):
    return state == goal

def get_path(node,goal):
    path = [goal]
    while node:
        path.append(node.state)
        node = node.parent
    path.reverse()
    return path

def astar(G,start=(1,3),goal=(12,10),heuristic=None,heuristic_precalculator=None):
    # Local variable cache
    heappush = heapq.heappush
    heappop = heapq.heappop
    adj = G.adj

    OPEN = []
    CLOSED = {}

    node = Anode(start,0,0,None)
    OPENd = {start:node.f}
    heappush(OPEN,node)

    if heuristic_precalculator:
        h_values = heuristic_precalculator(G,goal)

    while OPEN:
        q = heappop(OPEN)
        OPENd.pop(q.state)

        if is_goal(q.state,goal):return get_path(q,goal)
        if q.state in CLOSED: continue
        CLOSED[q.state] = q.f

        for s in get_successor(q.state,adj):
            if not MAPF_valid(q.state,s): continue

            g = q.g+1
            h = heuristic(s,h_values)
            f = g+h

            n = Anode(s,f,g,q)
            if OPENd.get(n.state,float("inf")) > n.f and CLOSED.get(n.state,float("inf")) > n.f:
                heappush(OPEN,n)
                OPENd[n.state] = n.f

def reverse_dijkstra(graph, goals):
    rg = graph.reverse()
    h_values = []
    for goal in goals:
        h_values.append(nx.single_source_shortest_path_length(rg,goal))
    
    return h_values
def SIC(state,h_values):
    return sum([h_values[aidx][x] for aidx,x in enumerate(state)])



print(astar(net,heuristic=SIC,heuristic_precalculator=reverse_dijkstra))
