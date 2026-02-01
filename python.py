import numpy as np
import networkx as nx
import heapq
from itertools import product
import os
from graphviz import Digraph
import uuid

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
],(1,3),(12,10)
mapcik2 = [
(10,11),
(10,12),
(11,30),
(12,30),
(20,21),
(20,22),
(21,30),
(22,30),
(30,15),
(30,25)
],(10,20),(15,25)
mapcik3 = [
    (1,2),
    (1,3),
    (2,1),
    (3,2)
], (1,2),(2,1)
mapcik21 = [
(10,11),
(10,12),
(11,50),
(12,50),
(20,21),
(20,22),
(21,50),
(22,50),
(50,15),
(50,25),
(30,31),
(30,32),
(31,50),
(32,50),
(50,35),
],(10,20,30),(15,25,35)
mapa,START,END = mapcik2
net = nx.DiGraph()
net.add_edges_from(mapa)






class Anode:
    __slots__ = ('state','f','g','parent','time')
    def __init__(self,state,f=None,g=0,parent=None,time=0):
        self.state=state
        self.f=f
        self.g = g
        self.parent=parent
        self.time=time
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

def get_path(node):
    vertexlist = [node.state]
    node = node.parent
    edgelist = []
    while node:
        edgelist.append(frozenset((node.state,vertexlist[-1])))
        vertexlist.append(node.state)
        node = node.parent
    vertexlist.reverse()
    edgelist.reverse()
    return vertexlist,edgelist

def reverse_dijkstra(graph, goals):
    rg = graph.reverse()
    h_values = []
    for goal in goals:
        h_values.append(nx.single_source_shortest_path_length(rg,goal))

    return h_values
def SIC(state,h_values):
    return sum([h_values[aidx][x] if x in h_values[aidx] else float("inf") for aidx,x in enumerate(state)])

def astar(G,start=(1,3),goal=(12,10),heuristic=None,heuristic_precalculator=None,constraint=None,agent=None):
    # Local variable cache
    heappush = heapq.heappush
    heappop = heapq.heappop
    adj = G.adj

    OPEN = []
    CLOSED = {}

    if heuristic_precalculator:
        h_values = heuristic_precalculator(G,goal)

    node = Anode(start,heuristic(start,h_values),0,None)
    OPENd = {(start,node.time):node.f}
    heappush(OPEN,node)

    while OPEN:
        q = heappop(OPEN)
        OPENd.pop((q.state,q.time))

        if is_goal(q.state,goal):return get_path(q)
        if (q.state,q.time) in CLOSED: continue
        CLOSED[(q.state,q.time)] = q.f
        for s in get_successor(q.state,adj):
            if not MAPF_valid(q.state,s): continue
            invalid = False
            for idx in range(len(agent)):
                if constraint:
                    print
                for cagent in constraint.get((s[idx],q.time+1),set()):
                    if agent[idx] in cagent:
                        invalid = True
                        break
                for cagent in constraint.get((frozenset((q.state[idx],s[idx])),q.time+1),set()):
                    if agent[idx] in cagent:
                        invalid = True
                        break
            if invalid:
                continue

            g = q.g+1
            h = heuristic(s,h_values)
            f = g+h

            n = Anode(s,f,g,q,q.time+1)
            if OPENd.get((n.state,n.time),float("inf")) > n.f:
                heappush(OPEN,n)
                OPENd[(n.state,n.time)] = n.f
                
                
                
                
                
                
class CTNode:
    def __init__(self,cost = None,con = None,vertexlist = None,edgelist = None, parent = None):
        self.cost = cost
        self.con = con
        self.vertexlist = vertexlist
        self.edgelist = edgelist
        self.parent = None
        self.children = []
        self.uuid = str(uuid.uuid1())

    def root(self):
        self.cost = 0
        self.con = {}
        self.vertexlist = []
        self.edgelist = []

    def branch(self,new_con):
        con = self.con.copy()
        con[new_con[1:]] = self.con.get(new_con[1:],set()).union(set((new_con[0],)))
        node = CTNode(self.cost,con,self.vertexlist.copy(),self.edgelist.copy(), self)
        return node

    def __repr__(self):
        temp = []
        for i in self.vertexlist:
            temp.append(str([x[0] if len(x) == 1 else x for x in i]))
        return str(self.con) + "\nCost:" + str(self.cost) + "\nSolutions:\n\t" + "\n\t".join(temp)

    def __str__(self):
        txt = ""
        cc = f"    Cost:{self.cost}"
        if self.con:
            for loop,i in enumerate(self.con):
                for v in self.con[i]:
                    if type(i[0]) == frozenset:
                        x = (v,tuple(i[0]),i[1])
                    else:
                        x = (v,i[0],i[1])
                    txt += str(x)
                    if not loop:
                        txt += cc
                    else:
                        txt += " "*(len(cc)+4)
                    txt += "\n"
        else:
            txt = "{}"+f"    Cost:{self.cost}\n"
        txt+="-------\n"
        for loop,i in enumerate(self.vertexlist):
            txt += str([x[0] if type(x)==tuple else x for x in i])
            txt += "\n"
        return txt


    def __lt__(self,o):
        return self.cost<o.cost


class Metaagent:
    
    def __init__(self,inital,start,goal):
        self.agent = inital
        self.start = start
        self.goal = goal
    
    def __contains__(self,o):
        if type(o) == Metaagent:
            for i in o.agent:
                if i not in self.agent:
                    return False
            return True
        else:
            return o in self.agent
    
    def __str__(self):
        return "M"+"".join(map(str,self.agent))
    
    def __repr__(self):
        return str(self)

def check_conflit(solution,agentindex,offset=0):
    conflit = None
    for timestep,i in enumerate(zip(*solution)):
        allocate={}
        for agent,j in enumerate(i):
            if j not in allocate:
                allocate[j] = agent
            else:
                conflit = ((agentindex[allocate[j]],agentindex[agent]),j,timestep+offset)
                break
        if conflit:
            break
    return conflit

def transpose(v,e):
    return list(zip(*v)),[[frozenset(tuple[i] for tuple in s) for s in e]for i in range(len(list(list(e[0])[0])))]

def macbs(G,start,goal):
    tree = Digraph();tree.attr(rankdir="TB")
    
    agents=[]
    agentindex={}
    for i in range(len(start)):
        agents.append(Metaagent((i,),(start[i],),(goal[i],)))
        agentindex[i]=agents[-1]
    
    OPEN = []
    root = CTNode()
    root.root()
    for i in agents:
        x = astar(G,i.start,i.goal,heuristic=SIC,heuristic_precalculator=reverse_dijkstra,constraint={},agent=i.agent)
        if x:
            vl,el = transpose(*x)
            root.vertexlist.append(vl[0])
            root.edgelist.append(el[0])
            root.cost += len(root.vertexlist[-1])-1
        else:
            return

    heapq.heappush(OPEN,root)
    tree.node(root.uuid,str(root),shape="box")

    while OPEN:
        # input(":")
        # # print()
        # # print()
        # # print()
        # # print()
        # # print()
        # # for i in OPEN:
        # #     print()
        # #     print(i.__repr__())
        tree.render("./tree",view=False,format="png",cleanup=True)


        q = heapq.heappop(OPEN)
        q:CTNode
        conflit = check_conflit(q.vertexlist,agentindex)
        if not conflit:
            conflit = check_conflit(q.edgelist,agentindex,1)
            if not conflit:
                return q.vertexlist,root

        for i in q.con
        
        for i in conflit[0]:
            new = q.branch((i,conflit[1],conflit[2]))
            x = astar(G,i.start,i.goal,heuristic=SIC,heuristic_precalculator=reverse_dijkstra,constraint=new.con,agent=i.agent)
            if x:
                vl,el = transpose(*x)
                for idx,j in enumerate(i.agent):
                    new.vertexlist[j],new.edgelist[j] = vl[idx],el[idx]
                    new.cost += len(new.vertexlist[j])-len(q.vertexlist[j])

                heapq.heappush(OPEN,new)
                q.children.append(new)
                tree.node(new.uuid,str(new),shape="box")
                tree.edge(q.uuid,new.uuid)
                
raw,root = macbs(net,start=START,goal=END)
temp = []
for i in raw:
    temp.append(str([x[0] if type(x) == tuple else x for x in i]))
print("Final Solution:",end="")
print("",*temp,sep="\n\t")
