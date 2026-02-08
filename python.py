import numpy as np
import networkx as nx
import heapq
from itertools import product
import os
from graphviz import Digraph
import uuid
import json
import time

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
mapa,START,END = mapcik21
with open("graph.json","r") as f:
    x = json.load(f)
    mapa,START,END = x["edges"],x["start"],x["goal"]
START = START[0],START[1],START[3],START[6],START[5],START[2]
END = END[0],END[1],END[3],END[6],END[5],END[2]
# START = START[2],START[3],START[5]
# END = END[2],END[3],END[5]

# START = START[0],START[4],START[2]
# END = END[0],END[4],END[2]

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
        return str((self.state,self.f,self.g))

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

def astar(G,start=(1,3),goal=(12,10),end_time=None,heuristic=None,heuristic_precalculator=None,constraint=None,agent=None):
    # Local variable cache
    # print(constraint)
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

        if is_goal(q.state,goal):
            # print("end")
            if end_time==None or q.time >= end_time:
                return get_path(q)
        if (q.state,q.time) in CLOSED: continue
        CLOSED[(q.state,q.time)] = q.f
        # print(q, q.parent)
        for s in get_successor(q.state,adj):
            if (s,q.time+1) in CLOSED: continue
            if not MAPF_valid(q.state,s):
                # print(q.state,s, "AAAAAAAAAAAAAAA")
                continue
            invalid = False
            for idx in range(len(agent)):
                if constraint:
                    print
                for cagent in constraint.get((s[idx],q.time+1),set()):
                    if agent[idx] in cagent.agent:
                        # print(cagent,s[idx],q.time+1)
                        invalid = True
                        break
                if invalid:
                    break
                for cagent in constraint.get((frozenset((q.state[idx],s[idx])),q.time+1),set()):
                    if agent[idx] in cagent.agent:
                        # print(cagent,s[idx],q.time+1)
                        invalid = True
                        break
                if invalid:
                    break
            if invalid:
                continue

            n = Anode(s,0,q.g,q,q.time+1)

            n.g += sum(1 for i in range(len(start)) if not (q.state[i] == goal[i] and s[i] == goal[i]))

            h = heuristic(s,h_values)
            n.f = n.g+h

            if OPENd.get((n.state,n.time),float("inf")) > n.f and (n.state,n.time) not in CLOSED:
                heappush(OPEN,n)
                OPENd[(n.state,n.time)] = n.f

    # print("end")





class CTNode:
    def __init__(self,cost = None,con = None,vertexlist = None,edgelist = None, parent = None,index=None):
        self.cost = cost
        self.con = con
        self.vertexlist = vertexlist
        self.edgelist = edgelist
        self.agentindex = index
        self.parent = None
        self.children = []
        self.uuid = str(uuid.uuid1())

    def root(self):
        self.cost = 0
        self.con = {}
        self.vertexlist = []
        self.edgelist = []
        self.agentindex = {}

    def end_time(self):
        return max(len(i) for i in self.vertexlist)-1

    def branch(self,new_con):
        con = self.con.copy()
        old = len(self.con.get(new_con.state,set()))
        con[new_con.state] = self.con.get(new_con.state,set()).union(set((new_con,)))
        if old == len(con[new_con.state]):
            print(self)
            print(new_con,new_con.state)
            raise Exception("Duplicate Con")
        node = CTNode(self.cost,con,self.vertexlist.copy(),self.edgelist.copy(), self,self.agentindex.copy())
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

class Constraint:

    def __init__(self,agent,where,time,conflit):
        self.agent = agent
        self.where = where
        self.time = time
        self.state = (where,time)
        self.conflit = conflit

    def __str__(self):
        return str(self.agent)
    def __repr__(self):
        return str(self)

    def __eq__(self,o):
        return self.agent == o.agent and self.where == o.where and self.time == o.time
    def __hash__(self):
        return hash((self.agent.agent,self.where,self.time))


def extender(li,x,m):
    return (li + ([x]*(m-len(li))))

def check_conflit(node,offset=0):
    conflit = None,None,None
    solution = (node.edgelist if offset else node.vertexlist)
    m = max(len(i) for i in solution)
    # print(m)
    aslou = [extender(list(i),frozenset((i[-1],)) if offset else i[-1],m) for i in solution]
    # print([len(x) for x in aslou])
    for timestep,i in enumerate(zip(*aslou)):
        allocate={}
        for agent,j in enumerate(i):
            if j not in allocate:
                allocate[j] = agent
            else:
                conflit = ((node.agentindex[allocate[j]],node.agentindex[agent]),j,timestep+offset),allocate[j],agent
                break
        if conflit[0]:
            break
    return conflit

def transpose(v,e):
    return list(zip(*v)),[[frozenset(tuple[i] for tuple in s) for s in e]for i in range(len(list(list(e[0])[0])))]

def rstrip(li):
    to = len(li)
    while to>1 and li[to-1] == li[to-2]:
        to -= 1
    return to


def solve(G,q_node,n_node,agent):
    x = astar(G,agent.start,agent.goal,q_node.end_time(),heuristic=SIC,heuristic_precalculator=reverse_dijkstra,constraint=n_node.con,agent=agent.agent)
    if x:
        vl,el = transpose(*x)
        for idx,j in enumerate(agent.agent):
            ocost = len(q_node.vertexlist[j])
            to = rstrip(vl[idx])
            n_node.vertexlist[j],n_node.edgelist[j] = vl[idx][:to],el[idx][:to]
            n_node.cost += len(n_node.vertexlist[j])-ocost
        return True
    return False

# def print(*arg):pass
DEBUG = True
info = {"Render   ":[0.0,0,0.0],"Conflitct":[0.0,0,0.0],"Merge    ":[0.0,0,0.0],"Branch   ":[0.0,0,0.0]}
infotext = ""
start_time = 0
tree = Digraph();tree.attr(rankdir="TB")

def macbs(G,B,start,goal):
    global info,infotext,start_time,tree
    if DEBUG:
        cou = time.time()
        start_time = cou


    tree = Digraph();tree.attr(rankdir="TB")
    CM = np.zeros((len(start),len(start,)))
    OPEN = []
    root = CTNode()
    root.root()
    for i in range(len(start)):
        agent = Metaagent((i,),(start[i],),(goal[i],))
        root.agentindex[i] = agent
        x = astar(G,agent.start,agent.goal,True,heuristic=SIC,heuristic_precalculator=reverse_dijkstra,constraint={},agent=agent.agent)
        if x:
            vl,el = transpose(*x)
            root.vertexlist.append(vl[0])
            root.edgelist.append(el[0])
            root.cost += len(root.vertexlist[-1])-1
        else:
            return

    heapq.heappush(OPEN,root)
    tree.node(root.uuid,str(root),shape="box")

    if DEBUG:
        infotext += "INIT"+str(time.time()-cou)
        cou=time.time()
    akame = 0
    while OPEN:
        if DEBUG:
            cou=time.time()
        # input(":")
        # # print()
        # # print()
        # # print()
        # # print()
        # # print()
        # # for i in OPEN:
        # #     print()
        # #     print(i.__repr__())

        if DEBUG:
            info["Render   "][0] = time.time()-cou
            info["Render   "][1]+=1
            info["Render   "][2]+=info["Render   "][0]
            cou=time.time()


        q = heapq.heappop(OPEN)
        q:CTNode

        if akame%100 == 0:
            # print("render")
            # tree.render("./tree",view=False,format="png",cleanup=True)
            print(q)
            akame = 1
        else:
            akame+=1
        conflit,raw0,raw1 = check_conflit(q)
        if not conflit:
            conflit,raw0,raw1 = check_conflit(q,1)
            if not conflit:
                tree.render("./tree",view=False,format="png",cleanup=True)
                print(q.agentindex)
                return q.vertexlist,q.cost,root,q.edgelist
        if DEBUG:
            info["Conflitct"][0] = time.time()-cou
            info["Conflitct"][1]+=1
            info["Conflitct"][2]+=info["Conflitct"][0]
            cou=time.time()


        CM[raw0,raw1]+=1
        CM[raw1,raw0]+=1

        # print(conflit)
        # print(conflit[0][0],conflit[0][1],conflit[0][0].shoud_merge(conflit[0][1]))
        # print(conflit[0][0].conflit,conflit[0][1].conflit)
        if CM[np.ix_(conflit[0][0].agent,conflit[0][1].agent)].sum()>B:
            for i in q.con.copy():
                for j in q.con[i].copy():
                    if (j.conflit[0][0] in conflit[0][0] or j.conflit[0][0] in conflit[0][1]) and (j.conflit[0][1] in conflit[0][0] or j.conflit[0][1] in conflit[0][1]):
                        q.con[i].remove(j)
                if len(q.con[i]) == 0:
                    q.con.pop(i)
            new = Metaagent((*conflit[0][0].agent,*conflit[0][1].agent),(*conflit[0][0].start,*conflit[0][1].start),(*conflit[0][0].goal,*conflit[0][1].goal))
            for j in conflit[0][0].agent:
                q.agentindex[j] = new
            for j in conflit[0][1].agent:
                q.agentindex[j] = new

            if solve(G,q,q,new):
                heapq.heappush(OPEN,q)
                ouuid = q.uuid
                q.uuid = str(uuid.uuid1())
                tree.node(q.uuid,str(q),shape="box")
                tree.edge(ouuid,q.uuid,label=f"{conflit[0][0]} X {conflit[0][1]}")
            if DEBUG:
                info["Merge    "][0] = time.time()-cou
                info["Merge    "][1]+=1
                info["Merge    "][2]+=info["Merge    "][0]
                cou=time.time()
            continue

        for i in conflit[0]:
            new = q.branch(Constraint(i,conflit[1],conflit[2],conflit))
            if solve(G,q,new,i):
                heapq.heappush(OPEN,new)
                q.children.append(new)
                tree.node(new.uuid,str(new),shape="box")
                tree.edge(q.uuid,new.uuid,label=f"{conflit[0][0]} X {conflit[0][1]}")

        if DEBUG:
            info["Branch   "][0] = time.time()-cou
            info["Branch   "][1]+=1
            info["Branch   "][2]+=info["Branch   "][0]
            cou=time.time()


raw,c,root,ee = macbs(net,1000000000000,start=START,goal=END)
temp = []
se = 0
for i in raw:
    aka = []
    for x in i:
        t = str(x[0] if type(x) == tuple else x)
        if len(t) == 1:
            t+=" "
        aka.append(t)
    temp.append(aka)
    se += len(i)-1
print(f"Final Solution ({c}):",end="")
print("",*temp,sep="\n\t")

print(info)
for i in info:
    infotext+=f"\n{i}:\t{info[i][2]:.4} ||\t{info[i][2]/(info[i][1]+1e-6):.4} ||\t{info[i][0]:.4} ||\t{info[i][1]}"
infotext += f"\nTotal: {time.time()-start_time}"
print(infotext)