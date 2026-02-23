import requests, zipfile, io, os, shutil
import numpy as np
import networkx as nx
import heapq
from itertools import product
from graphviz import Digraph
import uuid
import time
import imageio.v3 as iio
import pathlib
import json
import sys

INF = float("inf")

def download_map(url,dst:pathlib.Path="mapp",senario = 1,flush=True):
    req = False
    if flush and os.path.exists(dst):
        shutil.rmtree(dst)
    if not os.path.exists(dst):
        os.makedirs(dst, exist_ok=True)
        req = True
    if req:
        r = requests.get(url)
        z = zipfile.ZipFile(io.BytesIO(r.content))
        z.extractall(dst)
        r = requests.get(url[:-4]+"-scen-even.zip")
        z = zipfile.ZipFile(io.BytesIO(r.content))
        z.extractall(dst)

    edges = []
    with open(dst/dst.with_suffix(".map"),"r") as f:
        raw = f.readlines()
        MULTIPLIER = 10**len(raw[2].split(" ")[-1].strip())
        SEN.width = int(raw[2].split(" ")[-1].strip())
        SEN.heigt = int(raw[1].split(" ")[-1].strip())
        raw = raw[4:]
    for row,line in enumerate(raw):
        for col,ch in enumerate(line):
            if ch == ".":
                for hm,cm in [(-1,0),(1,0),(0,-1),(0,1)]:
                    if 0<=row+hm and row+hm<len(raw) and 0<=col+cm and col+cm<len(line):
                        if raw[row+hm][col+cm] == ".":
                            edges.append(((MULTIPLIER*row+col),(MULTIPLIER*(row+hm)+(col+cm))))
    G = nx.DiGraph()
    G.add_edges_from(edges)

    with open(dst/"scen-even"/(str(dst)+f"-even-{senario}.scen"),"r") as f:
        raw = f.readlines()[1:]

    bucket = {}
    alll = []
    for i in raw:
        ibucket,_,__,___,sx,sy,gx,gy,____ = i.split("\t")
        x = (int(sx) + MULTIPLIER*int(sy),int(gx) + MULTIPLIER*int(gy))
        bucket[int(ibucket)] = bucket.get(int(ibucket),[]) + [x]
        alll.append(x)
    return G,alll,bucket

def sample(alll,idx):
    s = []
    g = []
    for i in idx:
        s.append(alll[i][0])
        g.append(alll[i][1])
    return s,g

def random_sampler(alll,size):
    a = np.arange(len(alll))
    np.random.shuffle(a)
    return a[:size]

def set_sen():
    SEN.G,alll,_ = download_map(SEN.URL,SEN.dst,SEN.no,SEN.flush)
    SEN.start, SEN.goal = sample(alll,random_sampler(alll,SEN.num))

def jsonloader(path):
    with open(path,"r") as f:
        x = json.load(f)
    SEN.start,SEN.goal = x["start"],x["goal"]
    SEN.G = nx.DiGraph()
    SEN.G.add_edges_from(x["edges"])

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
    if glb.DIJKSTRA_CACHE != None:
        return glb.DIJKSTRA_CACHE

    rg = graph.reverse()
    h_values = {}
    for goal in goals:
        h_values[goal] = nx.single_source_shortest_path_length(rg,goal)
    glb.DIJKSTRA_CACHE = h_values
    return h_values
def SIC(state,h_values,goal):
    return sum([h_values[aidx][x] if x in h_values[aidx] else INF for aidx,x in zip(goal,state)])

def astar(G,start,goal,end_time=None,heuristic=None,heuristic_precalculator=None,constraint=None,agent=None):
    # Local variable cache
    # print(constraint)
    # Tim.set("L I")
    heappush = heapq.heappush
    heappop = heapq.heappop
    adj = G.adj
    glb.infonode["llc"]+=1
    OPEN = []
    CLOSED = {}

    if heuristic_precalculator:
        h_values = heuristic_precalculator(G,goal)

    node = Anode(start,heuristic(start,h_values,goal),0,None)
    OPENd = {(start,node.time):node.f}
    heappush(OPEN,node)

    # Tim.sep("L I")

    while OPEN:
        # Tim.set("L L")
        q = heappop(OPEN)
        OPENd.pop((q.state,q.time))
        glb.infonode["lle"]+=1

        if is_goal(q.state,goal):
            # print("end")
            if end_time==None or q.time >= end_time:
                return get_path(q)
        if (q.state,q.time) in CLOSED: continue
        CLOSED[(q.state,q.time)] = q.f
        # print(q, q.parent)
        # Tim.sep("L L")

        for s in get_successor(q.state,adj):
            # Tim.set("L S1")

            if (s,q.time+1) in CLOSED: continue
            if not MAPF_valid(q.state,s):
                # print(q.state,s, "AAAAAAAAAAAAAAA")
                continue
            invalid = False

            t_next = q.time+1
            for idx in range(len(agent)):
                s_idx = s[idx]
                bucket = constraint.get((s_idx,t_next),None)
                if bucket:
                    for cagent in bucket:
                        if agent[idx] in cagent.agent:
                            invalid = True
                            break
                    if invalid:
                        break
                bucket = constraint.get((frozenset((q.state[idx],s_idx)),t_next),None)
                if bucket:
                    for cagent in bucket:
                        if agent[idx] in cagent.agent:
                            invalid = True
                            break
                    if invalid:
                        break
            if invalid:
                continue
            # Tim.sep("L S1")
            # Tim.set("L S2")

            n = Anode(s,0,q.g,q,q.time+1)

            n.g += sum(1 for qs_i,si,gi in zip(q.state, s, goal) if not (qs_i == gi and si == gi))

            h = heuristic(s,h_values,goal)
            n.f = n.g+h
            # Tim.sep("L S2")
            # Tim.set("L S3")

            keyi = (n.state,n.time)
            if OPENd.get(keyi,INF) > n.f and keyi not in CLOSED:
                glb.infonode["llc"]+=1
                heappush(OPEN,n)
                OPENd[(n.state,n.time)] = n.f
            # Tim.sep("L S3")

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
        return "M"+"-".join(map(str,self.agent))

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
    Tim.set("Low real")
    x = astar(G,agent.start,agent.goal,q_node.end_time(),heuristic=SIC,heuristic_precalculator=reverse_dijkstra,constraint=n_node.con,agent=agent.agent)
    Tim.sep("Low real")
    Tim.set("Low Trasform")
    if x:
        vl,el = transpose(*x)
        for idx,j in enumerate(agent.agent):
            ocost = len(q_node.vertexlist[j])
            to = rstrip(vl[idx])
            n_node.vertexlist[j],n_node.edgelist[j] = vl[idx][:to],el[idx][:to]
            n_node.cost += len(n_node.vertexlist[j])-ocost
        Tim.sep("Low Trasform")
        return True
    Tim.sep("Low Trasform")
    return False



class Tim:
    tim = {}
    def set(label):
        Tim.tim[label] = time.time()
    def sep(label):
        if label not in glb.infotime:
            glb.infotime[label] = [0,0,0]
        glb.infotime[label][0] = time.time()-Tim.tim[label]
        glb.infotime[label][1]+=1
        glb.infotime[label][2]+=glb.infotime[label][0]

def grid_maker(sol):
    with open(SEN.dst/SEN.dst.with_suffix(".map"),"r") as f:
        raw = f.readlines()
        MULTIPLIER = 10**len(raw[2].split(" ")[-1].strip())
        SEN.width = int(raw[2].split(" ")[-1].strip())
        SEN.heigt = int(raw[1].split(" ")[-1].strip())
        raw = raw[4:]
    np.zeros((SEN.heigt*10,SEN.width*10),dtype=np.uint8)
    arr = (np.array([list(s) for s in raw],dtype="U1") == ".").astype(np.uint8)
    arr[arr == 1] = 255
    temp = arr.copy()
    mmm = 10**len(str(SEN.width))
    for i in sol:
        for l,j in enumerate(i):
            row = j//mmm
            col = j%mmm
            temp[row,col] = 192 if l == 0 else (96 if l == len(i)-1 else 128)

    r,g,b = arr.copy(),arr.copy(),arr.copy()
    g[temp==96] = 0
    b[temp==96] = 0
    r[temp==192] = 0
    b[temp==192] = 0
    r[temp==128] = 0
    g[temp==128] = 0
    arr = np.stack((r,g,b),axis=-1).astype(np.uint8)
    # arr = np.repeat(np.repeat(arr, 10, axis=0),10,axis=1)
    iio.imwrite("res.png",arr)



def macbs(G,B,start,goal):
    glb.start_time = time.time()
    Tim.set("INIT")

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
    if glb.render:
        glb.tree.node(root.uuid,str(root),shape="box")

    if glb.DEBUG:
        akame = 0

    Tim.sep("INIT")
    while OPEN:
        Tim.set("Render")
        # input(":")
        # # print()
        # # print()
        # # print()
        # # print()
        # # print()
        # # for i in OPEN:
        # #     print()
        # #     print(i.__repr__())
        # glb.tree.render("./tree",view=False,format="png",cleanup=True)

        Tim.sep("Render")


        q = heapq.heappop(OPEN)
        q:CTNode
        glb.infonode["hle"]+=1

        if glb.DEBUG:
            if akame%glb.perloop == 0:
                # glb.tree.render("./tree",view=False,format="png",cleanup=True)
                print(glb.infonode["hle"],q.cost,len(OPEN))
                # grid_maker(q.vertexlist)
                akame = 1
            else:akame+=1

        Tim.set("Conflitct")
        conflit,raw0,raw1 = check_conflit(q)
        if not conflit:
            conflit,raw0,raw1 = check_conflit(q,1)
            if not conflit:
                # glb.tree.render("./glb.tree",view=False,format="png",cleanup=True)
                if glb.DEBUG: print(q.agentindex)
                return q.vertexlist,q.cost,root.cost

        Tim.sep("Conflitct")

        Tim.set("Merge")
        CM[raw0,raw1]+=1
        CM[raw1,raw0]+=1

        # print(conflit)
        # print(conflit[0][0],conflit[0][1],conflit[0][0].shoud_merge(conflit[0][1]))
        # print(conflit[0][0].conflit,conflit[0][1].conflit)
        if CM[np.ix_(conflit[0][0].agent,conflit[0][1].agent)].sum()>B:
            # Tim.set("M con")
            for i in q.con.copy():
                for j in q.con[i].copy():
                    if (j.conflit[0][0] in conflit[0][0] or j.conflit[0][0] in conflit[0][1]) and (j.conflit[0][1] in conflit[0][0] or j.conflit[0][1] in conflit[0][1]):
                        q.con[i].remove(j)
                if len(q.con[i]) == 0:
                    q.con.pop(i)
            # Tim.sep("M con")
            # Tim.set("M agent")
            new = Metaagent((*conflit[0][0].agent,*conflit[0][1].agent),(*conflit[0][0].start,*conflit[0][1].start),(*conflit[0][0].goal,*conflit[0][1].goal))
            for j in conflit[0][0].agent:
                q.agentindex[j] = new
            for j in conflit[0][1].agent:
                q.agentindex[j] = new
            # Tim.sep("M agent")
            # Tim.set("M solve")

            if solve(G,q,q,new):
                heapq.heappush(OPEN,q)
                ouuid = q.uuid
                q.uuid = str(uuid.uuid1())
                if glb.render:
                    glb.tree.node(q.uuid,str(q),shape="box")
                    glb.tree.edge(ouuid,q.uuid,label=f"{conflit[0][0]} X {conflit[0][1]}")
            # Tim.sep("M solve")

            Tim.sep("Merge")
            glb.infonode["merge"]+=1
            continue

        Tim.set("Branch")
        for i in conflit[0]:
            Tim.set("Copy")
            new = q.branch(Constraint(i,conflit[1],conflit[2],conflit))
            Tim.sep("Copy")
            glb.infonode["hlc"]+=1
            if solve(G,q,new,i):
                heapq.heappush(OPEN,new)
                q.children.append(new)
                if glb.render:
                    glb.tree.node(new.uuid,str(new),shape="box")
                    glb.tree.edge(q.uuid,new.uuid,label=f"{conflit[0][0]} X {conflit[0][1]}")
        Tim.sep("Branch")


def log(sol,cost,lb):
    if sol:
        temp = len(str(list(SEN.G.nodes)[0]))
        print(f"\nFinal Solution ({cost}) ({lb}):\n=====================================================================================",end="")
        print("",*[[str(x)+(" "*(temp-len(str(x)))) for x in i] for i in sol],sep="\n")


    print("\nTimes:\n=====================================================================================")
    temp = max(map(len,glb.infotime.keys()))
    [print(f"{i+(" "*(temp-len(i)))}:\t{glb.infotime[i][2]:.4f} ||\t{glb.infotime[i][2]/(glb.infotime[i][1]+1e-6):.4f} ||\t{glb.infotime[i][0]:.4f} ||\t{glb.infotime[i][1]}") for i in glb.infotime]
    print(f"\nTotal: {glb.end_time-glb.start_time}")

    print("\nNode:\n=====================================================================================")
    print("")
    [print(f"{i}: {glb.infonode[i]}") for i in glb.infonode]
    print(f"\nllE node per low level: {glb.infonode["lle"]/glb.infonode["hle"]+len(SEN.start)-1:.6f}")
    print(f"llC node per low level: {glb.infonode["llc"]/glb.infonode["hle"]+len(SEN.start)-1:.6f}")

def pretest():
    if SEN.URL.endswith(".json"):
        jsonloader(SEN.URL)
        SEN.grid=False
    else:
        SEN.dst = pathlib.Path(pathlib.Path(SEN.URL).name).with_suffix("").with_suffix("")
        set_sen()

def save_out(stat):
    if not glb.logSave:return
    if not os.path.exists(glb.logsave):
        with open(glb.logsave,"w",encoding="utf-8") as f:
            f.write("[]")
    with open(glb.logsave,"r",encoding="utf-8") as f:
        arr = json.load(f)
    arr.append({"logversion":1,"stat":stat,"param":{"B":glb.B,"URL":SEN.URL,"num":SEN.num,"no":SEN.no,"render":glb.render,"debug":glb.DEBUG},"start":SEN.start,"goal":SEN.goal,"time":glb.infotime,"node":glb.infonode,"totaltime":glb.end_time-glb.start_time,"cost":glb.cost})
    with open(glb.logsave,"w",encoding="utf-8") as f:
        json.dump(arr,f)

def test():
    reverse_dijkstra(SEN.G,SEN.goal)
    sol,cost,lowerbound = macbs(SEN.G,glb.B,start=SEN.start,goal=SEN.goal)
    glb.cost = cost
    glb.end_time = time.time()
    log(sol,cost,lowerbound)



class glb:
    infotime = {}
    infonode = {"hle":0,"merge":0,"lle":0,"llc":0,"hlc":1}
    tree = Digraph(); tree.attr(rankdir="TB")
    start_time = 0
    end_time = None
    DIJKSTRA_CACHE  = None
    cost = None

    logsave = "save.json"
    render = True
    DEBUG = True
    perloop = 50
    logSave = True

    B = INF if sys.argv[3] else int(sys.argv[3])

class SEN:
    G = None;start = None;goal = None;dst = pathlib.Path("mapp");grid=True
    no = 1
    flush = False

    # URL = "https://movingai.com/benchmarks/mapf/brc202d.map.zip"
    URL = sys.argv[1]
    # URL = "graph.json"
    num = int(sys.argv[2])

pretest()
try:
    print(SEN.start)
    print(SEN.goal)
    test()
    save_out(True)
except BaseException as e:
    glb.end_time = time.time()
    log(False,False,False)
    # glb.tree.render("tree",view=False,format="png",cleanup=True)
    save_out(False)
    raise e