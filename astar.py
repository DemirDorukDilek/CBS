import numpy as np
import networkx as nx
np.genfromtxt("./mapcik.txt")

net - nx.read_edgelist("./mapcik.txt", edgetype=int)