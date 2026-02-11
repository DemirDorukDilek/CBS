import pathlib as p

x ="https://movingai.com/benchmarks/mapf/brc202d.map.zip"

y= p.Path(p.Path(x).name).with_suffix("")
print(y)