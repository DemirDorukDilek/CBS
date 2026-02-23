import json

with open("save.json") as f:
    o = json.load(f)
    

kk = set(map(lambda x:(x["param"]["B"],x["param"]["num"]), o))
len(list(filter(lambda x:x["param"]["B"]==float("inf") and x["param"]["num"]==25)))

print