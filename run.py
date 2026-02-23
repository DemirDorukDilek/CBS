import subprocess, sys

for ma in ["https://movingai.com/benchmarks/mapf/orz900d.map.zip"]:
    for B in [10, 100, "INF"]:
        for num in [10,15,20,25]:
            for i in range(25):
                print("RUN:",i)
                try:
                    pro = subprocess.run(["python","job.py", ma, str(num), str(B)],capture_output=True,text=True,timeout=60*11,check=True)
                except subprocess.TimeoutExpired:
                    pass
                finally:
                    print(pro.stdout)
                    print(pro.stderr)