import subprocess, sys

for ma in ["https://movingai.com/benchmarks/mapf/orz900d.map.zip"]:
    for B in [10, 100, "INF"]:
        for num in [10,15,20,25]:
            for i in range(25):
                print("RUN:",i)
                pro = subprocess.Popen(["python","job.py", ma, str(num), str(B)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                try:
                    stdout, stderr = pro.communicate(timeout=60*11)
                except subprocess.TimeoutExpired:
                    pro.terminate()  # SIGTERM -> job.py save_out(False) çağırır
                    try:
                        stdout, stderr = pro.communicate(timeout=15)
                    except subprocess.TimeoutExpired:
                        pro.kill()
                        stdout, stderr = pro.communicate()
                print(stdout)
                print(stderr)
