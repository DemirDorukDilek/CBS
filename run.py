import subprocess, sys
for i in range(100):
    print("RUN:",i)
    try:
        pro = subprocess.run([sys.executable,"job.py"],capture_output=True,text=True,timeout=60*11,check=True)
    except subprocess.TimeoutExpired:
        pass
    finally:
        print(pro.stdout)
        print(pro.stderr)