"""Run physical acceptance scenarios sequentially, stopping at the first failure."""
import subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[2]
for name in ('A','B','C'):
    print('Starting physical scenario '+name,flush=True)
    subprocess.run(['python3',str(root/'Webots/tests/run_coordination.py'),name],check=True)
print('All physical coordination scenarios passed',flush=True)
