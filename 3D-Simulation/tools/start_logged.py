"""Start one demo component with a PID and a persistent log file."""
import argparse
import os
from pathlib import Path
import subprocess
import time

parser = argparse.ArgumentParser()
parser.add_argument('component', choices=['fleet', 'bridge', 'headless', 'federated', 'backend'])
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
logs = root/'demo_logs'
logs.mkdir(exist_ok=True)
pidfile = logs/f'{args.component}.pid'
if pidfile.exists():
    try:
        os.kill(int(pidfile.read_text()), 0)
    except (OSError, ValueError):
        pass
    else:
        raise SystemExit(f'{args.component} already running; stop it before starting another copy')
logfile = logs/f'{args.component}.log'
if logfile.exists():
    logfile.rename(logs/f'{args.component}-{time.time_ns()}.log')
with logfile.open('w') as stream:
    process = subprocess.Popen(['bash', str(root/'tools/demo.sh'), args.component],
                               stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
pidfile.write_text(str(process.pid))
print(args.component, process.pid, logs/f'{args.component}.log')
