"""Start container simulation and automatically attach the native macOS viewer."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
RUNTIME=ROOT/'.runtime'

def main():
    RUNTIME.mkdir(exist_ok=True)
    ready=RUNTIME/'ready.json'
    previous=ready.read_text() if ready.exists() else ''
    # Stop only this project's previous simulation before replacing its model.
    subprocess.run([sys.executable,str(ROOT/'scripts/robot.py'),'sim-stop'],cwd=ROOT,check=False)
    time.sleep(1)
    sim=subprocess.Popen([sys.executable,str(ROOT/'scripts/robot.py'),'sim',*sys.argv[1:]],cwd=ROOT)
    try:
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            if sim.poll() is not None:return sim.returncode
            try:
                current=ready.read_text()
                if current!=previous and (RUNTIME/'viewer.npz').exists():
                    json.loads(current)
                    break
            except (OSError,ValueError):pass
            time.sleep(.2)
        else:
            print('Simulation has not produced a ready snapshot; inspect its output.',file=sys.stderr)
            return sim.wait()
        if sys.platform=='darwin':
            # The viewer belongs to this launch/session and follows this model.
            with (RUNTIME/f'native-view-{json.loads(current)["session"]}.log').open('w') as log:
                viewer=subprocess.Popen(['pixi','run','-e','viewer','view-native'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
            (RUNTIME/'native-view.pid').write_text(str(viewer.pid))
            print('Native MuJoCo viewer started; dashboard: http://localhost:8890',flush=True)
        return sim.wait()
    except KeyboardInterrupt:
        sim.send_signal(2)
        return sim.wait()

if __name__=='__main__':raise SystemExit(main())
