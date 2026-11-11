import os
import signal
from pathlib import Path
for entry in Path('/proc').iterdir():
 if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
 try:
  command=(entry/'cmdline').read_bytes().replace(b'\0',b' ')
  if b'ros2 launch base101_mujoco' in command:os.kill(int(entry.name),signal.SIGINT)
 except (OSError,ProcessLookupError):pass
