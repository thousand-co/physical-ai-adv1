"""Record selected reproducibility information without credentials or env dumps."""
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def command(args):
    result = subprocess.run(args, text=True, capture_output=True)
    return result.stdout.strip() if result.returncode == 0 else None


def main():
    crane = Path(os.environ.get('CRANE_WS', '/home/ubuntu/airobot_ws')).resolve()/'src/crane_plus'
    packages = {}
    for name in ['numpy', 'scipy', 'opencv-python', 'opencv-python-headless',
                 'Pillow', 'reportlab', 'PyYAML', 'mujoco', 'trimesh', 'coacd']:
        try: packages[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: pass
    debian_result = subprocess.run(['dpkg-query', '-W', '-f=${Package}=${Version}\n',
        'ros-humble-gazebo-ros-pkgs', 'ros-humble-gazebo-ros2-control',
        'ros-humble-ros2-control', 'ros-humble-ros2-controllers', 'gazebo',
        'python3-opencv', 'python3-scipy', 'ffmpeg'], text=True, capture_output=True)
    # dpkg can return partial results when an optional package is absent.
    debian = debian_result.stdout.strip()
    commit = None
    head = crane/'.git/HEAD'
    if head.is_file():
        value = head.read_text().strip()
        if value.startswith('ref: '):
            ref = value[5:]
            loose = crane/'.git'/ref
            if loose.is_file(): commit = loose.read_text().strip()
            else:
                packed = crane/'.git/packed-refs'
                if packed.is_file():
                    for line in packed.read_text().splitlines():
                        if line.endswith(' '+ref): commit = line.split()[0]
        else: commit = value
    report = dict(description='Observed environment; not a portable container or complete dependency lock.',
        operating_system=platform.freedesktop_os_release().get('PRETTY_NAME'),
        python=platform.python_version(), ros_distribution='humble', python_packages=packages,
        debian_packages=debian.splitlines() if debian else [],
        external_crane=dict(commit=commit,
            note='HEAD read from local checkout; local modifications were not audited.'))
    path = ROOT/'environment/observed.json'
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print('Recorded environment:', path)


if __name__ == '__main__': main()
