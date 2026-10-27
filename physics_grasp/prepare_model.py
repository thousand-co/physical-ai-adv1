"""Convert the installed CRANE+ URDF to a self-contained MuJoCo experiment."""
from pathlib import Path
import argparse
import shutil
import xml.etree.ElementTree as ET
import mujoco

ROOT = Path(__file__).resolve().parent

def prepare(urdf, description):
    assets = ROOT / 'assets'
    assets.mkdir(exist_ok=True)
    tree = ET.parse(urdf)
    robot = tree.getroot()
    ext = ET.SubElement(robot, 'mujoco')
    ET.SubElement(ext, 'compiler', discardvisual='false', meshdir='assets', fusestatic='false', balanceinertia='true')
    for mesh in robot.findall('.//mesh'):
        relative = mesh.get('filename').split('crane_plus_description/')[-1]
        source = Path(description) / relative
        dest = assets / source.name
        shutil.copyfile(source, dest)
        mesh.set('filename', dest.name)
    converted = ROOT / 'crane.urdf'
    tree.write(converted)
    model = mujoco.MjModel.from_xml_path(str(converted))
    mujoco.mj_saveLastXML(str(ROOT / 'imported.xml'), model)
    print('Imported', model.nq, 'DOFs;', model.ngeom, 'geometries')

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--urdf', required=True)
    p.add_argument('--description', required=True)
    a = p.parse_args()
    prepare(a.urdf, a.description)
