"""Preserve CRANE+ geometry; decompose concave fingers for contact physics."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import coacd

ROOT = Path(__file__).resolve().parent

def build():
    tree = ET.parse(ROOT / 'imported.xml')
    root = tree.getroot()
    root.find('compiler').set('meshdir', 'assets')
    ET.SubElement(root, 'option', timestep='0.002', integrator='implicitfast', cone='elliptic', iterations='80')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='720')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', friction='1.0 0.01 0.001', solref='0.006 1', solimp='0.95 0.99 0.001', condim='4')
    ET.SubElement(default, 'joint', armature='0.002', damping='0.3')
    assets = root.find('asset')
    for body in root.findall('.//body'):
        for geom in list(body.findall('geom')):
            if geom.get('contype') == '0':
                continue
            # Existing joint effort limits remain 1.5 Nm. No robot self contacts
            # in this first experiment; robot-table and robot-object remain on.
            geom.set('contype', '2')
            geom.set('conaffinity', '1')
            geom.set('group', '3')
            if body.get('name') not in ('crane_plus_link4', 'crane_plus_link_hand'):
                continue
            source = geom.get('mesh')
            mesh = trimesh.load(ROOT / 'assets' / (source + '.stl'), force='mesh')
            print(source, 'bounds', mesh.bounds.tolist(), flush=True)
            parts_dir = ROOT / 'assets' / source
            parts_dir.mkdir(exist_ok=True)
            cached = sorted(parts_dir.glob('part*.obj'))
            if not cached:
                coacd.set_log_level('error')
                parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=0.04, resolution=1000, mcts_iterations=100, seed=0)
                for i, (v, f) in enumerate(parts):
                    trimesh.Trimesh(v, f).export(parts_dir / f'part{i:02}.obj')
                cached = sorted(parts_dir.glob('part*.obj'))
            for i, path in enumerate(cached):
                name = source + f'_part{i}'
                ET.SubElement(assets, 'mesh', name=name, file=str(path.relative_to(ROOT / 'assets')))
                attrs = dict(geom.attrib, mesh=name, name=name)
                ET.SubElement(body, 'geom', **attrs)
            body.remove(geom)
            print('Convex parts:', len(cached), flush=True)
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='0 -0.3 1.5', dir='0 0 -1', diffuse='0.9 0.9 0.9')
    ET.SubElement(world, 'geom', name='table', type='plane', size='0.6 0.6 0.02', rgba='0.65 0.68 0.7 1', contype='1', conaffinity='3')
    cube = ET.SubElement(world, 'body', name='blue_box', pos='0.15 0.05 0.02')
    ET.SubElement(cube, 'freejoint', name='box_free')
    ET.SubElement(cube, 'geom', name='box', type='box', size='0.02 0.02 0.02', mass='0.03', rgba='0.08 0.35 0.9 1', contype='1', conaffinity='3')
    actuator = ET.SubElement(root, 'actuator')
    for j in root.findall('.//joint'):
        if not j.get('name'):
            continue
        j.set('damping', '0.3')
        j.set('frictionloss', '0.02')
        ET.SubElement(actuator, 'position', name=j.get('name'), joint=j.get('name'), kp='35', kv='1.5', ctrllimited='true', ctrlrange=j.get('range'), forcelimited='true', forcerange='-1.5 1.5')
    ET.indent(tree)
    tree.write(ROOT / 'scene.xml')

if __name__ == '__main__':
    build()
