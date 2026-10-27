"""Prepare a sorting scene; the belt is a scripted feeder, grasping uses physics."""
import copy
import json
import xml.etree.ElementTree as ET
from prepare_physical_delivery import PKG, make_world, make_robot, box_geometry


def prepare():
    make_world()
    make_robot()
    tree = ET.parse(PKG/'worlds/physical_delivery.world')
    world = tree.getroot().find('world')
    world.set('name', 'conveyor_sorting')
    template = world.find("model[@name='target_blue_00']")
    world.remove(template)
    world.remove(world.find("model[@name='distractor_red']"))
    colors = [('blue', '.122 .467 .706 1'), ('red', '.839 .153 .157 1'),
              ('green', '.173 .627 .173 1'), ('blue', '.122 .467 .706 1')]
    items = []
    for i, (color, rgba) in enumerate(colors):
        model = copy.deepcopy(template)
        name = f'parcel_{i:02d}'  # Color is deliberately absent from model names.
        model.set('name', name)
        model.find('pose').text = f'{.55+i*.07} -.025 .071 0 0 0'
        for field in ['ambient', 'diffuse']:
            model.find('link/visual/material/'+field).text = rgba
        model.find('link/sensor/plugin').set('name', f'contacts_{i}')
        world.append(model)
        items.append(dict(name=name, expected_color=color, start_x=.55+i*.07))
    reject = copy.deepcopy(world.find("model[@name='delivery_tray']"))
    reject.set('name', 'reject_tray')
    reject.find('pose').text = '.075 .075 .05 0 0 0'
    for material in reject.findall('link/visual/material'):
        for child in material: child.text = '.4 .4 .45 1'
    world.append(reject)
    # The visual belt is flush with the table. No elevated collision step at pick.
    belt = ET.SubElement(world, 'model', name='sorting_conveyor')
    ET.SubElement(belt, 'static').text = 'true'
    ET.SubElement(belt, 'pose').text = '.48 -.025 .0502 0 0 0'
    link = ET.SubElement(belt, 'link', name='belt')
    visual = ET.SubElement(link, 'visual', name='belt_surface')
    box_geometry(visual, [.59, .065, .0002])
    material = ET.SubElement(visual, 'material')
    for field in ['ambient', 'diffuse']: ET.SubElement(material, field).text = '.12 .13 .15 1'
    for i in range(12):
        visual = ET.SubElement(link, 'visual', name=f'stripe_{i}')
        ET.SubElement(visual, 'pose').text = f'{-.27+i*.05} 0 .0002 0 0 0'
        box_geometry(visual, [.002, .06, .0002])
        material = ET.SubElement(visual, 'material')
        for field in ['ambient', 'diffuse']: ET.SubElement(material, field).text = '.5 .5 .5 1'
    ET.indent(tree)
    tree.write(PKG/'worlds/conveyor_sorting.world')
    (PKG/'config/sorting_sequence.json').write_text(json.dumps(items, indent=2))


if __name__ == '__main__':
    prepare()
