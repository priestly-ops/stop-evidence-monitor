#!/usr/bin/env python3
"""Create disposable one-dimensional LiDAR test world from installed Gazebo diff_drive.sdf.

No ARES changes. Output is intentionally a distinct world so tests cannot
accidentally claim evidence from an unrelated Gazebo instance.
"""
from pathlib import Path
import glob
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET

WORLD = 'sem_m4'
SCAN_TOPIC = '/sem/m4_scan'

def locate_reference():
    if len(sys.argv) > 1:
        return Path(sys.argv[1])
    candidates = []
    for pattern in ('/usr/share/gz/gz-sim*/worlds/diff_drive.sdf',
                    '/usr/share/ignition/ignition-gazebo*/worlds/diff_drive.sdf',
                    '/usr/share/gz/gz-sim*/examples/worlds/diff_drive.sdf'):
        candidates.extend(glob.glob(pattern))
    candidates = [Path(p) for p in candidates if Path(p).is_file()]
    if not candidates:
        raise SystemExit('Reference SDF not located. Run: find /usr/share -name diff_drive.sdf\n'
                         'Then run: python3 scripts/m4_make_world.py /full/path/diff_drive.sdf')
    return sorted(candidates)[-1]

def main():
    source = locate_reference()
    root = ET.parse(source).getroot()
    world = root.find('world')
    if world is None:
        raise SystemExit('ABORT: no world in reference SDF')
    model = world.find("model[@name='vehicle_blue']")
    if model is None or model.find("link[@name='chassis']") is None:
        raise SystemExit('ABORT: expected vehicle_blue/chassis missing')
    if world.find("model[@name='sem_m4_wall']") is not None:
        raise SystemExit('ABORT: reference world already customized')
    world.set('name', WORLD)
    # Reduce rendering load; no need for the second moving robot.
    green = world.find("model[@name='vehicle_green']")
    if green is not None:
        world.remove(green)
    plugin = ET.SubElement(world, 'plugin', {
        'filename': 'gz-sim-sensors-system',
        'name': 'gz::sim::systems::Sensors',
    })
    ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    wall = ET.SubElement(world, 'model', {'name': 'sem_m4_wall'})
    ET.SubElement(wall, 'static').text = 'true'
    ET.SubElement(wall, 'pose').text = '8 2 0.9 0 0 0'
    link = ET.SubElement(wall, 'link', {'name': 'wall_link'})
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, {'name': kind})
        ET.SubElement(ET.SubElement(item, 'geometry'), 'box').append(
            ET.Element('size'))
        item.find('./geometry/box/size').text = '0.2 8 1.8'
    chassis = model.find("link[@name='chassis']")
    sensor = ET.SubElement(chassis, 'sensor', {'name': 'sem_m4_front_lidar', 'type': 'gpu_lidar'})
    ET.SubElement(sensor, 'pose').text = '0.7 0 0.35 0 0 0'
    ET.SubElement(sensor, 'topic').text = SCAN_TOPIC
    ET.SubElement(sensor, 'always_on').text = '1'
    ET.SubElement(sensor, 'update_rate').text = '10'
    ray = ET.SubElement(sensor, 'ray')
    scan = ET.SubElement(ray, 'scan')
    h = ET.SubElement(scan, 'horizontal')
    for name, value in [('samples','5'),('resolution','1'),('min_angle','-0.02'),('max_angle','0.02')]:
        ET.SubElement(h, name).text = value
    limits = ET.SubElement(ray, 'range')
    for name, value in [('min','0.15'),('max','12'),('resolution','0.001')]:
        ET.SubElement(limits, name).text = value
    output = Path.home() / 'stop_evidence_results/sem_m4.sdf'
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(output, encoding='unicode', xml_declaration=True)
    reloaded = ET.parse(output).getroot()
    assert reloaded.find("world/model[@name='sem_m4_wall']") is not None
    assert reloaded.find("world/model[@name='vehicle_blue']/link/sensor[@name='sem_m4_front_lidar']") is not None
    manifest = {
        'reference_file': str(source),
        'reference_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'generated_file': str(output),
        'generated_sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
        'world': WORLD, 'scan_topic': SCAN_TOPIC,
        'constraints': 'straight-line movement facing a fixed wall; no universal standstill claim',
    }
    (output.parent / 'sem_m4_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest, indent=2))
    print('\nNEXT: gz sim -s -r ' + str(output))

if __name__ == '__main__':
    main()
