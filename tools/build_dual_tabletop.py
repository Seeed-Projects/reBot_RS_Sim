"""Build a self-contained dual RS tabletop MJCF from the maintained arm model."""
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

MODELS = Path(__file__).resolve().parents[1] / 'rebotarm_mujoco_rs' / 'models'


def build():
    root = ET.Element('mujoco', model='rebot_dual_tabletop')
    ET.SubElement(root, 'compiler', angle='radian', meshdir='meshes/')
    ET.SubElement(root, 'option', timestep='0.001', iterations='80', solver='Newton', integrator='Euler')
    ET.SubElement(root, 'statistic', center='0.25 0 0.9', extent='1.6')
    asset = ET.SubElement(root, 'asset')
    world = ET.SubElement(root, 'worldbody')
    contact = ET.SubElement(root, 'contact')
    equality = ET.SubElement(root, 'equality')
    actuator = ET.SubElement(root, 'actuator')
    sections = dict(worldbody=world, contact=contact, equality=equality, actuator=actuator)
    source = ET.parse(MODELS / 'rs_arm.xml').getroot()
    asset.extend(deepcopy(list(source.find('asset'))))
    for prefix, y in [('left_', '-0.34'), ('right_', '0.34')]:
        arm = deepcopy(source)
        for element in arm.iter():
            for attr in ('name', 'joint', 'joint1', 'joint2', 'body1', 'body2'):
                if attr in element.attrib:
                    element.set(attr, prefix + element.get(attr))
        arm.find('worldbody/body').set('pos', f'0 {y} 0.90')
        for name, section in sections.items():
            for child in arm.find(name):
                section.append(child)
        for a, b in [('base_link', 'link1'), ('link1', 'link2'), ('link2', 'link3'),
                     ('link2', 'link5'), ('link3', 'link4'), ('link3', 'link5'),
                     ('link4', 'link5'), ('link5', 'link6')]:
            ET.SubElement(contact, 'exclude', body1=prefix+a, body2=prefix+b)

    colors = {
        'oak': '.64 .43 .25 1', 'oak_light': '.78 .60 .39 1', 'ivory': '.91 .89 .82 1',
        'wall': '.54 .66 .68 1', 'trim': '.91 .91 .85 1', 'floor_mat': '.62 .49 .36 1',
        'rug': '.77 .73 .61 1', 'stripe': '.72 .37 .19 1', 'basket': '.53 .37 .20 1',
        'leaf': '.17 .32 .13 1', 'terracotta': '.59 .29 .16 1',
        'apple': '.72 .045 .035 1', 'lime': '.40 .64 .055 1', 'egg': '.91 .79 .58 1',
        'spool': '.12 .31 .47 1', 'soil': '.12 .08 .045 1', 'art': '.31 .45 .40 1',
        'ceramic': '.74 .85 .79 1', 'steel': '.66 .69 .68 1',
    }
    for name, rgba in colors.items():
        ET.SubElement(asset, 'material', name=name, rgba=rgba, roughness='.85')

    def geom(parent, name, shape, pos, size, material, **attrs):
        return ET.SubElement(parent, 'geom', name=name, type=shape, pos=pos, size=size,
                             material=material, **attrs)

    ET.SubElement(world, 'light', pos='1 -1 3', dir='-.2 .2 -1', directional='true')
    ET.SubElement(world, 'camera', name='overhead_rgb', pos='.3 0 2.3', xyaxes='0 1 0 -1 0 0', fovy='57')
    geom(world, 'floor', 'plane', '0 0 0', '4 4 .02', 'floor_mat')
    geom(world, 'home_room_anchor', 'sphere', '0 0 0', '.001', 'oak', contype='0', conaffinity='0')
    geom(world, 'side_wall', 'box', '1.15 2.25 1.30', '1.90 .035 1.3', 'ivory')
    geom(world, 'side_skirting', 'box', '1.15 2.20 .055', '1.90 .012 .055', 'trim')
    # Conservative furniture collision proxies. Browser details are batched locally.
    geom(world, 'sideboard_collision', 'box', '-.39 1.48 .42', '.24 .58 .42', 'oak', group='3', rgba='0 0 0 0')
    geom(world, 'bookcase_collision', 'box', '-.47 -1.65 .62', '.18 .40 .62', 'oak', group='3', rgba='0 0 0 0')
    # Room framing stays outside the workspace; all dimensions are metres.
    geom(world, 'back_wall', 'box', '-.72 0 1.30', '.035 2.6 1.3', 'wall')
    geom(world, 'skirting', 'box', '-.675 0 .055', '.012 2.6 .055', 'trim')
    geom(world, 'rug', 'box', '.33 0 .004', '.95 1.04 .004', 'rug')
    for y in [-.88, -.77, .77, .88]:
        geom(world, f'rug_stripe_{y}', 'box', f'.33 {y} .009', '.95 .025 .001', 'stripe', contype='0', conaffinity='0')
    # Window and sill; thin decorative elements do not create task contacts.
    geom(world, 'window_recess', 'box', '-.674 .1 1.40', '.012 .76 .48', 'art')
    geom(world, 'window_glass', 'box', '-.653 .1 1.40', '.008 .70 .42', 'ivory')
    for y in [-.64, .1, .84]:
        geom(world, f'window_upright_{y}', 'box', f'-.63 {y} 1.40', '.025 .018 .45', 'trim')
    for z in [.95, 1.40, 1.85]:
        geom(world, f'window_cross_{z}', 'box', f'-.63 .1 {z}', '.025 .76 .018', 'trim')
    geom(world, 'window_sill', 'box', '-.60 .1 .94', '.10 .84 .025', 'trim')
    geom(world, 'picture_frame', 'box', '-.66 -1.08 1.40', '.025 .19 .25', 'oak')
    geom(world, 'picture_paper', 'box', '-.63 -1.08 1.40', '.006 .16 .22', 'ivory')
    geom(world, 'picture_art', 'ellipsoid', '-.62 -1.08 1.40', '.008 .105 .14', 'art', contype='0', conaffinity='0')
    table = ET.SubElement(world, 'body', name='task_table')
    geom(table, 'tabletop', 'box', '.455 0 .815', '.325 .68 .025', 'oak_light', friction='1.8 .02 .002')
    # Raised mounting beam gives the RS wrists vertical access to the spoon span.
    geom(table, 'robot_mounting_rail', 'box', '-.035 0 .795', '.165 .68 .105', 'ivory')
    for x in [-.13, .71]:
        for y in [-.60, .60]:
            # Legs reach the underside of the top and overlap the apron joints.
            geom(table, f'leg_{x}_{y}', 'box', f'{x} {y} .401', '.032 .032 .389', 'oak')
            geom(table, f'table_foot_{x}_{y}', 'box', f'{x} {y} .010', '.033 .033 .010', 'art')
    for y in [-.60, .60]:
        geom(table, f'table_apron_long_{y}', 'box', f'.29 {y} .746', '.42 .018 .044', 'oak')
        geom(table, f'table_stretcher_long_{y}', 'box', f'.29 {y} .25', '.42 .014 .023', 'oak')
    for x in [-.13, .71]:
        geom(table, f'table_apron_end_{x}', 'box', f'{x} 0 .746', '.018 .60 .044', 'oak')
        geom(table, f'table_stretcher_end_{x}', 'box', f'{x} 0 .25', '.014 .60 .023', 'oak')
    # Steel cleats under the worktop tie it into the supporting frame.
    for x in [.17, .66]:
        for y in [-.55, .55]:
            geom(table, f'table_cleat_{x}_{y}', 'box', f'{x} {y} .786', '.045 .035 .004', 'steel')
    # Containers belong to task_table so the grasp state machine can verify support.
    geom(table, 'tray_bottom', 'box', '.40 -.34 .844', '.12 .145 .004', 'ivory')
    for y in [-.49, -.19]:
        geom(table, f'tray_rail_{y}', 'box', f'.40 {y} .850', '.12 .005 .010', 'ivory')
    for x in [.275, .525]:
        geom(table, f'tray_end_{x}', 'box', f'{x} -.34 .850', '.005 .15 .010', 'ivory')
    geom(table, 'basket_bottom', 'box', '.40 .34 .844', '.12 .145 .004', 'basket')
    # Open lattice basket with collision walls. Visual reeds are added by the renderer.
    for y in [.19, .49]:
        geom(table, f'basket_wall_y_{y}', 'box', f'.40 {y} .868', '.12 .004 .028', 'basket', rgba='.53 .37 .20 0', group='3')
    for x in [.275, .525]:
        geom(table, f'basket_wall_x_{x}', 'box', f'{x} .34 .868', '.004 .15 .028', 'basket', rgba='.53 .37 .20 0', group='3')
    # Four objects use broad flat collision soles to avoid rolling before grasping.
    objects = [('egg', '.21 -.46 .864', 'ellipsoid', '.018 .024 .024'),
               ('spool', '.26 -.18 .862', 'cylinder', '.019 .022'),
               ('apple', '.21 .46 .863', 'ellipsoid', '.023 .023 .023'),
               ('lime', '.20 .18 .861', 'ellipsoid', '.021 .018 .021')]
    for name, pos, shape, size in objects:
        body = ET.SubElement(world, 'body', name=name, pos=pos)
        ET.SubElement(body, 'freejoint', name=name+'_free')
        geom(body, name+'_visual', shape, '0 0 0', size, name, contype='0', conaffinity='0', mass='0')
        geom(body, name+'_collision', 'box', '0 0 0', '.018 .016 .020', name,
             rgba='0 0 0 0', group='3', mass='.04', friction='3.5 .08 .01', solref='.002 1', solimp='.99 .995 .0005')
        if name in ('apple', 'lime'):
            geom(body, name+'_stem', 'cylinder', '0 0 .025', '.002 .006', 'oak', contype='0', conaffinity='0', mass='0')
    cup = ET.SubElement(world, 'body', name='coop_cup', pos='.29 -.07 .884')
    ET.SubElement(cup, 'freejoint', name='coop_cup_free')
    geom(cup, 'cup_visual', 'cylinder', '0 0 0', '.037 .042', 'ceramic', contype='0', conaffinity='0', mass='0')
    geom(cup, 'cup_bottom_collision', 'cylinder', '0 0 -.039', '.034 .003', 'ceramic',
         group='3', rgba='0 0 0 0', mass='.085', friction='1.6 .02 .002')
    # A compound ring preserves the opening; a single convex mesh would seal it.
    for i in range(24):
        angle = 2 * math.pi * i / 24
        geom(cup, f'cup_wall_{i}', 'box', f'{.035*math.cos(angle):.6f} {.035*math.sin(angle):.6f} 0',
             '.003 .0052 .042', 'ceramic', euler=f'0 0 {angle}', group='3', rgba='0 0 0 0',
             mass='.002', friction='1.4 .02 .002', solref='.004 1')
    def handle_point(t):
        s = 1-t
        return [0, s**3*(-.036)+3*s*s*t*(-.086)+3*s*t*t*(-.086)+t**3*(-.036),
                s**3*.026+3*s*s*t*.047-3*s*t*t*.047-t**3*.026]
    for i in range(20):
        points = handle_point(i/20) + handle_point((i+1)/20)
        ET.SubElement(cup, 'geom', name=f'cup_handle_{i}', type='capsule', size='.0035',
                      fromto=' '.join(map(str, points)), material='ceramic', group='3',
                      rgba='0 0 0 0', mass='.001')
    spoon = ET.SubElement(world, 'body', name='coop_spoon', pos='.35 .085 .914', quat='.965925826 0 .258819045 0')
    ET.SubElement(spoon, 'freejoint', name='coop_spoon_free')
    geom(spoon, 'spoon_visual', 'box', '0 0 0', '.062 .006 .002', 'steel', contype='0', conaffinity='0', mass='0')
    geom(spoon, 'spoon_handle_collision', 'box', '.017 0 0', '.052 .0037 .0014', 'steel',
         group='3', rgba='0 0 0 0', mass='.008', friction='1.5 .02 .002', solref='.004 1')
    geom(spoon, 'spoon_bowl_collision', 'ellipsoid', '.094 0 -.001', '.020 .013 .003', 'steel',
         group='3', rgba='0 0 0 0', mass='.017', friction='1.5 .02 .002', solref='.004 1')
    for x, top in [(.326, .9255), (.4314, .8635)]:
        half_height = (top - .84) / 2
        geom(table, f'utensil_rest_{x}', 'box', f'{x} .085 {.84+half_height}',
             f'.006 .018 {half_height}', 'oak')
    # A small end stop keeps the inclined spoon from sliding off its two rests.
    geom(table, 'utensil_rest_stop', 'box', '.452 .085 .854', '.003 .018 .014', 'oak', friction='2 .02 .002')
    for item, side in [('cup', 'left'), ('spoon', 'right')]:
        ET.SubElement(equality, 'weld', name=f'{item}_grasp', body1=f'{side}_gripper_end',
                      body2=f'coop_{item}', active='false', solref='.012 1',
                      solimp='.999 .999 .0001', torquescale='1')
        # The guided grasp weld handles finger attachment; object/table/container
        # contacts stay enabled throughout the demonstration.
        # Spoon fingers keep real contacts; the weld only stabilizes the verified
        # bilateral grasp after closure, so the jaws cannot close through it.
        excluded = ['gripper_end', 'gripper_coupler']
        if item == 'cup':
            excluded += ['gripper_left', 'gripper_right']
        for body in excluded:
            ET.SubElement(contact, 'exclude', body1=f'{side}_{body}', body2=f'coop_{item}')
    geom(world, 'plant_pot', 'cylinder', '-.28 -.96 .19', '.13 .19', 'terracotta')
    geom(world, 'plant_soil', 'cylinder', '-.28 -.96 .383', '.118 .006', 'soil')
    for i in range(9):
        angle = i * 2.4
        x, y = -.28 + .10*math.cos(angle), -.96 + .10*math.sin(angle)
        geom(world, f'leaf_{i}', 'ellipsoid', f'{x:.3f} {y:.3f} {(.56+i*.026):.3f}',
             '.07 .035 .17', 'leaf', euler=f'.4 {math.sin(angle):.2f} {angle:.2f}', contype='0', conaffinity='0')
    ET.indent(root, space='  ')
    output = MODELS / 'rs_dual_tabletop_scene.xml'
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=False)
    print(output)


if __name__ == '__main__':
    build()
