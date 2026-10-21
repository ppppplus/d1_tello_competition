"""Build physical facade openings and matching reference entry/exit routes."""
import copy
import math
import xml.etree.ElementTree as ET


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def box(model, name, position, size, color, collision=True):
    link = element(model, 'link', name=name)
    element(link, 'pose', ' '.join(map(str, [*position, 0, 0, 0])))
    for kind in ('collision', 'visual') if collision else ('visual',):
        obj = element(link, kind, name=name + '_' + kind)
        element(element(element(obj, 'geometry'), 'box'), 'size', ' '.join(map(str, size)))
        if kind == 'visual':
            material = element(obj, 'material')
            element(material, 'ambient', color)
            element(material, 'diffuse', color)
    return link


def adapt_entry(world, config, entry, fire_window=0):
    entry = copy.deepcopy(entry)
    if fire_window:
        entry['fire_window_id'] = fire_window
    windows = {w['id']: w for w in entry['windows']}
    if entry['fire_window_id'] not in windows:
        raise ValueError('fire_window must select one of the configured windows')
    selected = windows[entry['fire_window_id']]
    entry['selected_window'] = selected
    x0, x1, front, back = entry['bounds_xy']
    thickness, height = entry['wall_thickness'], entry['wall_height']
    door = entry['door']
    holes = [(door['center_x'] - door['width']/2, door['center_x'] + door['width']/2, 0., door['height'])]
    holes += [(w['center_x'] - w['width']/2, w['center_x'] + w['width']/2,
               w['center_z'] - w['height']/2, w['center_z'] + w['height']/2)
              for w in entry['windows']]
    for a, b, low, high in holes:
        if not (x0 <= a < b <= x1 and 0 <= low < high < height):
            raise ValueError('Opening lies outside the facade')
    model = element(world, 'model', name='building_envelope')
    element(model, 'static', 'true')
    cuts = sorted({x0, x1, *(v for h in holes for v in h[:2])})
    # Each x strip is split vertically around its opening; no collision box
    # spans a door/window. Frames sit outside the free opening dimensions.
    for i, (left, right) in enumerate(zip(cuts, cuts[1:])):
        openings = sorted((low, high) for a, b, low, high in holes if a < (left+right)/2 < b)
        bottom = 0.
        for j, (low, high) in enumerate(openings + [(height, height)]):
            if low > bottom:
                box(model, f'facade_{i}_{j}', ((left+right)/2, front, (bottom+low)/2),
                    (right-left, thickness, low-bottom), '.69 .72 .76 1')
            bottom = high
    box(model, 'west_wall', (x0, (front+back)/2, height/2), (thickness, back-front, height), '.66 .70 .75 1')
    box(model, 'east_wall', (x1, (front+back)/2, height/2), (thickness, back-front, height), '.66 .70 .75 1')
    box(model, 'north_wall', ((x0+x1)/2, back, height/2), (x1-x0, thickness, height), '.66 .70 .75 1')
    # Collision-only ceiling keeps the overhead teaching camera unobstructed.
    ceiling = element(model, 'link', name='ceiling')
    element(ceiling, 'pose', f'{(x0+x1)/2} {(front+back)/2} {height+.04} 0 0 0')
    obj = element(ceiling, 'collision', name='ceiling_collision')
    element(element(element(obj, 'geometry'), 'box'), 'size', f'{x1-x0} {back-front} .08')
    for w in entry['windows']:
        a, b = w['center_x']-w['width']/2, w['center_x']+w['width']/2
        low, high = w['center_z']-w['height']/2, w['center_z']+w['height']/2
        color = '.15 .48 .74 1'  # All windows match; only the red circle identifies the entry.
        for side, x in enumerate((a-.025, b+.025)):
            box(model, f'window_{w["id"]}_side_{side}', (x, front-.095, w['center_z']), (.05, .035, w['height']+.1), color, False)
        for side, z in enumerate((low-.025, high+.025)):
            box(model, f'window_{w["id"]}_horizontal_{side}', (w['center_x'], front-.095, z), (w['width']+.1, .035, .05), color, False)
    for side, x in enumerate((door['center_x']-door['width']/2-.025, door['center_x']+door['width']/2+.025)):
        box(model, f'door_frame_{side}', (x, front-.095, door['height']/2), (.05, .035, door['height']), '.98 .75 .05 1', False)
    box(model, 'door_header', (door['center_x'], front-.095, door['height']+.025), (door['width']+.1, .035, .05), '.98 .75 .05 1', False)
    marker = element(model, 'link', name='fire_window_marker')
    marker_z = selected['center_z']+selected['height']/2+.3
    element(marker, 'pose', f'{selected["center_x"]} {front-thickness/2-.012} {marker_z} {math.pi/2} 0 0')
    visual = element(marker, 'visual', name='red_fire_circle')
    cylinder = element(element(visual, 'geometry'), 'cylinder')
    element(cylinder, 'radius', entry.get('fire_marker_radius', .15))
    element(cylinder, 'length', .012)
    material = element(visual, 'material')
    element(material, 'ambient', '1 .015 .01 1')
    element(material, 'diffuse', '1 .015 .01 1')
    element(material, 'emissive', '.4 0 0 1')
    entry['fire_marker_position'] = [selected['center_x'], front-thickness/2-.012, marker_z]
    config['entry'] = entry
    x, z = selected['center_x'], selected['center_z']

    def waypoint(position, phase, speed=.8):
        return dict(position=position, phase=phase, speed=speed, task=-1)

    config['patrol'] = [
        waypoint([*config['uav_initial_pose'][:2], z], 'takeoff'),
        waypoint([x, front-1.8, z], 'fire_window_align'),
        waypoint([x, front-.65, z], 'window_approach', .35),
        waypoint([x, front+.85, z], 'window_enter', .35),
        waypoint([x, front+.85, 3.45], 'indoor_climb', .4),
    ] + config['patrol'][1:]
    if config.get('landing_mode') == 'upper_rally':
        config['egress'] = [
            waypoint([*config['landing_point'], 3.45], 'return_to_upper_rally'),
            waypoint([*config['landing_point'], config['landing_z']], 'landing', .35),
        ]
    else:
        config['egress'] = [
            waypoint([x, front+.85, 3.45], 'return_to_window'),
            waypoint([x, front+.85, z], 'window_descent', .4),
            waypoint([x, front-.65, z], 'window_exit', .35),
            waypoint([*config['landing_point'], z], 'return_to_pad'),
            waypoint([*config['landing_point'], 0.], 'landing', .35),
        ]
    config['stage_meta'][0]['label'] = 'Tello 穿着火窗入楼并首报目标 · D1 在楼外等待'
    # A longer straight approach leaves the full body clear of the lintel
    # before turning. The rest of the accepted stairs/mission route is kept.
    config['route'][3][2] = front+.7
    config['route'][7][2] = front+.7
    config['stage_meta'][3]['label'] = 'D1 从独立地面门走入楼内'
    config['stage_meta'][7]['label'] = 'D1 门内转向后前往 1 号目标'
    for name, key in (('d1', 'initial_pose'), ('uav', 'uav_initial_pose')):
        pose = config[key]
        if x0 <= pose[0] <= x1 and front <= pose[1] <= back:
            raise ValueError(f'{name} must start outside the building')
        world.find(f"model[@name='{name}']/pose").text = ' '.join(map(str, pose))
    # Forward RGB camera lets students observe the red fire circle. The
    # reference uses configured waypoints; it does not claim visual recognition.
    uav_pose = config['uav_initial_pose']
    uav_pose[-1] = math.pi/2
    world.find("model[@name='uav']/pose").text = ' '.join(map(str, uav_pose))
    link = world.find("model[@name='uav']/link[@name='base_link']")
    sensor = element(link, 'sensor', name='uav_front_camera', type='camera')
    element(sensor, 'pose', '.06 0 .005 0 0 0')
    element(sensor, 'always_on', 'true')
    element(sensor, 'update_rate', 20)
    element(sensor, 'topic', '/competition/uav_front/image')
    camera = element(sensor, 'camera')
    element(camera, 'horizontal_fov', 1.3)
    image = element(camera, 'image')
    element(image, 'width', 640)
    element(image, 'height', 480)
    element(image, 'format', 'R8G8B8')
    clip = element(camera, 'clip')
    element(clip, 'near', .01)
    element(clip, 'far', 60)
    spectator = element(world, 'model', name='entry_camera')
    element(spectator, 'static', 'true')
    link = element(spectator, 'link', name='camera_link')
    sensor = element(link, 'sensor', name='entry_camera', type='camera')
    element(sensor, 'pose', f'6 -7 5.3 0 {math.atan2(3.5,8.5)} {math.pi/2}')
    element(sensor, 'always_on', 'true')
    element(sensor, 'update_rate', 20)
    element(sensor, 'topic', '/competition/entry/image')
    camera = element(sensor, 'camera')
    element(camera, 'horizontal_fov', 1.3)
    image = element(camera, 'image')
    element(image, 'width', 1280)
    element(image, 'height', 720)
    element(image, 'format', 'R8G8B8')
    clip = element(camera, 'clip')
    element(clip, 'near', .05)
    element(clip, 'far', 60)
