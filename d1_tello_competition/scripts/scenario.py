"""Seeded referee scenario and uniform stations; no contestant solution."""
import copy
import random
import secrets

COLORS = {'red': '.88 .06 .06 1', 'green': '.04 .72 .22 1',
          'gray': '.48 .48 .48 1'}


def randomize(world, config, entry, seed=-1, fire_window=0):
    seed = secrets.randbits(31) if seed < 0 else seed
    rng = random.Random(seed)
    # Both floors participate without requiring a descent or a harder gait.
    while True:
        slots = sorted(rng.sample(range(6), 4))
        if slots[0] < 3 <= slots[-1]:
            break
    roles = ['red', 'red', 'green', 'green']
    rng.shuffle(roles)
    assignment = dict(zip(slots, roles))
    config['seed'] = seed
    config['task_mode'] = 'random_four_of_six'
    config['tasks'] = []
    config['stations'] = []
    for slot in range(6):
        x, y, z = (2.5, 4.7, 6.9)[slot % 3], (4.3 if slot < 3 else 6.5), (0 if slot < 3 else .8)
        direction = 1 if slot < 3 else -1
        color = assignment.get(slot, 'gray')
        station = dict(id='target_'+str(slot+1), slot=slot, position=[x,y,z],
                       marker_position=[x,y+direction*.4,z+.942],
                       delivery_point=[x,y-direction*.3,z],
                       support_link='floor' if slot<3 else 'upper_platform')
        config['stations'].append(station)
        if color != 'gray':
            config['tasks'].append(dict(station, color='red' if color=='red' else 'green',
                                        marker_color=color, required_deliveries=0 if color=='red' else 1))
        tag=world.find("model[@name='compact_two_level_arena']/link[@name='station_%d_tag']/visual[@name='task_top']" % (slot+1))
        for channel in ('ambient','diffuse'):
            tag.find('material/'+channel).text=COLORS[color]
    config['station_colors']=[assignment.get(s,'gray') for s in range(6)]
    entry=copy.deepcopy(entry)
    entry['fire_window_id']=fire_window or rng.randint(1,6)
    return entry
