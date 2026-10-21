"""Seeded referee scenario, uniform stations and known-map reference route."""
import copy
import math
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
    config['patrol']=[dict(position=[2.6,.85,3.45],task=-1)]
    for station in config['stations']:
        index=next((i for i,t in enumerate(config['tasks']) if t['slot']==station['slot']),-1)
        config['patrol'].append(dict(position=[*station['marker_position'][:2],3.45],task=index,slot=station['slot']))
    entry=copy.deepcopy(entry)
    entry['fire_window_id']=fire_window or rng.randint(1,6)
    build_route(config)
    return entry


def build_route(c):
    route=[];meta=[]
    def add(row,label,task=-1,view='walk'):
        route.append(row);meta.append(dict(label=label,task=task,view=view))
    def turn(a,label,task=-1,view='walk'):
        add(['turn',a],label+' · 转向',task,view);add(['wait',1.5],label+' · 站稳',task,view)
    def walk(x,y,a,z,label,task=-1,view='walk'):
        add(['walk',x,y,a,z,100],label,task,view)
    add(['scout',180],'等待无人机入楼并首报')
    turn(math.pi/2,'D1 进门')
    walk(1,2.2,math.pi/2,0,'D1 从黄色门框进楼')
    cx,cy=1.,2.2
    for floor in (0,1):
        z=0 if floor==0 else .8
        indices=[i for i,t in enumerate(c['tasks']) if (t['slot']>=3)==bool(floor)]
        # Visit upper stations right-to-left, ending near the left rally pads.
        if floor:indices.reverse()
        for i in indices:
            t=c['tasks'][i];x,y,_=t['position'];label=str(t['slot']+1)+' 号点';d=1 if floor==0 else -1
            add(['await_target',i,180],'等待无人机报告 '+label,i)
            a=0 if x>cx else math.pi
            turn(a,'前往 '+label,i);walk(x,cy,a,z,'前往 '+label,i)
            turn(d*math.pi/2,label+' 接近',i)
            work=y-d*(.9 if t['color']=='red' else .78)
            walk(x,work,d*math.pi/2,z,label+' 作业点',i)
            facing=d*math.pi/2 if t['color']=='red' else -d*math.pi/2
            turn(facing,label+' 装置对准',i)
            kind='fire' if t['color']=='red' else 'drop'
            for attempt in range(max(1,t['required_deliveries'])):
                add(['wait',2],label+' 停稳',i,kind)
                add([kind,5],label+' '+kind+' '+str(attempt+1),i,kind)
            turn(-d*math.pi/2,label+' 退出',i)
            walk(x,cy,-d*math.pi/2,z,label+' 回到通道',i)
            cx=x
        if floor==0:
            turn(0,'前往楼梯');walk(9.5,cy,0,0,'前往楼梯入口')
            turn(math.pi/2,'正向上楼',view='stairs');walk(9.5,8.25,math.pi/2,.8,'连续八级台阶',view='stairs')
            turn(math.pi,'上层左转',view='stairs');walk(8.25,8.25,math.pi,.8,'进入二楼',view='stairs')
            cx,cy=8.25,8.25
    turn(math.pi,'前往集合区');walk(1.,cy,math.pi,.8,'左侧黄色集合区')
    turn(-math.pi/2,'集合点定位');walk(1.,8.,-math.pi/2,.8,'进入黄色集合区')
    add(['finish',100],'两机集合')
    c['route'],c['stage_meta']=route,meta
