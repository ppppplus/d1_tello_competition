#!/usr/bin/env python3
"""Create a writable run from installed assets and supervise Gazebo processes."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from entry_scene import adapt_entry


def boolean(value):
    if value.lower() not in ('true', 'false'):
        raise argparse.ArgumentTypeError('use true or false')
    return value.lower() == 'true'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['demo', 'student'], default='demo')
    parser.add_argument('--gui', type=boolean, default=True)
    parser.add_argument('--cameras', type=boolean, default=False)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--output', default='')
    parser.add_argument('--max-sim-time', type=float, default=0.)
    parser.add_argument('--exit-on-finish', type=boolean, default=False)
    parser.add_argument('--fire-window', type=int, choices=range(7), default=0)
    parser.add_argument('--camera-views', default='all')
    parser.add_argument('--camera-fps', type=int, choices=range(1, 61), default=20)
    args, unknown = parser.parse_known_args()
    if unknown and unknown != ['--ros-args']:
        parser.error('unexpected arguments: ' + ' '.join(unknown))
    if args.rate < 0 or args.max_sim_time < 0:
        parser.error('rate and max-sim-time must be nonnegative')
    share = Path(get_package_share_directory('d1_tello_competition'))
    prefix = Path(get_package_prefix('d1_tello_competition'))
    out = Path(args.output).expanduser().resolve() if args.output else (
        Path.home() / '.ros/d1_tello_competition/runs' / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((share / 'policy/manifest.json').read_text())
    if hashlib.sha256((share / 'policy/actor.bin').read_bytes()).hexdigest() != manifest['actor_sha256']:
        raise RuntimeError('Policy checksum mismatch')
    config = json.loads((share / 'config/mission.json').read_text())
    config.update(output=str(out), mode=args.mode)
    text = (share / 'worlds/competition.sdf.in').read_text()
    # Resolve paths as XML text so install directories containing & also work.
    tree = ET.ElementTree(ET.fromstring(text))
    for element in tree.iter():
        if element.text:
            element.text = element.text.replace('@SHARE@', str(share)).replace('@OUTPUT@', str(out))
    world = tree.find('world')
    adapt_entry(world, config, json.loads((share / 'config/entry.json').read_text()), args.fire_window)
    (out / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2))
    world.find('physics/real_time_factor').text = str(args.rate)
    views = set(args.camera_views.split(','))
    if not views <= {'all', 'overview', 'dog', 'uav', 'fpv', 'uav_front', 'entry'}:
        parser.error('unknown camera view')
    for parent in tree.iter():
        for sensor in list(parent):
            if sensor.tag != 'sensor' or sensor.get('type') != 'camera':
                continue
            role = sensor.findtext('topic', '').split('/')[-2]
            if 'all' not in views and role not in views:
                parent.remove(sensor)
            else:
                sensor.find('update_rate').text = str(args.camera_fps)
    if not args.cameras:
        for parent in tree.iter():
            for child in list(parent):
                if (child.tag == 'sensor' and child.get('type') == 'camera') or (
                        child.tag == 'plugin' and child.get('name') == 'ignition::gazebo::systems::Sensors'):
                    parent.remove(child)
    for uri in tree.iter('uri'):
        if not Path(uri.text).is_file():
            raise FileNotFoundError(uri.text)
    tree.write(out / 'world.sdf', encoding='unicode')
    env = os.environ.copy()
    env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = str(prefix / 'lib') + os.pathsep + env.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    env.setdefault('IGN_PARTITION', 'd1_tello_competition')
    children, logs = [], []

    def start(command, name):
        log = (out / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        return process

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    print(f'Run directory: {out}\nMode: {args.mode}', flush=True)
    result_code = 0
    try:
        server = start(['ign', 'gazebo', '-s', '-r', str(out / 'world.sdf')], 'gazebo')
        gui = start([str(prefix / 'lib/d1_tello_competition/competition_gui'), str(share / 'config/gui.config')], 'gui') if args.gui else None
        bridge = None
        if args.cameras:
            bridge = start([str(prefix / 'lib/d1_tello_competition/camera_bridge')], 'cameras')
        while server.poll() is None:
            if gui and gui.poll() is not None:
                result_code = gui.returncode
                break
            if bridge and bridge.poll() is not None:
                raise RuntimeError(f'Camera bridge exited: see {out / "cameras.log"}')
            if (out / 'result.json').exists() and args.exit_on_finish:
                try:
                    result = json.loads((out / 'result.json').read_text())
                except json.JSONDecodeError:
                    time.sleep(.1)
                    continue
                result_code = 0 if result['strict_pass'] else 1
                print(f'Mission: {result["completed_count"]}/3, pass={result["strict_pass"]}, failure={result["failure"]}', flush=True)
                break
            if args.max_sim_time and (out / 'mission_state.json').exists():
                state = json.loads((out / 'mission_state.json').read_text())
                if state['time'] >= args.max_sim_time:
                    break
            time.sleep(.1)
        else:
            result_code = server.returncode or 0
        if result_code:
            print(f'Simulation exited with code {result_code}. Logs: {out}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in reversed(children):
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())
