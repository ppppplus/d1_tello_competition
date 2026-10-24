"""Scenario invariants; run with python3 -m unittest discover -s tests."""
import copy,json,sys,unittest,xml.etree.ElementTree as E
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from scenario import randomize
class Scenarios(unittest.TestCase):
    def setUp(self):
        self.world=E.parse(ROOT/'worlds/competition.sdf.in').getroot().find('world')
        self.config=json.loads((ROOT/'config/mission.json').read_text());self.entry=json.loads((ROOT/'config/entry.json').read_text())
    def run_seed(self,seed,window=0):
        w,c=copy.deepcopy(self.world),copy.deepcopy(self.config)
        e=randomize(w,c,self.entry,seed,window)
        return w,c,e
    def test_randomized_constraints_and_reproducibility(self):
        seen=set();windows=set()
        for seed in range(200):
            w,c,e=self.run_seed(seed)
            self.assertEqual(c,self.run_seed(seed)[1]);self.assertEqual(e,self.run_seed(seed)[2])
            slots=[t['slot'] for t in c['tasks']];self.assertEqual(len(set(slots)),4)
            self.assertLess(min(slots),3);self.assertGreaterEqual(max(slots),3)
            self.assertEqual(sorted(t['marker_color'] for t in c['tasks']),['green','green','red','red'])
            self.assertEqual(sum(t['required_deliveries'] for t in c['tasks']),2)
            for key in ('route','stage_meta','patrol','egress'):
                self.assertNotIn(key,c)
            seen.update((t['slot'],t['marker_color']) for t in c['tasks']);windows.add(e['fire_window_id'])
        self.assertEqual(len(seen),12);self.assertEqual(windows,set(range(1,7)))
    def test_camera_information_separation(self):
        flag=1<<25
        for v in self.world.findall('.//visual'):
            if v.get('name')=='task_top':self.assertEqual(int(v.findtext('visibility_flags')),flag)
        for sensor in self.world.findall('.//sensor'):
            if sensor.get('type')!='camera':continue
            role=sensor.findtext('topic','').split('/')[-2]
            if role in ('uav','overview','fpv','dog'):
                visible=bool(int(sensor.findtext('camera/visibility_mask'))&flag)
                self.assertEqual(visible,role in ('uav','overview'))
    def test_transparent_solid_ceiling(self):
        from entry_scene import adapt_entry
        w,c,e=self.run_seed(0)
        adapt_entry(w,c,e)
        roof=w.find("model[@name='building_envelope']/link[@name='ceiling']")
        pose=list(map(float,roof.findtext('pose').split()))
        size=list(map(float,roof.findtext('collision/geometry/box/size').split()))
        self.assertAlmostEqual(pose[2]-size[2]/2,3.9)
        self.assertAlmostEqual(size[2],.08)
        self.assertEqual(roof.findall('visual'),[])
        self.assertEqual(roof.findtext('sensor/contact/collision'),'ceiling_collision')
        self.assertEqual(size[:2],[10.28,9.18-1.5])
    def test_delivery_support_matches_visible_pad(self):
        for slot in range(6):
            pad=self.world.find("model[@name='compact_two_level_arena']/link[@name='station_%d_pad']" % (slot+1))
            self.assertEqual(pad.findtext('visual/geometry/box/size'),pad.findtext('collision/geometry/box/size'))
            self.assertEqual(pad.findtext('sensor/contact/collision'),pad.find('collision').get('name'))
            terrain=json.loads((ROOT/'config/terrain.json').read_text())['surfaces']
            surface=next(s for s in terrain if s['name']=='station_%d_pad' % (slot+1))
            z=float(pad.findtext('pose').split()[2]);height=float(pad.findtext('collision/geometry/box/size').split()[2])
            self.assertAlmostEqual(surface['top_z'],z+height/2)
            self.assertAlmostEqual(surface['top_z'],(.8 if slot>=3 else 0)+.001)

        for seed in range(20):
            _,c,_=self.run_seed(seed)
            for task in c['tasks']:
                self.assertEqual(task['support_link'],'station_%d_pad' % (task['slot']+1))

    def test_window_override(self):
        for n in range(1,7):self.assertEqual(self.run_seed(0,n)[2]['fire_window_id'],n)
    def test_color_is_only_on_top(self):
        a,_,_=self.run_seed(0);b,_,_=self.run_seed(4)
        for root in (a,b):
            for parent in root.iter():
                for child in list(parent):
                    if child.tag=='visual' and child.get('name')=='task_top':parent.remove(child)
        self.assertEqual(E.tostring(a),E.tostring(b))
        self.assertEqual(len(a.findall("model[@name='demo_target_0']")),1)
        self.assertEqual(sum(m.get('name','').startswith('demo_target_') for m in a.findall('model')),6)
if __name__=='__main__':unittest.main()
