"""Aggregate completed positive runs and the open-gripper negative control."""
import json
from pathlib import Path
import numpy as np
from check_physical_delivery import check

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/physical_delivery'

def summarize():
    names=['delivery','seed1000','seed1001','seed1002']
    positive=[]
    for name in names:
        r=check(OUT/(name+'.json'))
        if not all(stage in [e['state'] for e in r['events']] for stage in ['clearing_box_rim','opening_after_retreat']):raise ValueError('Run predates clearance/release fixes: '+name)
        initial=np.array(r['initial_model_positions']['distractor_red']);final=np.array(r['final_model_positions']['distractor_red'])
        lift_samples=[s for s in r['samples'] if s['phase']=='transporting']
        positive.append(dict(recording=name+'.mp4',evidence=name+'.json',scene=r.get('scene',dict(seed=None,mass_kg=.03,friction=1.)),success=True,elapsed_sim_s=r['events'][-1]['time']-r['events'][1]['time'],distractor_displacement_m=float(np.linalg.norm(final-initial)),minimum_carry_center_height_m=min(s['position'][2] for s in lift_samples),final_target_position=r['final_model_positions']['target_blue_00'],floor_contact_confirmed=r['floor_contact_confirmed']))
    negative=json.loads((OUT/'open_gripper.json').read_text())
    if negative['success'] or negative['final_state']!='error' or negative['lift_confirmed']:
        raise ValueError('Open-gripper control did not fail safely')
    result=dict(engine='Gazebo Classic ODE',positive_successes=len(positive),positive_trials=len(positive),positive_runs=positive,open_gripper_control=dict(success=False,expected_failure_observed=True,evidence='open_gripper.json',last_event=negative['events'][-1]),scope='One blue 4cm cube and one red distractor; dedicated reachable scene; simulator-state position plus learned visual detection; CEM grasp parameters transferred from MuJoCo; fixed delivery controller; no hardware validation')
    (OUT/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return result

if __name__=='__main__':summarize()
