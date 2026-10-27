"""Run the actual Gazebo delivery and its negative control, serially."""
import argparse
import json
import os
from pathlib import Path
import subprocess
from check_physical_delivery import check
from summarize_physical_delivery import summarize

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'artifacts/physical_delivery'

def main(skip_default=False):
    cases=[('delivery',None,.03,1.),('seed1000',1000,.02,.7),('seed1001',1001,.04,1.3),('seed1002',1002,.03,1.)]
    for name,seed,mass,friction in cases:
        if skip_default and name=='delivery':check(OUT/'delivery.json');continue
        environment=os.environ.copy()
        environment['DELIVERY_MODE']='physical'
        environment.pop('PHYSICAL_SEED',None)
        environment['PHYSICAL_MASS']=str(mass);environment['PHYSICAL_FRICTION']=str(friction)
        if seed is not None:environment['PHYSICAL_SEED']=str(seed)
        with (OUT/(name+'.log')).open('w') as log:
            subprocess.run([str(ROOT/'run_demo.sh'),'gui:=false','output:='+str(OUT/(name+'.json'))],cwd=ROOT,env=environment,stdout=log,stderr=subprocess.STDOUT,check=True)
        check(OUT/(name+'.json'));print(name,'passed',flush=True)
    environment=os.environ.copy()
    environment['DELIVERY_MODE']='physical'
    for key in ['PHYSICAL_SEED','PHYSICAL_MASS','PHYSICAL_FRICTION']:environment.pop(key,None)
    with (OUT/'open_gripper.log').open('w') as log:
        result=subprocess.run([str(ROOT/'run_demo.sh'),'gui:=false','disable_grip:=true','output:='+str(OUT/'open_gripper.json')],cwd=ROOT,env=environment,stdout=log,stderr=subprocess.STDOUT)
    negative=json.loads((OUT/'open_gripper.json').read_text())
    if result.returncode==0 or negative['success'] or negative['events'][-1]['reason']!='no_bilateral_contact_at_grasp':
        raise RuntimeError('Negative control did not fail at the expected grasp checkpoint')
    print('Open-gripper negative control passed',flush=True)
    summarize()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--skip-default',action='store_true');a=p.parse_args()
    main(a.skip_default)
