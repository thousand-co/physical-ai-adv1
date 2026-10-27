#!/usr/bin/env python3
"""Compare recorded pregrasp phases; speed is unchanged and shorter clip is frozen."""
import json
import subprocess
from pathlib import Path
root = Path(__file__).resolve().parents[1]
folder = root / 'artifacts/motion_optimization'
values = {}
for name in ('before','after'):
    data = json.loads((folder / (name+'.json')).read_text())
    events = {e['state']:e for e in data['events']}
    start, end = events['initializing'], events['lifting']
    values[name] = {'final_state':data['final_state'],
                   'start_wall_seconds':start['wall_seconds'],
                   'grasp_complete_wall_seconds':end['wall_seconds'],
                   'initializing_to_grasp_sim_seconds':end['sim_seconds']-start['sim_seconds'],
                   'initializing_to_grasp_wall_seconds':end['wall_seconds']-start['wall_seconds']}
a,b=values['before'], values['after']
values['reduction_percent'] = 100 * (1 - b['initializing_to_grasp_sim_seconds']/a['initializing_to_grasp_sim_seconds'])
values['scope'] = 'One run per version, same single-target world; initializing to lifting. Joint feedback cumulative travel includes high-frequency simulation noise and is not used as a path-length metric.'
(folder/'comparison.json').write_text(json.dumps(values,indent=2))
duration = max(v['initializing_to_grasp_wall_seconds'] for v in (a,b)) + 2
filters=[]
for i,(name, v) in enumerate((('BEFORE',a),('AFTER',b))):
    start = v['start_wall_seconds']
    end = v['grasp_complete_wall_seconds'] + 1
    filters.append(f'[{i}:v]trim=start={start}:end={end},setpts=PTS-STARTPTS,crop=600:600:30:80,pad=640:720:20:85:color=0x12171c,tpad=stop_mode=clone:stop_duration={duration},trim=duration={duration},drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text={name}:x=24:y=20:fontsize=28:fontcolor=white,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text=Grasp_in_{v["initializing_to_grasp_sim_seconds"]:.2f}_sim_s:x=24:y=56:fontsize=18:fontcolor=white[v{i}]')
filters.append('[v0][v1]hstack=inputs=2[out]')
subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(folder/'before.mp4'),'-i',str(folder/'after.mp4'),'-filter_complex',';'.join(filters),'-map','[out]','-c:v','libx264','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(folder/'comparison.mp4')],check=True)
print(json.dumps(values,indent=2))
