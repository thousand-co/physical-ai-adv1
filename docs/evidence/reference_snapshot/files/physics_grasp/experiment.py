"""Contact-only grasping with a learned, low-dimensional residual policy.

CEM optimizes approach offsets, wrist pitch, closing command and segment times.
IK and minimum-jerk interpolation are conventional controllers, not learned nets.
Object state is written only at reset. No welds, mocap or attachment forces.
"""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
import json
import shutil
import subprocess
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares
import mujoco

ROOT = Path(__file__).resolve().parent
NAMES = ['radial_offset_m', 'lateral_offset_m', 'wrist_height_m', 'pitch_rad', 'close_rad', 'move_s', 'close_s', 'lift_s']
LOW = np.array([-.02, -.04, .125, 2.7, -.1, .7, .3, .7])
HIGH = np.array([.02, .02, .16, 3.5, .66, 2.2, 1.2, 2.2])
INITIAL = np.array([0., -.015, .145, np.pi, .6, 1.4, .7, 1.4])
SCALE = np.array([.009, .009, .012, .18, .22, .3, .2, .3])

class Experiment:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.box_body = self.model.body('blue_box').id
        self.box_geom = self.model.geom('box').id
        self.hand_body = self.model.body('crane_plus_link_hand').id
        self.fixed_body = self.model.body('crane_plus_link4').id
        self.renderer = None

    def ik(self, xyz, pitch, seed=None):
        # Forward kinematics from the original URDF (metres, radians).
        def residual(q):
            yaw, a, b, c = q
            t2, t3, t4 = a, a-b, a-b-c
            radial = .083*np.sin(t2) + .0935*np.sin(t3)
            z = .0714 + .083*np.cos(t2) + .0935*np.cos(t3)
            return np.r_[np.array([radial*np.cos(yaw), radial*np.sin(yaw), z])-xyz, .05*(t4-pitch)]
        guess = [np.arctan2(xyz[1], xyz[0]), .5, -1.5, -.8] if seed is None else seed
        result = least_squares(residual, guess, bounds=(self.model.jnt_range[:4,0]+1e-5, self.model.jnt_range[:4,1]-1e-5), max_nfev=100)
        return result.x, float(np.linalg.norm(residual(result.x)))

    def frame(self, label):
        import cv2
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=720, width=960)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [.09, .02, .10]
        cam.distance, cam.azimuth, cam.elevation = .62, 140, -22
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0
        self.renderer.update_scene(self.data, camera=cam, scene_option=option)
        frame = self.renderer.render().copy()
        cv2.putText(frame, 'CRANE+ | contact-only physics | learned residual policy', (20,32), cv2.FONT_HERSHEY_SIMPLEX, .65, (240,240,240), 2)
        cv2.putText(frame, f'{label}   t={self.data.time:.2f}s', (20,64), cv2.FONT_HERSHEY_SIMPLEX, .65, (240,240,240), 2)
        return frame

    def rollout(self, params, seed=0, record=False, friction=1., mass=.03, disable_grip=False):
        rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        xy = np.array([.105, .025]) + rng.uniform([-.008,-.015], [.008,.015])
        self.data.qpos[5:8] = [*xy, .021]
        self.data.qpos[8:12] = [1,0,0,0]
        self.model.body_mass[self.box_body] = mass
        self.model.body_inertia[self.box_body] = mass*(.04**2)/6
        self.model.geom_friction[:,0] = friction
        p = np.clip(params, LOW, HIGH)
        dr, dy, height, pitch, close, move_s, close_s, lift_s = p
        yaw = np.arctan2(xy[1],xy[0])
        rot = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        # Wrist target is object position + learned local offset, not the tip.
        goal_xy = xy + rot @ [dr,dy]
        wrist = np.r_[goal_xy, height]
        grasp, err1 = self.ik(wrist, pitch)
        lift_pitch = pitch - .65
        local_grip = np.array([0., .015, .11])
        def rotation(angle):
            c,s=np.cos(angle),np.sin(angle)
            return np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]]) @ np.array([[c,0,s],[0,1,0],[-s,0,c]])
        lift_wrist = wrist + (rotation(pitch)-rotation(lift_pitch))@local_grip + [0,0,.065]
        lift, err2 = self.ik(lift_wrist, lift_pitch)
        start, err3 = lift.copy(), err2
        if max(err1,err2,err3) > .004:
            return {'success':False,'reward':-5.-100*max(err1,err2,err3),'ik_error':max(err1,err2,err3)}, []
        self.data.qpos[:5] = [*start, -.65]
        self.data.ctrl[:] = self.data.qpos[:5]
        mujoco.mj_forward(self.model, self.data)
        steps = [('settle', .3, np.r_[start,-.65]), ('approach',move_s,np.r_[grasp,-.65]), ('close',close_s,np.r_[grasp,-.65 if disable_grip else close]), ('lift',lift_s,np.r_[lift,-.65 if disable_grip else close]), ('hold',1.0,np.r_[lift,-.65 if disable_grip else close])]
        frames, heights, contacts, speeds, efforts = [], [], [], [], []
        command_peak_velocity = 0.
        command_peak_acceleration = 0.
        hold_ok = 0
        hold_total = 0
        last_frame = -1
        for phase, duration, target in steps:
            origin = self.data.ctrl.copy()
            delta = float(np.max(np.abs(target-origin)))
            duration = max(duration, 1.875*delta/1.5, np.sqrt(5.773503*delta/4.0))
            count = int(np.ceil(duration/self.model.opt.timestep))
            duration = count*self.model.opt.timestep
            command_peak_velocity = max(command_peak_velocity, 1.875*delta/duration)
            command_peak_acceleration = max(command_peak_acceleration, 5.773503*delta/duration**2)
            for k in range(count):
                t = (k+1)/count
                blend = 10*t**3-15*t**4+6*t**5
                self.data.ctrl[:] = origin+(target-origin)*blend
                mujoco.mj_step(self.model,self.data)
                if k%5 == 0:
                    touch = set()
                    for contact in self.data.contact:
                        a,b = contact.geom1,contact.geom2
                        if self.box_geom in (a,b):
                            other = b if a==self.box_geom else a
                            touch.add(int(self.model.geom_bodyid[other]))
                    bilateral = self.hand_body in touch and self.fixed_body in touch
                    z = float(self.data.xpos[self.box_body,2])
                    if phase == 'hold':
                        hold_total += 1
                        hold_ok += int(z > .065 and bilateral)
                        heights.append(z)
                        contacts.append(bilateral)
                    speeds.append(self.data.qvel[:4].copy())
                    efforts.append(float(np.sum(np.abs(self.data.actuator_force*self.data.qvel[:5]))*.01))
                if record and int(self.data.time*25) > last_frame:
                    frames.append(self.frame(phase))
                    last_frame = int(self.data.time*25)
            if os.getenv('GRASP_TRACE'):
                print(phase, 'target',target.round(3),'actual',self.data.qpos[:5].round(3),'box',self.data.xpos[self.box_body].round(3),flush=True)
        hold_fraction = hold_ok / max(hold_total,1)
        mean_height = float(np.mean(heights))
        final_height = float(self.data.xpos[self.box_body,2])
        success = bool(hold_fraction >= .95 and final_height > .065)
        velocities = np.array(speeds)
        roughness = float(np.mean(np.diff(velocities,axis=0)**2))
        reward = 8*hold_fraction + 30*np.clip(mean_height-.02,-.02,.08) + 3*success - .08*self.data.time - .05*roughness
        # A shaping term rewards physically useful partial lift and bilateral contacts.
        reward += .5*float(np.mean(contacts))
        result = dict(success=success,reward=float(reward),hold_fraction=hold_fraction,mean_hold_height_m=mean_height,final_height_m=final_height,bilateral_contact_fraction=float(np.mean(contacts)),duration_s=float(self.data.time),joint_velocity_roughness=roughness,energy_proxy_j=float(sum(efforts)),command_peak_velocity_rad_s=command_peak_velocity,command_peak_acceleration_rad_s2=command_peak_acceleration,measured_peak_velocity_rad_s=float(np.max(np.abs(velocities))),seed=seed,friction=friction,mass_kg=mass)
        return result, frames


def train(args):
    out = ROOT / 'results'
    out.mkdir(exist_ok=True)
    env = Experiment()
    rng = np.random.default_rng(args.seed)
    mean, std = INITIAL.copy(), SCALE.copy()
    best, best_score = INITIAL.copy(), -np.inf
    if args.resume:
        best = np.array(json.loads((out/'policy.json').read_text())['parameters'])
        mean, std = best.copy(), SCALE*.3
    history = []
    start_time = time.monotonic()
    baseline = [env.rollout(INITIAL,s)[0] for s in range(3)]
    for generation in range(args.generations):
        candidates = np.clip(rng.normal(mean,std,(args.population, len(mean))),LOW,HIGH)
        candidates[0] = best
        scores = []
        results = []
        for p in candidates:
            episodes = [env.rollout(p,s)[0] for s in range(3)]
            score = float(np.mean([e['reward'] for e in episodes]))
            scores.append(score)
            results.append(sum(e['success'] for e in episodes))
        elite = candidates[np.argsort(scores)[-max(4,args.population//4):]]
        mean = .25*mean+.75*elite.mean(axis=0)
        std = np.maximum(.7*elite.std(axis=0)+.3*std, SCALE*.06)
        idx = int(np.argmax(scores))
        if scores[idx] > best_score:
            best_score, best = scores[idx], candidates[idx].copy()
        item = dict(generation=generation,best_reward=best_score,batch_max_successes=max(results),mean_reward=float(np.mean(scores)),parameters=best.tolist())
        history.append(item)
        print(json.dumps(item),flush=True)
        (out/'policy.json').write_text(json.dumps(dict(algorithm='CEM residual policy search',parameter_names=NAMES,parameters=best.tolist(),training_seeds=[0,1,2],seed=args.seed,command_velocity_limit_rad_s=1.5,command_acceleration_limit_rad_s2=4.0),indent=2))
        (out/'training.json').write_text(json.dumps(history,indent=2))
    print('Training seconds',time.monotonic()-start_time,flush=True)
    evaluate(env,best,out,args.eval_episodes,baseline)


def evaluate(env,params,out,n,baseline=None):
    cases=[]
    for i in range(n):
        seed=1000+i
        friction=[.7,1.,1.3][i%3]
        mass=[.02,.03,.04][(i//3)%3]
        learned,_=env.rollout(params,seed,friction=friction,mass=mass)
        initial,_=env.rollout(INITIAL,seed,friction=friction,mass=mass)
        no_grip,_=env.rollout(params,seed,friction=friction,mass=mass,disable_grip=True)
        cases.append(dict(learned=learned,initial=initial,open_gripper_control=no_grip))
    summary={k:dict(successes=sum(x[k]['success'] for x in cases),episodes=n,success_rate=sum(x[k]['success'] for x in cases)/n) for k in ['learned','initial','open_gripper_control']}
    report=dict(summary=summary,cases=cases,baseline_training=baseline,success_definition='Box center above 0.065 m with both finger bodies in contact for >=95% of the 1 second hold; final box center above 0.065 m',scope='MuJoCo CRANE+ URDF geometry, concave hand meshes decomposed; simulator state observation; fixed phase sequence; CEM learned offsets/pitch/closing/timing; no Gazebo or hardware transfer',parameters=dict(zip(NAMES,params.tolist())))
    (out/'evaluation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(summary),flush=True)
    # Record a held-out episode, including failures without filtering.
    result,frames=env.rollout(params,1000,record=True,friction=.7,mass=.02)
    import cv2
    writer=cv2.VideoWriter(str(out/'learned_grasp.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),25,(960,720))
    if not writer.isOpened():
        raise RuntimeError('Could not open video encoder')
    for frame in frames:
        writer.write(cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
    writer.release()
    if frames and shutil.which('ffmpeg'):
        video = out/'learned_grasp.mp4'
        encoded = out/'learned_grasp_h264.mp4'
        subprocess.run(['ffmpeg','-y','-loglevel','error','-i',str(video),'-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(encoded)],check=True)
        encoded.replace(video)
    if env.renderer is not None:
        env.renderer.close()
        env.renderer = None
    if frames:
        cv2.imwrite(str(out/'hold.png'),cv2.cvtColor(frames[-1],cv2.COLOR_RGB2BGR))
    (out/'video_episode.json').write_text(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--generations',type=int,default=15)
    p.add_argument('--population',type=int,default=24)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--eval-episodes',type=int,default=30)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--evaluate',action='store_true')
    a=p.parse_args()
    if a.evaluate:
        out=ROOT/'results'
        params=np.array(json.loads((out/'policy.json').read_text())['parameters'])
        evaluate(Experiment(),params,out,a.eval_episodes)
    else:
        train(a)
