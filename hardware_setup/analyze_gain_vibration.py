#!/usr/bin/env python3
"""Offline comparison of recent recorded SE3 gains/ripple; no ROS connection."""
import argparse
from pathlib import Path
import csv,json,numpy as np
from scipy.signal import butter,sosfiltfilt,welch
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
parser=argparse.ArgumentParser();parser.add_argument('path',nargs='?',default=str(Path.home()/'catkin_ws/artifacts/real_action_logs'));parser.add_argument('--last',type=int,default=4);args=parser.parse_args()
root=Path(args.path);names=[p.parent.name for p in sorted(root.glob('*/telemetry.csv'),key=lambda p:p.stat().st_mtime)[-args.last:]]
records=[];series=[]
for name in names:
 rows=list(csv.DictReader((root/name/'telemetry.csv').open()));meta=json.load((root/name/'metadata.json').open());kp=meta['parameters']['se3_kp'];record=dict(run=name,se3_kp=kp,se3_kd=meta['parameters']['se3_kd'],joint_kp=meta['parameters']['joint_kp'],dropped=meta['dropped'],windows=[])
 indices=[i for i,r in enumerate(rows) if r['mode']=='HOLDING'];groups=np.split(indices,np.where(np.diff(indices)>1)[0]+1)
 se3=[r for r in rows if r['mode']=='SE3']
 if len(se3)<80:continue
 identifier=max(int(r['action_id']) for r in se3);se3=[r for r in se3 if int(r['action_id'])==identifier]
 pe=np.array([[float(r['translation_error_'+str(i)]) for i in range(3)] for r in se3])
 record['se3_position_rms_mm']=float(np.sqrt(np.mean(np.sum(pe**2,axis=1)))*1000)
 windows=[('initial_hold',[rows[i] for i in groups[0]]),('se3',se3),('final_hold',[rows[i] for i in groups[-1]])]
 for label,part in windows:
  if len(part)<80:continue
  t=np.array([float(r['time']) for r in part]);t-=t[0]
  p=np.array([[float(r['actual_'+a]) for a in 'xyz'] for r in part])*1000
  R=Rotation.from_quat(np.array([[float(r['actual_'+a]) for a in ['qx','qy','qz','qw']] for r in part]));rot=(R*R[0].inv()).as_rotvec()*180/np.pi
  if len(p)<80:continue
  hp=sosfiltfilt(butter(3,3,fs=100,btype='highpass',output='sos'),p,axis=0)
  hr=sosfiltfilt(butter(3,3,fs=100,btype='highpass',output='sos'),rot,axis=0)
  if label=='se3':
   intervals=[('moving',(t>.5)&(t<2.8)),('late_settling',(t>=5.2)&(t<t[-1]-.2))]
   m=(t>.5)&(t<2.8);f,psd=welch(rot[m,1],fs=100,nperseg=128,detrend='linear');sel=(f>=3)&(f<=45)
   series.append((name,kp,meta['parameters']['se3_kd'],t,hr[:,1],f[sel],psd[sel]))
  else:intervals=[(label,(t>.2)&(t<t[-1]-.2))]
  for interval,mask in intervals:
   if mask.sum()<30:continue
   record['windows'].append(dict(window=interval,samples=int(mask.sum()),
     position_hp3hz_rms_norm_mm=float(np.sqrt(np.mean(np.sum(hp[mask]**2,axis=1)))),
     rotation_hp3hz_rms_norm_deg=float(np.sqrt(np.mean(np.sum(hr[mask]**2,axis=1)))),
     rotation_y_hp3hz_rms_deg=float(np.sqrt(np.mean(hr[mask,1]**2))),rotation_y_hp3hz_peak_to_peak_deg=float(np.ptp(hr[mask,1]))))
 records.append(record)
assert records,'No usable SE3 runs found'
out=root/records[-1]['run']/'analysis';out.mkdir(exist_ok=True)
(out/'vibration_analysis.json').write_text(json.dumps(dict(method='3 Hz third-order zero-phase high-pass, 100-Hz sampled FK; transition edges excluded; not external vibration metrology',nyquist_hz=50,runs=records),indent=2))
fig,axes=plt.subplots(2,2,figsize=(13,9))
for i,(name,kp,kd,t,y,f,p) in enumerate(series):
 label='Kp=%s Kd=%s (%s)'%(kp,kd,name[9:15]);axes[0,0].plot(t,y,label=label,alpha=.75);axes[0,1].semilogy(f,p,label=label)
axes[0,0].set(xlim=(.5,3),xlabel='SE3 time [s]',ylabel='Rotation Y, >3 Hz [deg]',title='Small angular ripple during motion')
axes[0,1].set(xlim=(3,30),xlabel='Frequency [Hz]',ylabel='PSD [deg^2/Hz]',title='Moving interval 0.5-2.8 s; Welch n=128')
x=np.arange(len(records));width=.25
for j,window in enumerate(['initial_hold','late_settling','final_hold']):
 values=[next((w['position_hp3hz_rms_norm_mm'] for w in r['windows'] if w['window']==window),float('nan')) for r in records]
 axes[1,0].bar(x+(j-1)*width,values,width,label=window)
axes[1,0].set_xticks(x);axes[1,0].set_xticklabels([str(r['se3_kp'])+'/'+str(r['se3_kd']) for r in records]);axes[1,0].set(xlabel='SE3 Kp/Kd runs',ylabel='>3 Hz position RMS norm [mm]',title='Hold/late-settling high-frequency components')
axes[1,1].bar(x,[r['se3_position_rms_mm'] for r in records]);axes[1,1].set_xticks(x);axes[1,1].set_xticklabels([str(r['se3_kp'])+'/'+str(r['se3_kd']) for r in records]);axes[1,1].set(xlabel='SE3 Kp/Kd runs',ylabel='Tracking RMS norm [mm]',title='Tracking improves with increased Kp')
for ax in axes.flat:ax.grid(True,alpha=.3)
for ax in [axes[0,0],axes[0,1],axes[1,0]]:ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(out/'gain_and_vibration_comparison.png',dpi=170)
print(json.dumps(records[-1],indent=2));print('Saved',out/'gain_and_vibration_comparison.png')
