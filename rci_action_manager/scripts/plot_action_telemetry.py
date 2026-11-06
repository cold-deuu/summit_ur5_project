#!/usr/bin/env python3
"""Plot base-frame EE trajectories, 3-axis tracking errors and admittance logs."""
import argparse,csv,json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('path',help='Run directory or telemetry.csv')
    p.add_argument('--show',action='store_true');p.add_argument('--until-fault',action='store_true',help='Plot valid history before first latched fault; CSV remains unchanged');args=p.parse_args()
    import matplotlib
    if not args.show:matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from scipy.spatial.transform import Rotation
    source=Path(args.path)
    if source.is_dir():
        if (source/'telemetry.csv').exists():source=source/'telemetry.csv'
        else:
            candidates=sorted(source.glob('*/telemetry.csv'),key=lambda item:item.stat().st_mtime)
            if not candidates:p.error('No telemetry.csv found in '+str(source))
            source=candidates[-1];print('Latest run:',source.parent)
    rows=list(csv.DictReader(source.open()));assert rows,'No telemetry samples'
    raw_count=len(rows);cutoff_stamp=None
    first_fault=next((i for i,r in enumerate(rows) if r['fault_latched']=='1'),None)
    if args.until_fault and first_fault is not None:
        if first_fault==0:p.error('No samples before first fault')
        cutoff_stamp=float(rows[first_fault]['time']);rows=rows[:first_fault]
    frames={r['frame'] for r in rows};assert len(frames)==1,'Mixed coordinate frames'
    def matrix(names):return np.array([[float(r[n]) for n in names] for r in rows])
    time=matrix(['time'])[:,0];time-=time[0]
    def vector(name,n):return matrix([name+'_'+str(i) for i in range(n)])
    def position(name):return matrix([name+'_'+axis for axis in 'xyz'])
    def rotation(name):return Rotation.from_quat(matrix([name+'_'+axis for axis in ['qx','qy','qz','qw']]))
    frame=next(iter(frames));out=source.parent/'plots';out.mkdir(exist_ok=True)
    xyz={name:position(name) for name in ['actual','nominal','reference']}
    R0=rotation('actual')[0]
    rot={name:(rotation(name)*R0.inv()).as_rotvec()*180/np.pi for name in xyz}
    pe=vector('translation_error',3)*1000;re=vector('rotation_error',3)*180/np.pi
    pe_nominal=(xyz['nominal']-xyz['actual'])*1000
    re_nominal=(rotation('nominal')*rotation('actual').inv()).as_rotvec()*180/np.pi
    styles={'nominal':dict(color='.6',ls=':'),'reference':dict(color='tab:orange',ls='--'),'actual':dict(color='tab:blue')}
    def save(fig,name):fig.tight_layout();fig.savefig(out/name,dpi=170)
    def equal_3d(ax,points):
        points=np.asarray(points);middle=(points.max(axis=0)+points.min(axis=0))/2
        radius=max(np.ptp(points,axis=0).max()/2,1e-4)
        ax.set_xlim(middle[0]-radius,middle[0]+radius);ax.set_ylim(middle[1]-radius,middle[1]+radius);ax.set_zlim(middle[2]-radius,middle[2]+radius)
    fig=plt.figure(figsize=(9,7));ax=fig.add_subplot(111,projection='3d')
    for name in xyz:ax.plot(*xyz[name].T,label=name,**styles[name])
    ax.scatter(*xyz['actual'][0],color='green',label='start');ax.scatter(*xyz['actual'][-1],color='red',label='end')
    ax.set(xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]',title='EE trajectory | '+frame);ax.legend();equal_3d(ax,np.vstack(list(xyz.values())));save(fig,'trajectory_3d.png')
    for data,filename,title,units in [(xyz,'trajectory_translation_xyz.png','Translation','m'),
        (rot,'trajectory_rotation_xyz.png','Rotation vector from initial orientation (world axes)','deg')]:
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
        for i,ax in enumerate(axes):
            for name in data:ax.plot(time,data[name][:,i],label=name,**styles[name])
            ax.set_ylabel('XYZ'[i]+' ['+units+']');ax.grid(True)
        axes[0].legend();axes[0].set_title(title+' | '+frame);axes[-1].set_xlabel('Time [s]');save(fig,filename)
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
    fault=np.array([r['fault_latched']=='1' for r in rows]);fault_time=time[np.flatnonzero(fault)[0]] if fault.any() else (cutoff_stamp-float(rows[0]['time']) if cutoff_stamp is not None else None)
    for i in range(3):
        for col,data,label in [(0,pe,'Position error [mm]'),(1,re,'Rotation-vector error [deg]')]:
            ax=axes[i,col];ax.plot(time,data[:,i],label='compliant reference error')
            nominal=pe_nominal if col==0 else re_nominal
            ax.plot(time,nominal[:,i],color='.6',ls=':',label='nominal deviation');ax.set_ylabel('XYZ'[i]+' '+label);ax.grid(True)
            if fault_time is not None:ax.axvline(fault_time,color='red',ls='--')
    for ax in axes[-1]:ax.set_xlabel('Time [s]')
    axes[0,0].legend();axes[0,0].set_title('reference - actual');axes[0,1].set_title('log(R_reference R_actual^T)');save(fig,'tracking_error_xyz.png')
    fig=plt.figure(figsize=(13,6))
    for index,data,title,unit in [(1,pe,'Position error','mm'),(2,re,'Rotation-vector error','deg')]:
        ax=fig.add_subplot(1,2,index,projection='3d');ax.plot(*data.T,label='compliant error')
        nominal=pe_nominal if index==1 else re_nominal
        ax.plot(*nominal.T,color='.6',ls=':',label='nominal deviation');equal_3d(ax,np.vstack([data,nominal]));ax.legend()
        ax.scatter(*data[0],color='green');ax.scatter(*data[-1],color='red')
        ax.set(xlabel='X ['+unit+']',ylabel='Y ['+unit+']',zlabel='Z ['+unit+']',title=title)
    save(fig,'tracking_error_3d.png')
    raw=vector('wrench_bias_corrected',6);filtered=vector('wrench_filtered',6)
    fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
    for i in range(3):
        for col,offset,unit in [(0,0,'N'),(1,3,'Nm')]:
            ax=axes[i,col];ax.plot(time,raw[:,i+offset],color='.7',label='bias corrected')
            ax.plot(time,filtered[:,i+offset],label='filtered + deadband');ax.set_ylabel('XYZ'[i]+' ['+unit+']');ax.grid(True)
    axes[0,0].legend();axes[0,0].set_title('Wrist force');axes[0,1].set_title('Wrist torque')
    for ax in axes[-1]:ax.set_xlabel('Time [s]')
    save(fig,'wrench_filter.png')
    displacement=vector('admittance_displacement',6);velocity=vector('admittance_velocity',6)
    fig,axes=plt.subplots(4,3,figsize=(14,11),sharex=True)
    for i in range(3):
        for row,data,unit in [(0,displacement[:,:3]*1000,'offset [mm]'),(1,velocity[:,:3]*1000,'velocity [mm/s]'),
                             (2,displacement[:,3:]*180/np.pi,'offset [deg]'),(3,velocity[:,3:]*180/np.pi,'velocity [deg/s]')]:
            axes[row,i].plot(time,data[:,i]);axes[row,i].set_ylabel('XYZ'[i]+' '+unit);axes[row,i].grid(True)
    for ax in axes[-1]:ax.set_xlabel('Time [s]')
    save(fig,'admittance_response.png')
    valid=np.array([r['ft_bias_ready']=='1' and not r['reason'] for r in rows]);valid=valid if valid.any() else np.ones(len(rows),bool)
    def metrics(data,mask=valid):return dict(rms_axes=np.sqrt(np.mean(data[mask]**2,axis=0)).tolist(),max_abs_axes=np.max(np.abs(data[mask]),axis=0).tolist(),max_norm=float(np.max(np.linalg.norm(data[mask],axis=1))))
    summary=dict(samples=len(rows),raw_csv_samples=raw_count,trimmed_at_fault=cutoff_stamp is not None,duration_s=float(time[-1]),frame=frame,position_error_mm=metrics(pe),rotation_error_deg=metrics(re),fault_time_s=None if fault_time is None else float(fault_time),modes=sorted({r['mode'] for r in rows}))
    summary['nominal_position_deviation_mm']=metrics(pe_nominal)
    summary['nominal_rotation_deviation_deg']=metrics(re_nominal)
    summary['actions']=[]
    for mode,identifier in sorted({(r['mode'],r.get('action_id','0')) for r in rows}):
        mask=valid & np.array([r['mode']==mode and r.get('action_id','0')==identifier for r in rows])
        if mask.any():summary['actions'].append(dict(mode=mode,action_id=int(identifier),samples=int(mask.sum()),
            position_error_mm=metrics(pe,mask),rotation_error_deg=metrics(re,mask)))
    timing_path=source.parent/'command_timing.csv'
    if timing_path.exists():
        timing_rows=list(csv.DictReader(timing_path.open()))
        timing_rows=[r for r in timing_rows if float(rows[0]['time'])<=float(r['time'])<=float(rows[-1]['time'])]
        if timing_rows:
            stamp=np.array([float(r['time']) for r in timing_rows])-float(rows[0]['time'])
            fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
            for ax,key,label in zip(axes,['interval_ms','snapshot_age_ms','sequence_gap'],
                                    ['Publish interval [ms]','Snapshot age [ms]','Control sequence gap']):
                values=np.array([float(r[key]) for r in timing_rows]);ax.plot(stamp,values);ax.set_ylabel(label);ax.grid(True)
            axes[-1].set_xlabel('Time [s]');save(fig,'command_timing.png')
            intervals=np.array([float(r['interval_ms']) for r in timing_rows if float(r['interval_ms'])>0])
            if len(intervals):summary['command_interval_ms']=dict(zip(['p50','p95','p99','max'],np.percentile(intervals,[50,95,99,100]).tolist()))
    (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2));print('Plots:',out)
    if args.show:plt.show()
    else:plt.close('all')

if __name__=='__main__':main()
