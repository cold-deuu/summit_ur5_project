#!/usr/bin/env python
"""Read-only ROS telemetry recorder. CSV disk writes happen on a worker thread."""
from __future__ import division, print_function
import argparse,csv,errno,json,os,sys,threading
from datetime import datetime
try:
    import queue
except ImportError:  # Python 2
    import Queue as queue
import rospy
from rci_action_manager.msg import ControlTelemetry
from std_msgs.msg import Float64MultiArray

POSES=['actual','nominal','reference']
VECTORS={'translation_error':3,'rotation_error':3,'joint_actual':6,'joint_command':6,
         'joint_error':6,'wrench_bias_corrected':6,'wrench_filtered':6,
         'admittance_displacement':6,'admittance_velocity':6}
FIELDS=['time','frame','mode','action_id','reason','command_enabled','ft_bias_ready','fault_latched']
for pose in POSES:FIELDS += [pose+'_'+x for x in ['x','y','z','qx','qy','qz','qw']]
for name,count in VECTORS.items():FIELDS += [name+'_'+str(i) for i in range(count)]

def row(msg):
    values=[msg.header.stamp.to_sec(),msg.header.frame_id,msg.mode,msg.action_id,msg.reason,
            int(msg.command_enabled),int(msg.ft_bias_ready),int(msg.fault_latched)]
    for name in POSES:
        pose=getattr(msg,name+'_pose');p=pose.position;q=pose.orientation
        values += [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    for name in VECTORS:values += list(getattr(msg,name))
    return values

def makedirs(path):
    try: os.makedirs(path)
    except OSError as exc:
        if exc.errno!=errno.EEXIST: raise

def open_csv(path):
    # csv needs newline='' on Python 3 and a binary file on Python 2.
    return open(path,'w',newline='') if sys.version_info[0]>=3 else open(path,'wb')

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',help='Exact run directory (default timestamped root)')
    args=parser.parse_args(rospy.myargv()[1:]);rospy.init_node('action_telemetry_logger')
    root=rospy.get_param('~output_root',os.path.join(os.path.expanduser('~'),'catkin_ws/artifacts/real_action_logs'))
    out=args.output if args.output else os.path.join(root,datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    makedirs(out)
    controller_namespace=rospy.get_param('~controller_namespace','/real_action_controller')
    metadata=dict(topic=rospy.get_param('~topic',controller_namespace+'/telemetry'),
        frame='from CSV frame column',translation_units='m',rotation_error_units='rad',
        rotation_error_definition='log(R_reference * R_actual^T), base/world axes',
        nominal='task reference before admittance',reference='task reference after admittance',
        command='joint command calculated; command_enabled does not mean command was published when fault/reason present',
        parameters=rospy.get_param(controller_namespace,{}),samples=0,dropped=0,timing_samples=0,
        command_timing_topic=controller_namespace+'/command_timing',
        command_timing_note='Node publication time/steady interval/snapshot age/sequence gap; not hardware receipt')
    pending=queue.Queue(maxsize=2000);stop=threading.Event();failure=[]
    def save_metadata():
        with open(os.path.join(out,'metadata.json'),'w') as meta: meta.write(json.dumps(metadata,indent=2))
    save_metadata()
    def receive(msg,kind='telemetry'):
        try:pending.put_nowait((kind,msg))
        except queue.Full:
            metadata['dropped']+=1;rospy.logwarn_throttle(5,'Telemetry queue full; samples dropped')
    def write():
        try:
            with open_csv(os.path.join(out,'telemetry.csv')) as file, open_csv(os.path.join(out,'command_timing.csv')) as timing_file:
                writer=csv.writer(file);writer.writerow(FIELDS)
                timing_writer=csv.writer(timing_file)
                timing_writer.writerow(['time','interval_ms','snapshot_age_ms','sequence_gap'])
                while not stop.is_set() or not pending.empty():
                    try:kind,msg=pending.get(timeout=.1)
                    except queue.Empty:continue
                    if kind=='timing':
                        if len(msg.data)!=4:continue
                        timing_writer.writerow(msg.data);metadata['timing_samples']+=1
                    else:
                        writer.writerow(row(msg));metadata['samples']+=1
                    if (metadata['samples']+metadata['timing_samples'])%100==0:
                        file.flush();timing_file.flush();save_metadata()
        except Exception as exc:
            failure.append(repr(exc));rospy.logerr('Telemetry logging stopped: %s',exc)
    worker=threading.Thread(target=write);worker.start()
    subscriber=rospy.Subscriber(metadata['topic'],ControlTelemetry,receive,queue_size=200)
    timing_subscriber=rospy.Subscriber(metadata['command_timing_topic'],Float64MultiArray,
                                     lambda msg:receive(msg,'timing'),queue_size=200)
    rospy.loginfo('Action telemetry CSV: %s',os.path.join(out,'telemetry.csv'))
    try:rospy.spin()
    finally:
        subscriber.unregister();timing_subscriber.unregister();stop.set();worker.join(timeout=5)
        metadata['writer_error']=failure;save_metadata()
        rospy.loginfo('Saved %d samples (%d dropped) to %s',metadata['samples'],metadata['dropped'],out)

if __name__=='__main__':main()
