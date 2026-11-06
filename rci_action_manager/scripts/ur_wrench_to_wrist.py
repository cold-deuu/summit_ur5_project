#!/usr/bin/env python3
"""Transform measured UR wrench to wrist axes/origin; publishes no commands."""
from collections import deque
import threading
import time
import traceback
import numpy as np
import rospy
import tf
from geometry_msgs.msg import WrenchStamped

class WrenchAdapter:
    def __init__(self):
        self.target=rospy.get_param('~target_frame','wrist_3_link')
        self.source=rospy.get_param('~source_frame','tool0_controller')
        self.listener=tf.TransformListener()
        self.queue=deque(maxlen=100);self.lock=threading.Lock()
        self.received=0;self.published=0;self.last_output=time.monotonic();self.tf_age=None
        self.publisher=rospy.Publisher('/hqp_shadow/wrench',WrenchStamped,queue_size=5)
        self.subscriber=rospy.Subscriber('/wrench',WrenchStamped,self.receive,queue_size=10,tcp_nodelay=True)
        # Processing runs in the wall-clock main loop below. Subscriber/TF
        # threads keep receiving even if ROS time is corrected.
    def receive(self,msg):
        if msg.header.frame_id.lstrip('/')!=self.source.lstrip('/'):
            rospy.logwarn_throttle(2,'Unexpected UR wrench frame; rejecting');return
        if msg.header.stamp.is_zero():return
        with self.lock:self.queue.append(msg);self.received+=1
    def process(self,event):
        # Delay 30 ms to allow measured TCP TF to arrive. Never substitute an
        # identity or latest transform when the stamped transform is unavailable.
        now=rospy.Time.now();msg=None
        with self.lock:
            while self.queue and (now-self.queue[0].header.stamp).to_sec()>.2:
                self.queue.popleft()
            if not self.queue:return
        try:
            latest=self.listener.getLatestCommonTime(self.target,self.source)
            self.tf_age=(now-latest).to_sec() if not latest.is_zero() else 0.0
            # Field TF can trail the sensor by more than 30 ms. Choose the
            # newest bounded-age sample whose exact transform has arrived;
            # retain newer samples for retry rather than dropping them.
            with self.lock:
                msg=next((value for value in reversed(self.queue)
                          if .03<=(now-value.header.stamp).to_sec()<=.2 and
                          (latest.is_zero() or value.header.stamp<=latest)),None)
            if msg is None:
                if not latest.is_zero() and (now-latest).to_sec()>.2:
                    rospy.logwarn_throttle(2,'Wrench-to-wrist TF is stale; withholding transformed wrench')
                return
            position,quaternion=self.listener.lookupTransform(self.target,self.source,msg.header.stamp)
        except (tf.Exception,tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
            rospy.logwarn_throttle(2,'Waiting for stamped wrench-to-wrist TF');return
        R=tf.transformations.quaternion_matrix(quaternion)[:3,:3]
        wrench=msg.wrench
        force=np.array([wrench.force.x,wrench.force.y,wrench.force.z])
        torque=np.array([wrench.torque.x,wrench.torque.y,wrench.torque.z])
        if not np.isfinite(np.r_[force,torque,position,R.ravel()]).all():return
        force=R@force;torque=R@torque+np.cross(position,force)
        out=WrenchStamped();out.header.stamp=msg.header.stamp;out.header.frame_id=self.target
        out.wrench.force.x,out.wrench.force.y,out.wrench.force.z=force
        out.wrench.torque.x,out.wrench.torque.y,out.wrench.torque.z=torque
        self.publisher.publish(out)
        self.published+=1;self.last_output=time.monotonic()
        with self.lock:
            while self.queue and self.queue[0].header.stamp<=msg.header.stamp:
                self.queue.popleft()

if __name__=='__main__':
    rospy.init_node('ur_wrench_to_wrist');adapter=WrenchAdapter()
    # Do not let an uncaught Timer callback exception silently disable F/T.
    # An errored iteration publishes nothing; retain bounded-age data to retry.
    next_update=time.monotonic()
    while not rospy.is_shutdown():
        try:
            adapter.process(None)
        except Exception:
            rospy.logerr_throttle(2,'Wrench adapter processing failed; no transformed output: %s',traceback.format_exc())
        if time.monotonic()-adapter.last_output>1:
            rospy.logwarn_throttle(2,'Wrench adapter stalled: received=%d transformed=%d buffered=%d TF_age=%s processor=wall_loop',
                                   adapter.received,adapter.published,len(adapter.queue),adapter.tf_age)
        next_update+=.01
        if next_update<time.monotonic():next_update=time.monotonic()+.01
        time.sleep(max(0,next_update-time.monotonic()))
