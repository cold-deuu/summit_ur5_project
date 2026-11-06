// Offline software-only A/B check. Uses ROS's actual position-only spline,
// plus a toy 30-ms first-order joint servo (NOT an identified UR robot model).
#include <trajectory_interface/quintic_spline_segment.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using Segment=trajectory_interface::QuinticSplineSegment<double>;
struct Result { double rms=0,peak=0,final_error=0; int publications=0; };
Result simulate(double horizon) {
  constexpr double driver_dt=.002,control_dt=.01,servo_tau=.03;
  Segment::State start,end,desired;
  start.position={0};end.position={0};
  Segment segment(0,start,horizon,end);
  double q=0,velocity=0,pending_position=0,sum=0;int deliver=-1;Result result;
  for(int step=0;step<4000;++step) {
    const double time=step*driver_dt;
    const double u=std::min(1.,time/3.),target=.2*(3*u*u-2*u*u*u);
    segment.sample(time,desired);
    if(step%5==0) { // 100 Hz HQP position messages; one 500-Hz tick transport delay
      const double qdot=std::max(-1.57,std::min(1.57,6*(target-q)-.1*velocity));
      pending_position=q+qdot*control_dt;deliver=step+1;++result.publications;
    }
    if(step==deliver) {
      start.position={desired.position[0]};end.position={pending_position};
      segment.init(time,start,time+horizon,end);
      segment.sample(time,desired);
    }
    velocity=(desired.position[0]-q)/servo_tau;q+=velocity*driver_dt;
    const double error=std::abs(target-q);result.peak=std::max(result.peak,error);
    sum+=error*error;result.final_error=error;
  }
  result.rms=std::sqrt(sum/4000);return result;
}
int main() {
  const auto old=simulate(.05),aligned=simulate(.01);
  if(old.publications!=800 || aligned.publications!=800 || !(aligned.rms<old.rms))
    throw std::runtime_error("100-Hz aligned-horizon regression failed");
  std::cout<<"{\"software_model_only\":true,\"publications_per_8s\":800,"
           <<"\"old_50ms_rms_rad\":"<<old.rms<<",\"new_10ms_rms_rad\":"<<aligned.rms
           <<",\"old_final_error_rad\":"<<old.final_error
           <<",\"new_final_error_rad\":"<<aligned.final_error<<"}\n";
}
