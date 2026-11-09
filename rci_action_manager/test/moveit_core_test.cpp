#include "rci_action_manager/controller/moveit_optimal_controller.hpp"
#include <iostream>
#include <stdexcept>
extern "C" void allocation_guard_start();
extern "C" unsigned long allocation_guard_stop();
using namespace moveit_optimal;
void require(bool value,const char *why) { if(!value) throw std::runtime_error(why); }
int main(int argc,char **argv) {
    require(argc==2 || argc==3,"Pass URDF path and optional native flag");
    PathProgressGovernor governor;
    governor.startCruise(1,.5,2,10);
    double old_position=0,old_velocity=0,old_acceleration=0;
    int cruise_samples=0;
    for(int i=0;i<2000;++i) {
        governor.advance(.01);
        require(governor.position>=old_position-1e-9 && governor.position<=1+1e-9,"Governor overshot/backtracked");
        require(governor.velocity>=-1e-8 && governor.velocity<=.500001,"Governor speed bound");
        require(std::abs(governor.acceleration)<=2.000001,"Governor acceleration bound");
        require(std::abs(governor.acceleration-old_acceleration)<=.100001,"Governor jerk bound");
        require(std::abs(governor.velocity-old_velocity)<=.020001,"Governor velocity discontinuity");
        old_position=governor.position;old_velocity=governor.velocity;old_acceleration=governor.acceleration;
        if(std::abs(governor.velocity-.5)<1e-8)++cruise_samples;
    }
    require(std::abs(governor.position-1)<1e-8 && std::abs(governor.velocity)<1e-8,"Governor endpoint did not stop");
    require(cruise_samples>50,"Intermediate path did not maintain cruise speed");
    // Both jerk-triangular and acceleration-limited short moves must stop.
    for(double length: {.001,.05,.2}) {
        governor.startCruise(length,.5,2,10);
        for(int i=0;i<2000;++i) {
            const double previous=governor.position;governor.advance(.01);
            require(governor.position>=previous-1e-9 && governor.position<=length+1e-9,"Short S-curve overshoot");
        }
        require(std::abs(governor.position-length)<1e-9 && std::abs(governor.velocity)<1e-9,"Short S-curve did not stop");
    }
    PathClock clock;
    for(int i=0;i<1000;++i) {
        const double previous=clock.rate;
        const double increment=clock.advance(i<200 || i>=600?1:0,.01,10);
        require(increment>=-1e-10 && increment<=.01000001,"Path clock increment outside bounds");
        require(clock.rate>=-1e-10 && clock.rate<=1.00000001,"Path clock rate outside bounds");
        require(std::abs(clock.rate-previous)<.04,"Path clock rate discontinuity");
    }
    Sample a,b; a.q.setConstant(.1); a.v.setConstant(.02); a.a.setConstant(.01);
    b.q.setConstant(.3); b.v.setConstant(.03); b.a.setConstant(-.01);
    auto segment=Segment::make(0,2,a,b);
    for(const auto &test: std::array<std::pair<double,Sample>,2>{{{0,a},{2,b}}}) {
        auto value=segment.sample(test.first);
        require((value.q-test.second.q).norm()<1e-10,"Quintic position boundary");
        require((value.v-test.second.v).norm()<1e-10,"Quintic velocity boundary");
        require((value.a-test.second.a).norm()<1e-10,"Quintic acceleration boundary");
    }
    Plan joined;Sample end_of_path=b;end_of_path.q.setConstant(.4);end_of_path.v.setZero();end_of_path.a.setZero();
    joined.first=a;joined.last=end_of_path;joined.duration=3.3;
    joined.segments.push_back(segment);joined.segments.push_back(Segment::make(2,1.3,b,end_of_path));
    require(joined.buildPath(),"Joined spline path build");
    const double knot=2*joined.parameter_scale;
    const Sample left=joined.sampleProgress(knot-1e-7),right=joined.sampleProgress(knot+1e-7);
    require((left.q-right.q).norm()<1e-5,"Spline knot position discontinuity");
    require((left.v-right.v).norm()<1e-5,"Spline knot velocity discontinuity");
    require((left.a-right.a).norm()<1e-4,"Spline knot acceleration discontinuity");
    for(bool torque_limits: {false,true}) {
        Parameters p; p.urdf=argv[1]; p.position_margin=0;
        p.joint_names={"ur5e_shoulder_pan_joint","ur5e_shoulder_lift_joint","ur5e_elbow_joint",
            "ur5e_wrist_1_joint","ur5e_wrist_2_joint","ur5e_wrist_3_joint"};
        if(argc==3) {
            for(auto &name:p.joint_names) name=name.substr(5);
            p.wrist_joint="wrist_3_joint";
        }
        p.lower=Vec::Constant(-3.141592653589793); p.upper=-p.lower;
        p.velocity.setConstant(.4); p.acceleration.setConstant(.8); p.effort<<150,150,150,28,28,28;
        p.torque_limits=torque_limits;
        p.admittance.mass<<3,3,3,.3,.3,.3;
        p.admittance.damping<<35,35,35,3.5,3.5,3.5;
        p.tracking_stiffness<<100,100,100,10,10,10;
        p.holding_stiffness=p.tracking_stiffness;
        p.admittance.displacement_limit.setConstant(.08); p.admittance.velocity_limit.setConstant(.06);
        Controller controller(p);
        JointState joints; joints.valid=true;
        joints.q={3.0,-1.57,1.57,-1.57,-1.57,0};
        Wrench wrench; wrench.ready=true;
        double now=1;
        Snapshot status;
        // Warm model/solver, then inspect the entire normal control step.
        for(int i=0;i<10;++i) {
            joints.stamp=wrench.stamp=now;
            controller.joint_input.store(joints); controller.wrench_input.store(wrench);
            controller.step(now); now+=.01;
        }
        int slot=controller.acquirePlan(); auto &plan=controller.plan(slot);
        Sample start,end; start.q=Eigen::Map<const Vec>(joints.q.data()); end=start; end.q(0)-=.05;
        plan.first=start; plan.last=end; plan.duration=2; plan.start_time=now; plan.id=1;
        plan.segments.push_back(Segment::make(0,2,start,end)); controller.submitPlan(slot);
        unsigned long allocations=0;
        for(int i=0;i<500;++i) {
            joints.stamp=wrench.stamp=now;
            wrench.value[2]=(i>=50 && i<70)?6:0;
            controller.joint_input.store(joints); controller.wrench_input.store(wrench);
            allocation_guard_start();
            controller.step(now);
            allocations+=allocation_guard_stop();
            require(controller.output.load(status),"Snapshot unavailable");
            require(status.command_valid,"Control step failed");
            require(status.reason!=Reason::QP_FAILURE,"QP failure");
            for(int j=0;j<6;++j) {
                require(std::abs(status.velocity[j])<=.400001,"Velocity bound");
                require(std::abs(status.velocity[j]-joints.v[j])<=.008001,"Acceleration bound");
                joints.q[j]=status.command[j]; joints.v[j]=status.velocity[j];
            }
            if(i==100) controller.hold_mode.store(true);
            if(i==150) controller.hold_mode.store(false);
            now+=.01;
        }
        std::cout<<"torque_limits="<<torque_limits<<" allocations_in_500_steps="<<allocations<<std::endl;
        require(allocations==0,"Dynamic allocation in control step");

        // A plan from the displaced measured posture must not reapply the
        // accumulated K=0 HOLD offset (regression for early path aborts).
        p.hold_mode=true; p.limit_joint_admittance=true;p.tracking_admittance=false;
        p.tracking_stiffness.setZero(); p.holding_stiffness.setZero();
        Controller displaced(p);
        joints.q={0,-1.57,1.57,-1.57,-1.57,0};joints.v.fill(0);
        auto advance=[&]() {
            joints.stamp=wrench.stamp=now;
            displaced.joint_input.store(joints);displaced.wrench_input.store(wrench);
            displaced.step(now);require(displaced.output.load(status),"Displaced snapshot");
            require(status.command_valid,"Displaced control step");
            joints.q=status.command;joints.v=status.velocity;now+=.01;
        };
        for(int i=0;i<800;++i) {
            wrench.value.fill(0);wrench.value[2]=i<300?6:0;
            wrench.value[5]=i<300?.3:0; // Accumulate rotation before TRACKING bypass.
            advance();
        }
        require(Eigen::Map<const Vec>(status.displacement.data()).head<3>().norm()>.04,"Regression requires accumulated HOLD displacement");
        require(Eigen::Map<const Vec>(status.displacement.data()).tail<3>().norm()>.01,"Regression requires accumulated HOLD rotation");
        slot=displaced.acquirePlan();auto &fresh=displaced.plan(slot);
        start=Sample();start.q=Eigen::Map<const Vec>(joints.q.data());end=start;end.q(0)+=.02;
        fresh.first=start;fresh.last=end;fresh.duration=2;fresh.start_time=now;fresh.id=2;
        fresh.segments.push_back(Segment::make(0,2,start,end));displaced.submitPlan(slot);
        for(int i=0;i<250;++i) {
            wrench.value[2]=6; // Wrench must not affect TRACKING when bypassed.
            advance();
            require(status.result!=Result::ABORTED,"New plan reapplied old HOLD displacement");
            require(status.max_error<.05,"New plan reference discontinuity after HOLD");
            if(status.state==moveit_optimal::State::TRACKING) {
                for(int j=0;j<6;++j) {
                    require(std::abs(status.displacement[j])<1e-12,"TRACKING retained admittance displacement");
                    require(std::abs(status.admittance_velocity[j])<1e-12,"TRACKING retained admittance velocity");
                }
            }
        }
        require(status.result==Result::SUCCEEDED,"Displaced-start plan did not finish");

        // Master OFF must retain a fixed HOLD reference despite nonzero wrench.
        p.admittance_enabled=false;p.geometric_tracking=true;p.acceleration_limits=false;
        Controller rigid(p);joints.v.fill(0);
        const auto held=joints.q;
        for(int i=0;i<300;++i) {
            joints.stamp=wrench.stamp=now;wrench.value.fill(10);
            rigid.joint_input.store(joints);rigid.wrench_input.store(wrench);
            rigid.step(now);require(rigid.output.load(status),"Rigid snapshot");
            require(status.command_valid,"Rigid HOLD control step");
            for(int j=0;j<6;++j) {
                require(std::abs(status.displacement[j])<1e-12,"Master OFF displacement");
                require(std::abs(status.admittance_velocity[j])<1e-12,"Master OFF velocity");
                require(std::abs(status.command[j]-held[j])<1e-8,"Master OFF HOLD moved under wrench");
            }
            joints.q=status.command;joints.v=status.velocity;now+=.01;
        }
        // A 50-ms input schedule is intentionally infeasible. Geometric mode
        // must preserve its path and finish later under the same QP bounds.
        slot=rigid.acquirePlan();auto &fast=rigid.plan(slot);
        start=Sample();start.q=Eigen::Map<const Vec>(joints.q.data());end=start;end.q(0)+=.2;
        fast.first=start;fast.last=end;fast.duration=.05;fast.start_time=now;fast.id=3;
        fast.segments.push_back(Segment::make(0,.05,start,end));
        require(fast.buildPath(),"Valid geometric path rejected by builder");
        double progress=0;const Vec original=start.q;
        fast.follow(original,progress,p.path_lookahead,p.path_search_window);
        require(progress<1e-10,"Stationary robot advanced path progress");
        Vec advanced=original;advanced(0)+=.08;
        fast.follow(advanced,progress,p.path_lookahead,p.path_search_window);
        const double reached=progress;
        fast.follow(original,progress,p.path_lookahead,p.path_search_window);
        require(progress>=reached,"Path progress moved backwards");
        rigid.submitPlan(slot);allocations=0;
        const double geometric_start=now;double completed_after=-1,peak_velocity=0,peak_velocity_step=0;
        Vec prior_velocity=Vec::Zero();
        for(int i=0;i<1200;++i) {
            joints.stamp=wrench.stamp=now;
            rigid.joint_input.store(joints);rigid.wrench_input.store(wrench);
            allocation_guard_start();rigid.step(now);allocations+=allocation_guard_stop();
            require(rigid.output.load(status),"Geometric snapshot");
            require(status.command_valid && status.result!=Result::ABORTED,"Geometric tracking failed");
            if(status.result==Result::SUCCEEDED && completed_after<0)completed_after=now-geometric_start;
            for(int j=0;j<6;++j) {
                require(std::abs(status.velocity[j])<=.400001,"Geometric velocity bound");
                peak_velocity=std::max(peak_velocity,std::abs(status.velocity[j]));
                // Scalar progress acceleration is not a joint acceleration
                // bound: q_ss*s_dot^2 and the adaptive clock also contribute.
                // QP acceleration is intentionally OFF in this scenario.
                peak_velocity_step=std::max(peak_velocity_step,std::abs(status.velocity[j]-prior_velocity(j)));
                const double measured_step=.2*(status.command[j]-joints.q[j]);
                joints.q[j]+=measured_step;joints.v[j]=measured_step/.01;
            }
            prior_velocity=Eigen::Map<const Vec>(status.velocity.data());now+=.01;
            if(i==20)require(status.state==moveit_optimal::State::TRACKING,"Fast schedule ended geometric tracking early");
        }
        require(allocations==0,"Geometric RT allocation");
        require(status.result==Result::SUCCEEDED,"Geometric path did not converge");
        require(std::abs(joints.q[0]-end.q(0))<p.goal_tolerance,"Geometric endpoint error");
        std::cout<<"geometric input_seconds=.05 completed_seconds="<<completed_after
                 <<" peak_velocity="<<peak_velocity<<" max_velocity_step="<<peak_velocity_step
                 <<" allocations="<<allocations<<std::endl;

        // Disable the explicit acceleration bound without disabling speed
        // bounds. Isolate this check from the separate predicted-torque bound.
        p.acceleration_limits=false;p.torque_limits=false;
        p.geometric_tracking=false;p.transition_duration=.02;
        Controller unslewed(p);joints.v.fill(0);joints.stamp=wrench.stamp=now;
        unslewed.joint_input.store(joints);unslewed.wrench_input.store(wrench);unslewed.step(now);now+=.01;
        slot=unslewed.acquirePlan();auto &direct=unslewed.plan(slot);
        start=Sample();start.q=Eigen::Map<const Vec>(joints.q.data());end=start;end.q(0)+=.02;
        direct.first=start;direct.last=end;direct.duration=.05;direct.start_time=now;direct.id=4;
        direct.segments.push_back(Segment::make(0,.05,start,end));
        require(direct.buildPath(),"Acceleration-OFF path build");unslewed.submitPlan(slot);
        joints.stamp=wrench.stamp=now;
        unslewed.joint_input.store(joints);unslewed.wrench_input.store(wrench);unslewed.step(now);
        now+=.01;joints.stamp=wrench.stamp=now;
        unslewed.joint_input.store(joints);unslewed.wrench_input.store(wrench);unslewed.step(now);
        require(unslewed.output.load(status) && status.command_valid,"Acceleration-OFF step failed");
        require(std::abs(status.velocity[0])>.008001,"Acceleration bound remained active");
        require(std::abs(status.velocity[0])<=.400001,"Acceleration OFF bypassed speed bound");

        // A fixed-schedule linear corner (wrist reversal) exceeds the bounds as
        // timed. TOPP-RA retiming must bound the reference and track it.
        {
            p.acceleration_limits=true;p.torque_limits=torque_limits;p.geometric_tracking=true;
            p.transition_duration=.4;p.path_retiming=true;p.jerk<<8,8,8,10,10,10;
            Controller smooth(p);joints.v.fill(0);
            for(int i=0;i<10;++i) {
                joints.stamp=wrench.stamp=now;wrench.value.fill(0);
                smooth.joint_input.store(joints);smooth.wrench_input.store(wrench);smooth.step(now);now+=.01;
            }
            slot=smooth.acquirePlan();auto &corner=smooth.plan(slot);
            Sample s0,s1,s2;s0.q=Eigen::Map<const Vec>(joints.q.data());s1=s0;s2=s0;
            s1.q(5)+=1.2;s2.q(5)+=.2;s1.q(0)+=.1;s2.q(0)+=.2;s1.v=(s2.q-s0.q)/4;
            corner.first=s0;corner.last=s2;corner.duration=4;corner.start_time=now;corner.id=5;
            corner.segments.push_back(Segment::make(0,2,s0,s1));corner.segments.push_back(Segment::make(2,2,s1,s2));
            std::string error;
            require(retimePlan(corner,p,error),"TOPP-RA retiming failed");
            require(corner.timed && corner.duration>4,"Retiming did not slow the infeasible schedule");
            require((corner.last.q-s2.q).norm()<1e-6 && corner.last.v.norm()<1e-12,"Retimed endpoint changed");
            for(int k=0;k<=1000;++k) {
                const Sample value=corner.sample(corner.duration*k/1000.0);
                for(int j=0;j<6;++j) {
                    require(std::abs(value.v(j))<=p.velocity(j)*p.retiming_velocity_scale*1.05,"Retimed velocity bound");
                    require(std::abs(value.a(j))<=p.acceleration(j)*p.retiming_acceleration_scale*1.1,"Retimed acceleration bound");
                }
            }
            double reference_jerk=0;
            for(int k=1;k<=1000;++k) {
                const double t0=corner.duration*(k-1)/1000.0,t1=corner.duration*k/1000.0;
                reference_jerk=std::max(reference_jerk,((corner.sample(t1).a-corner.sample(t0).a)/(t1-t0)).cwiseAbs().cwiseQuotient(p.jerk).maxCoeff());
            }
            require(reference_jerk<=p.retiming_jerk_scale*1.05,"Retimed reference jerk bound");
            require(corner.buildPath(),"Retimed path build");smooth.submitPlan(slot);
            allocations=0;double peak_jerk=0;Vec prior_velocity=Vec::Zero(),prior_acceleration=Vec::Zero();
            for(int i=0;i<3000 && status.result!=Result::SUCCEEDED;++i) {
                joints.stamp=wrench.stamp=now;
                smooth.joint_input.store(joints);smooth.wrench_input.store(wrench);
                allocation_guard_start();smooth.step(now);allocations+=allocation_guard_stop();
                require(smooth.output.load(status) && status.command_valid,"Retimed step failed");
                require(status.result!=Result::ABORTED,"Retimed tracking aborted");
                const Vec velocity=Eigen::Map<const Vec>(status.velocity.data());
                const Vec acceleration=(velocity-prior_velocity)/.01;
                for(int j=0;j<6;++j) {
                    peak_jerk=std::max(peak_jerk,std::abs(acceleration(j)-prior_acceleration(j))/.01);
                }
                prior_velocity=velocity;prior_acceleration=acceleration;
                joints.q=status.command;joints.v=status.velocity;now+=.01;
            }
            require(allocations==0,"Retimed RT allocation");
            require(status.result==Result::SUCCEEDED,"Retimed corner did not finish");
            std::cout<<"retimed duration="<<corner.duration<<" reference_jerk/limit="<<reference_jerk<<" peak_command_jerk="<<peak_jerk
                     <<" allocations="<<allocations<<std::endl;
        }
    }
}
