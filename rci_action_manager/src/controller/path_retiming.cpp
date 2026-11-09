#include "rci_action_manager/controller/moveit_optimal_controller.hpp"
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/parametrizer/spline.hpp>
#include <toppra/solver/seidel.hpp>
#include <cmath>
#include <limits>

namespace moveit_optimal {
namespace {
// The received C2 interpolant is the geometric path; planner time is only its parameter.
class PlanPath : public toppra::GeometricPath {
public:
    explicit PlanPath(const Plan &plan) : toppra::GeometricPath(6),plan_(plan) {}
    toppra::Vector eval_single(toppra::value_type s,int order=0) const override {
        const Sample value=plan_.sample(s);
        if(order==0) return value.q;
        if(order==1) return value.v;
        if(order==2) return value.a;
        throw std::invalid_argument("PlanPath provides derivatives up to order 2");
    }
    toppra::Bound pathInterval() const override { toppra::Bound b; b<<0,plan_.duration; return b; }
private:
    const Plan &plan_;
};
}
bool retimePlan(Plan &plan,const Parameters &p,std::string &error) {
    try {
        // Grid resolution follows joint-space path length, not planner timestamps.
        double length=0; Vec previous=plan.sample(0).q;
        for(int k=1;k<=2000;++k) {
            const Vec q=plan.sample(plan.duration*k/2000.0).q;
            length+=(q-previous).norm(); previous=q;
        }
        if(!(length>1e-6)) { error="path has no length"; return false; }
        auto path=std::make_shared<PlanPath>(plan);
        // TOPP-RA switches path acceleration at every gridpoint, so a dense grid
        // makes the C2 spline jerky while a coarse one overshoots slightly. Stretch
        // each candidate uniformly in time (v/k, a/k^2, j/k^3) into all bounds and
        // keep the shortest result.
        const Vec velocity_bound=p.velocity*p.retiming_velocity_scale;
        const Vec acceleration_bound=p.acceleration*p.retiming_acceleration_scale;
        const Vec jerk_bound=p.jerk*p.retiming_jerk_scale;
        const toppra::Vector velocity=velocity_bound,acceleration=acceleration_bound;
        const int finest=static_cast<int>(std::max(10.0,std::min(1000.0,std::ceil(length/.01))));
        std::vector<Sample> samples;double duration=0,stretch=1,best=std::numeric_limits<double>::infinity();
        for(int gridpoints=finest;;gridpoints=std::max(10,gridpoints*2/3)) {
            toppra::LinearConstraintPtrs constraints{
                std::make_shared<toppra::constraint::LinearJointVelocity>(-velocity,velocity),
                std::make_shared<toppra::constraint::LinearJointAcceleration>(-acceleration,acceleration)};
            constraints[1]->discretizationType(toppra::DiscretizationType::Interpolation);
            toppra::algorithm::TOPPRA algorithm(constraints,path);
            algorithm.solver(std::make_shared<toppra::solver::Seidel>());
            algorithm.setN(gridpoints);
            if(algorithm.computePathParametrization(0,0)==toppra::ReturnCode::OK) {
                const auto &data=algorithm.getParameterizationData();
                // Cubic spline through the TOPP-RA grid: continuous acceleration instead
                // of the piecewise-constant path acceleration of the raw solution.
                toppra::parametrizer::Spline spline(path,data.gridpoints,data.parametrization);
                const double candidate_duration=spline.pathInterval()(1);
                if(spline.validate() && candidate_duration>0) {
                    const size_t count=std::max<size_t>(1,std::min<size_t>(p.max_points-1,
                        static_cast<size_t>(std::ceil(candidate_duration/p.retiming_sample_period))));
                    toppra::Vector times(count+1);
                    for(size_t k=0;k<=count;++k) times(k)=candidate_duration*k/count;
                    times(count)=std::nextafter(candidate_duration,0.0); // spline range check excludes rounding
                    const toppra::Vectors q=spline.eval(times,0),v=spline.eval(times,1),a=spline.eval(times,2);
                    std::vector<Sample> candidate(count+1);
                    double velocity_ratio=0,acceleration_ratio=0,jerk_ratio=0;bool finite=true;
                    for(size_t k=0;k<=count;++k) {
                        candidate[k].q=q[k];candidate[k].v=v[k];candidate[k].a=a[k];
                        finite=finite && candidate[k].q.allFinite() && candidate[k].v.allFinite() && candidate[k].a.allFinite();
                        velocity_ratio=std::max(velocity_ratio,candidate[k].v.cwiseAbs().cwiseQuotient(velocity_bound).maxCoeff());
                        acceleration_ratio=std::max(acceleration_ratio,candidate[k].a.cwiseAbs().cwiseQuotient(acceleration_bound).maxCoeff());
                        if(k) jerk_ratio=std::max(jerk_ratio,((candidate[k].a-candidate[k-1].a).cwiseAbs()/
                            (times(k)-times(k-1))).cwiseQuotient(jerk_bound).maxCoeff());
                    }
                    const double k=std::max({1.0,velocity_ratio,std::sqrt(acceleration_ratio),std::cbrt(jerk_ratio)});
                    if(finite && candidate_duration*k<best) {
                        best=candidate_duration*k;stretch=k;duration=candidate_duration;samples.swap(candidate);
                    }
                }
            }
            if(gridpoints==10) break;
        }
        if(samples.empty()) { error="no feasible TOPP-RA parametrization"; return false; }
        const size_t count=samples.size()-1;
        toppra::Vector times(count+1);
        for(size_t k=0;k<=count;++k) {
            times(k)=duration*stretch*k/count;
            samples[k].v/=stretch;samples[k].a/=stretch*stretch;
        }
        // Both ends at rest (HOLDING transfer). The clamped spline still has a
        // nonzero end acceleration, so join each end with a quintic whose duration
        // is lengthened until its sampled velocity, acceleration and jerk fit.
        samples.front().v.setZero();samples.front().a.setZero();
        samples.back().v.setZero();samples.back().a.setZero();
        auto fits=[&](const Segment &segment) {
            Vec previous=segment.sample(segment.start).a;
            for(int k=1;k<=100;++k) {
                const Sample value=segment.sample(segment.start+segment.duration*k/100);
                if((value.v.cwiseAbs().array()>velocity_bound.array()).any() ||
                   (value.a.cwiseAbs().array()>acceleration_bound.array()).any() ||
                   (((value.a-previous)/(segment.duration/100)).cwiseAbs().array()>jerk_bound.array()).any()) return false;
                previous=value.a;
            }
            return true;
        };
        auto blend=[&](const Sample &from,const Sample &to,double nominal) {
            double duration=nominal;
            for(int k=0;k<40 && !fits(Segment::make(0,duration,from,to));++k) duration*=1.1;
            return duration;
        };
        const double end_window=std::min(.25*times(count),.5);
        size_t first=1,last=count-1;
        while(first<count && times(first)<end_window) ++first;
        while(last>first && times(count)-times(last)<end_window) --last;
        plan.segments.clear(); // capacity reserved at startup (max_points)
        if(first>=last) {
            const double total=blend(samples.front(),samples.back(),times(count));
            plan.segments.push_back(Segment::make(0,total,samples.front(),samples.back()));
        } else {
            const double head=blend(samples.front(),samples[first],times(first)),shift=head-times(first);
            plan.segments.push_back(Segment::make(0,head,samples.front(),samples[first]));
            for(size_t k=first;k<last;++k)
                plan.segments.push_back(Segment::make(times(k)+shift,times(k+1)-times(k),samples[k],samples[k+1]));
            const double tail=blend(samples[last],samples.back(),times(count)-times(last));
            plan.segments.push_back(Segment::make(times(last)+shift,tail,samples[last],samples.back()));
        }
        plan.first=samples.front();plan.last=samples.back();plan.timed=true;
        plan.duration=plan.segments.back().start+plan.segments.back().duration;
        return true;
    } catch(const std::exception &e) {
        error=e.what(); return false;
    }
}
}
