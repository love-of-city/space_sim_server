// Native implementation of online_elbow_ik.py, not a dynamics or IK replacement.
// Fixed six-axis doubles, no fast-math, no changed thresholds or search budget.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C" __attribute__((visibility("default")))
#endif

template <int N> using Vec = std::array<double, N>;
template <int R, int C> using Mat = std::array<Vec<C>, R>;
using V3 = Vec<3>;
using V6 = Vec<6>;
using M3 = Mat<3, 3>;
using M4 = Mat<4, 4>;
using M6 = Mat<6, 6>;

template <std::size_t N> double dot(const std::array<double,N>& a, const std::array<double,N>& b) {
    double r = 0.; for (std::size_t i=0;i<N;++i) r += a[i]*b[i]; return r;
}
template <std::size_t N> double norm(const std::array<double,N>& a) { return std::sqrt(dot(a,a)); }
template <int N> Vec<N> load(const double* p) {
    Vec<N> a{}; std::copy_n(p,N,a.begin()); return a;
}
template <std::size_t N> std::array<double,N> add(const std::array<double,N>& a,
        const std::array<double,N>& b, double scale=1.) {
    auto c=a; for (std::size_t i=0;i<N;++i) c[i]+=scale*b[i]; return c;
}
template <int N> Mat<N,N> eye() {
    Mat<N,N> a{}; for (int i=0;i<N;++i) a[i][i]=1.; return a;
}
template <std::size_t R, std::size_t K, std::size_t C>
std::array<std::array<double,C>,R> mul(const std::array<std::array<double,K>,R>& a,
        const std::array<std::array<double,C>,K>& b) {
    std::array<std::array<double,C>,R> c{};
    for (std::size_t i=0;i<R;++i) for(std::size_t j=0;j<C;++j)
        for(std::size_t k=0;k<K;++k) c[i][j]+=a[i][k]*b[k][j];
    return c;
}
template <std::size_t R, std::size_t C>
std::array<double,R> mv(const std::array<std::array<double,C>,R>& a, const std::array<double,C>& b) {
    std::array<double,R> c{}; for (std::size_t i=0;i<R;++i) c[i]=dot(a[i],b); return c;
}
M3 transpose(const M3& a) {
    M3 b{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) b[i][j]=a[j][i]; return b;
}
V3 cross(const V3& a, const V3& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
Vec<4> quaternion(const M3& m) {
    Vec<4> q{};
    double tr=m[0][0]+m[1][1]+m[2][2],s;
    if(tr>0) {
        s=std::sqrt(tr+1.)*2.;
        q={.25*s,(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s};
    } else if(m[0][0]>=m[1][1] && m[0][0]>=m[2][2]) {
        s=std::sqrt(1.+m[0][0]-m[1][1]-m[2][2])*2.;
        q={(m[2][1]-m[1][2])/s,.25*s,(m[0][1]+m[1][0])/s,(m[0][2]+m[2][0])/s};
    } else if(m[1][1]>=m[2][2]) {
        s=std::sqrt(1.+m[1][1]-m[0][0]-m[2][2])*2.;
        q={(m[0][2]-m[2][0])/s,(m[0][1]+m[1][0])/s,.25*s,(m[1][2]+m[2][1])/s};
    } else {
        s=std::sqrt(1.+m[2][2]-m[0][0]-m[1][1])*2.;
        q={(m[1][0]-m[0][1])/s,(m[0][2]+m[2][0])/s,(m[1][2]+m[2][1])/s,.25*s};
    }
    const double n=norm(q); for(auto& v:q) v/=n;
    if(q[0]<0) for(auto& v:q) v=-v;
    return q;
}
double angle(const M3& m) {
    auto q=quaternion(m); return 2.*std::atan2(norm(V3{q[1],q[2],q[3]}),std::abs(q[0]));
}

struct Geometry { V3 p{}; M3 r{}; std::array<V3,6> origins{},axes{}; M6 jac{}; };
Geometry geometry(int segments, const double* packed, const V6& q) {
    Geometry g; auto t=eye<4>();
    for(int s=0;s<segments;++s) {
        const double* p=packed+s*23;
        M4 fixed{}; for(int i=0;i<4;++i) for(int j=0;j<4;++j) fixed[i][j]=p[i*4+j];
        t=mul(t,fixed);
        int index=static_cast<int>(p[16]);
        if(index<0) continue;
        if(index>=6) throw std::invalid_argument("joint index");
        V3 pos=load<3>(p+17), axis=load<3>(p+20);
        auto origin=mv(t,Vec<4>{pos[0],pos[1],pos[2],1.});
        auto rotated=mv(t,Vec<4>{axis[0],axis[1],axis[2],0.});
        for(int i=0;i<3;++i) {g.origins[index][i]=origin[i];g.axes[index][i]=rotated[i];}
        const double an=norm(axis); for(auto& v:axis)v/=an;
        const double x=axis[0],y=axis[1],z=axis[2];
        M3 skew{{{0.,-z,y},{z,0.,-x},{-y,x,0.}}};
        auto square=mul(skew,skew); auto rotation=eye<4>();
        const double sn=std::sin(q[index]),cn=std::cos(q[index]);
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)
            rotation[i][j]=(i==j?1.:0.)+sn*skew[i][j]+(1.-cn)*square[i][j];
        auto before=eye<4>(),after=eye<4>();
        for(int i=0;i<3;++i){before[i][3]=pos[i];after[i][3]=-pos[i];}
        t=mul(mul(mul(t,before),rotation),after);
    }
    for(int i=0;i<3;++i){g.p[i]=t[i][3];for(int j=0;j<3;++j)g.r[i][j]=t[i][j];}
    for(int j=0;j<6;++j) {
        const auto c=cross(g.axes[j],add(g.p,g.origins[j],-1.));
        for(int i=0;i<3;++i){g.jac[i][j]=c[i];g.jac[i+3][j]=g.axes[j][i];}
    }
    return g;
}
std::pair<double,V6> height(const Geometry& g,int first,int second,const V3& up) {
    V6 grad{};
    for(int j=0;j<second;++j)grad[j]+=dot(cross(g.axes[j],add(g.origins[second],g.origins[j],-1.)),up);
    for(int j=0;j<first;++j)grad[j]-=dot(cross(g.axes[j],add(g.origins[first],g.origins[j],-1.)),up);
    return {dot(up,add(g.origins[second],g.origins[first],-1.)),grad};
}
V6 solve(M6 a,V6 b) {
    // Partial-pivot LU, same linear system as numpy.linalg.solve (not a new solver policy).
    for(int k=0;k<6;++k) {
        int pivot=k;for(int i=k+1;i<6;++i)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;
        if(!std::isfinite(a[pivot][k]) || a[pivot][k]==0.)throw std::runtime_error("singular metric");
        std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);
        for(int i=k+1;i<6;++i){
            const double factor=a[i][k]/a[k][k];
            for(int j=k+1;j<6;++j)a[i][j]-=factor*a[k][j];
            b[i]-=factor*b[k];
        }
    }
    V6 result{};
    for(int i=5;i>=0;--i){double v=b[i];for(int j=i+1;j<6;++j)v-=a[i][j]*result[j];result[i]=v/a[i][i];}
    return result;
}
enum Param {
    HEIGHT,HEIGHT_GAIN,HEIGHT_RATE,HEIGHT_WEIGHT,WRIST_DROP,WRIST_GAIN,WRIST_RATE,WRIST_WEIGHT,
    J3_MARGIN,J3_ANGLE_SCALE,J3_LENGTH_SCALE,J3_GAIN,J3_RATE,J3_WEIGHT,REG,CORRECTION,
    LINEAR_LIMIT,ANGULAR_LIMIT,RELATIVE,CHAR_LENGTH,NOMINAL,POSITION_LIMIT,ORIENTATION_LIMIT
};
enum Diag {
    D_HEIGHT,D_PRED_HEIGHT,D_WRIST,D_PRED_WRIST,D_J3,D_PRED_J3,D_ELBOW_ACTIVE,D_WRIST_ACTIVE,
    D_J3_ACTIVE,D_BASE_COST,D_PRED_COST,D_CORRECTION,D_LINEAR,D_ANGULAR,D_LINEAR_BUDGET,
    D_ANGULAR_BUDGET,D_SCALE,D_POSITION_OFFSET,D_ORIENTATION_OFFSET
};
enum Status {IDLE,SHAPE_SATISFIED,HEIGHT_SATISFIED,TASK_SHAPE,TASK_HEIGHT,NO_CORRECTION,LIMITED,ACTIVE};
double ball_scale(const V3& offset,const V3& change,double radius) {
    double a=dot(change,change); if(a<=1e-30)return 1.;
    double b=dot(offset,change),remaining=std::max(0.,radius*radius-dot(offset,offset));
    return std::clamp((-b+std::sqrt(b*b+a*remaining))/a,0.,1.);
}

API int posture_abi() { return 1; }
API int posture_geometry(int segments,const double* packed,const double* q,double* output) {
    try {
        if(segments<=0 || segments>128 || !packed || !q || !output)return -1;
        for(int i=0;i<6;++i)if(!std::isfinite(q[i]))return -1;
        const auto g=geometry(segments,packed,load<6>(q));
        std::fill_n(output,88,0.);
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j)output[i*4+j]=g.r[i][j];
            output[i*4+3]=g.p[i];
        }
        output[15]=1.;
        for(int i=0;i<6;++i)for(int j=0;j<3;++j){
            output[16+i*3+j]=g.origins[i][j];
            output[34+i*3+j]=g.axes[i][j];
        }
        for(int i=0;i<6;++i)for(int j=0;j<6;++j)output[52+i*6+j]=g.jac[i][j];
        return 0;
    } catch(...) { return -1; }
}
API int posture_step(int segments,const double* packed,const double* prefs,const int* indices,
        const double* input,double dt,int initialized,double* state,double* output,double* diag) {
    try {
        if(segments<=0 || segments>128 || !packed || !prefs || !indices || !input ||
            !state || !output || !diag || !std::isfinite(dt) || dt<=0.) return -1;
        for(int i=0;i<2;++i)if(indices[i]<0 || indices[i]>=6)return -1;
        for(int i=2;i<5;++i)if(indices[i]<-1 || indices[i]>=6)return -1;
        const auto q=load<6>(input),twist=load<6>(input+6),velocity=load<6>(input+12),
            speed=load<6>(input+18),low=load<6>(input+24),high=load<6>(input+30);
        for(int i=0;i<6;++i)if(!std::isfinite(q[i]) || !std::isfinite(twist[i]) ||
                !std::isfinite(speed[i]) || speed[i]<=0 || std::isnan(low[i]) ||
                std::isnan(high[i]) || low[i]>high[i] || q[i]<low[i] || q[i]>high[i])return -1;
        const bool wrist=indices[2]>=0,joint3=indices[4]>=0;
        const int shoulder=indices[0],elbow=indices[1],wu=indices[2],wl=indices[3],j3=indices[4];
        if(wrist && wl<0)return -1;
        V3 up=load<3>(prefs+23);const double un=norm(up);if(un<1e-12)return -1;
        for(auto& v:up)v/=un;
        const double* p=prefs;
        auto g=geometry(segments,packed,q);
        auto [h,gradient]=height(g,shoulder,elbow,up);
        auto [drop,wgrad]=wrist?height(g,wl,wu,up):std::pair<double,V6>{0.,{}};
        std::fill_n(diag,19,0.);
        diag[D_HEIGHT]=diag[D_PRED_HEIGHT]=h;diag[D_WRIST]=diag[D_PRED_WRIST]=drop;
        diag[D_J3]=diag[D_PRED_J3]=joint3?q[j3]:0.;
        if(std::all_of(twist.begin(),twist.end(),[](double v){return v==0.;}))return IDLE;
        V6 lower{},upper{};
        for(int i=0;i<6;++i){
            lower[i]=std::max(-speed[i],(low[i]-q[i])/dt);upper[i]=std::min(speed[i],(high[i]-q[i])/dt);
            if(!std::isfinite(velocity[i]) || velocity[i]<lower[i]-1e-10 || velocity[i]>upper[i]+1e-10)return -2;
        }
        const auto base_q=add(q,velocity,dt);auto bg=geometry(segments,packed,base_q);
        V3 anchor=load<3>(state);M3 anchor_r{};
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)anchor_r[i][j]=state[3+i*3+j];
        if(initialized){anchor=add(anchor,add(bg.p,g.p,-1.));anchor_r=mul(mul(bg.r,transpose(g.r)),anchor_r);}
        else{anchor=bg.p;anchor_r=bg.r;}
        std::copy(anchor.begin(),anchor.end(),state);
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)state[3+i*3+j]=anchor_r[i][j];
        const auto offset=add(bg.p,anchor,-1.);
        const auto relative=mul(bg.r,transpose(anchor_r));
        diag[D_POSITION_OFFSET]=norm(offset);diag[D_ORIENTATION_OFFSET]=angle(relative);
        const double jscale=p[J3_LENGTH_SCALE]/p[J3_ANGLE_SCALE];
        auto shape=[&](const Geometry& k) {
            return std::pair<double,double>{dot(up,add(k.origins[elbow],k.origins[shoulder],-1.)),
                wrist?dot(up,add(k.origins[wu],k.origins[wl],-1.)):0.};
        };
        auto cost=[&](double eh,double wd,const V6& joints) {
            auto sq=[](double x){return x*x;};
            double value=p[HEIGHT_WEIGHT]*sq(std::max(0.,p[HEIGHT]-eh));
            if(wrist)value+=p[WRIST_WEIGHT]*sq(std::max(0.,p[WRIST_DROP]-wd));
            if(joint3)value+=p[J3_WEIGHT]*sq(jscale*std::max(0.,joints[j3]+p[J3_MARGIN]));
            return value;
        };
        auto [bh,bd]=shape(bg);const double base_cost=cost(bh,bd,base_q);
        diag[D_PRED_HEIGHT]=bh;diag[D_PRED_WRIST]=bd;diag[D_BASE_COST]=diag[D_PRED_COST]=base_cost;
        diag[D_PRED_J3]=joint3?base_q[j3]:0.;
        struct Goal{double value;V6 grad;double target,gain,rate,weight;int diag_index;};
        std::array<Goal,3> goals{};
        int count=1;
        goals[0]={h,gradient,p[HEIGHT],p[HEIGHT_GAIN],p[HEIGHT_RATE],p[HEIGHT_WEIGHT],D_ELBOW_ACTIVE};
        if(wrist)goals[count++]={drop,wgrad,p[WRIST_DROP],p[WRIST_GAIN],p[WRIST_RATE],p[WRIST_WEIGHT],D_WRIST_ACTIVE};
        if(joint3){V6 grad{};grad[j3]=-jscale;goals[count++]={-jscale*q[j3],grad,jscale*p[J3_MARGIN],
            p[J3_GAIN],jscale*p[J3_RATE],p[J3_WEIGHT],D_J3_ACTIVE};}
        bool satisfied=true;for(int i=0;i<count;++i)satisfied&=goals[i].value>=goals[i].target;
        if(satisfied)return wrist||joint3?SHAPE_SATISFIED:HEIGHT_SATISFIED;
        M6 weighted=g.jac;for(int i=3;i<6;++i)for(auto& v:weighted[i])v*=p[CHAR_LENGTH];
        M6 metric{};V6 rhs{};bool terms=false;
        for(int i=0;i<6;++i)for(int j=0;j<6;++j){
            for(int k=0;k<6;++k)metric[i][j]+=weighted[k][i]*weighted[k][j];
            if(i==j)metric[i][j]+=p[REG]*p[REG];
        }
        for(int k=0;k<count;++k){
            const auto& goal=goals[k];
            const double desired=std::min(goal.rate,goal.gain*std::max(0.,goal.target-goal.value));
            const double deficit=desired-dot(goal.grad,velocity);
            if(goal.value>=goal.target || deficit<=0 || norm(goal.grad)<1e-12)continue;
            diag[goal.diag_index]=1.;terms=true;
            for(int i=0;i<6;++i){
                rhs[i]+=goal.weight*deficit*goal.grad[i];
                for(int j=0;j<6;++j)metric[i][j]+=goal.weight*(goal.grad[i]*goal.grad[j]);
            }
        }
        if(!terms)return wrist||joint3?TASK_SHAPE:TASK_HEIGHT;
        const auto correction=solve(metric,rhs);if(norm(correction)<1e-12)return NO_CORRECTION;
        const double equivalent=std::hypot(norm(load<3>(input+6)),p[CHAR_LENGTH]*norm(load<3>(input+9)));
        const double linear_budget=std::min(p[LINEAR_LIMIT],p[RELATIVE]*equivalent),
            angular_budget=std::min(p[ANGULAR_LIMIT],p[RELATIVE]*equivalent/p[CHAR_LENGTH]);
        diag[D_LINEAR_BUDGET]=linear_budget;diag[D_ANGULAR_BUDGET]=angular_budget;
        double maxc=0.;for(auto v:correction)maxc=std::max(maxc,std::abs(v));
        double scale=std::min(1.,p[CORRECTION]*std::min(1.,equivalent/p[NOMINAL])/maxc);
        for(int i=0;i<6;++i) {
            if(correction[i]>1e-14)scale=std::min(scale,std::max(0.,(upper[i]-velocity[i])/correction[i]));
            else if(correction[i]<-1e-14)scale=std::min(scale,std::max(0.,(lower[i]-velocity[i])/correction[i]));
        }
        const auto disturbance=mv(g.jac,correction);
        const V3 ld=load<3>(disturbance.data()),ad=load<3>(disturbance.data()+3);
        if(norm(ld)>0)scale=std::min(scale,linear_budget/norm(ld));
        if(norm(ad)>0)scale=std::min(scale,angular_budget/norm(ad));
        if(scale<=1e-12)return LIMITED;
        auto quat=quaternion(relative);V3 orientation{quat[1],quat[2],quat[3]};
        double sine=norm(orientation);for(auto& v:orientation)v*=sine<1e-12?2.:angle(relative)/sine;
        for(int k=0;k<2;++k) {
            V3 off=k?orientation:offset,change=k?ad:ld;for(auto& v:change)v*=dt;
            const double radius=k?p[ORIENTATION_LIMIT]:p[POSITION_LIMIT];
            if(radius-norm(off)<1e-8 && dot(off,change)>0)return LIMITED;
            const double allowed=ball_scale(off,change,radius);if(allowed<scale)scale=.95*allowed;
        }
        if(scale<=1e-10)return LIMITED;
        for(int retry=0;retry<4;++retry) {
            const auto step=add(velocity,correction,scale),next_q=add(q,step,dt);
            const auto ng=geometry(segments,packed,next_q);auto [nh,nd]=shape(ng);
            const double next_cost=cost(nh,nd,next_q);
            V6 scaled=correction;for(auto& v:scaled)v*=scale;const auto delta=mv(g.jac,scaled);
            const double linear=std::max(norm(load<3>(delta.data())),norm(add(ng.p,bg.p,-1.))/dt);
            const double angular=std::max(norm(load<3>(delta.data()+3)),angle(mul(ng.r,transpose(bg.r)))/dt);
            bool bounds=true;for(int i=0;i<6;++i)bounds&=next_q[i]>=low[i]-1e-12 && next_q[i]<=high[i]+1e-12;
            const double po=norm(add(ng.p,anchor,-1.)),ro=angle(mul(ng.r,transpose(anchor_r)));
            if(bounds && linear<=linear_budget+1e-12 && angular<=angular_budget+1e-12 &&
                po<=p[POSITION_LIMIT]+1e-12 && ro<=p[ORIENTATION_LIMIT]+1e-12 && next_cost<base_cost-1e-12) {
                const auto achieved=mv(g.jac,step),residual=add(twist,achieved,-1.);
                std::copy(step.begin(),step.end(),output);std::copy(achieved.begin(),achieved.end(),output+6);
                std::copy(residual.begin(),residual.end(),output+12);
                diag[D_PRED_HEIGHT]=nh;diag[D_PRED_WRIST]=nd;diag[D_PRED_COST]=next_cost;
                diag[D_PRED_J3]=joint3?next_q[j3]:0.;diag[D_CORRECTION]=norm(scaled);
                diag[D_LINEAR]=linear;diag[D_ANGULAR]=angular;diag[D_SCALE]=scale;
                diag[D_POSITION_OFFSET]=po;diag[D_ORIENTATION_OFFSET]=ro;
                return ACTIVE;
            }
            scale*=.5;
        }
        return LIMITED;
    } catch(...) { return -3; }
}
