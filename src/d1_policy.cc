// D1 Fortress adapter. Upstream rl_sar observations / weights / PD settings.
// Only joint torques are applied after one-time initialization.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace dsim = ignition::gazebo;
class D1CompetitionPolicy : public dsim::System, public dsim::ISystemConfigure, public dsim::ISystemPreUpdate, public dsim::ISystemPostUpdate {
  dsim::Model model{dsim::kNullEntity};
  dsim::Link base{dsim::kNullEntity};
  std::array<dsim::Joint,12> joints;
  std::vector<std::string> names;
  Eigen::VectorXf mean, stddev, action=Eigen::VectorXf::Zero(12);
  std::array<Eigen::MatrixXf,4> weight;
  std::array<Eigen::VectorXf,4> bias;
  std::array<double,12> q0, target, torque{};
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPub;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPub;
  double now=0,lastCommand=-100,nextPolicy=0,nextPD=0;
  double pdHz=200;
  float actionClip=3.f;
  int observationDim=45;
  double phase=0,stairHeight=0,stairStart=1.,stairTread=.3,stairHalfWidth=.65,originX=-2,originY=-2;
  int stairCount=8;
  nlohmann::json surfaces=nlohmann::json::array();
  bool experiment=false;
  unsigned long pdUpdates=0;
  std::array<double,3> cmd{};
  bool initialized=false,ready=false;
  std::array<bool,4> intervalFootContact{};
  bool intervalNonfootContact=false;
  double intervalNonfootForce=0.;
  int contactSensorCount=0;
  bool contactWrenchesAvailable=false;
  std::ofstream trace, network, telemetry;
 public:
  void Configure(const dsim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
      dsim::EntityComponentManager &ecm,dsim::EventManager &) override {
    model=dsim::Model(entity); base=dsim::Link(model.LinkByName(ecm,"BASE_LINK"));
    if(!base.Valid(ecm)) throw std::runtime_error("Missing BASE_LINK");
    base.EnableVelocityChecks(ecm);
    int k=0;
    for(auto leg:{"FR","FL","RR","RL"}) for(auto axis:{"ABAD","HIP","KNEE"}) {
      std::string n=std::string(leg)+"_"+axis+"_JOINT";names.push_back(n);
      joints[k]=dsim::Joint(model.JointByName(ecm,n));
      if(!joints[k].Valid(ecm))throw std::runtime_error("Missing joint "+n);
      joints[k].EnablePositionCheck(ecm);joints[k].EnableVelocityCheck(ecm);
      target[k]=q0[k]=k%3==0?0:k%3==1?.8:-1.5;k++;
    }
    std::ifstream f(sdf->Get<std::string>("actor"),std::ios::binary);
    int dims[5];f.read(reinterpret_cast<char*>(dims),20);
    if(!f || (dims[0]!=45 && dims[0]!=62) || dims[4]!=12)throw std::runtime_error("Invalid policy");
    observationDim=dims[0];
    mean.resize(observationDim);stddev.resize(observationDim);
    f.read(reinterpret_cast<char*>(mean.data()),observationDim*4);f.read(reinterpret_cast<char*>(stddev.data()),observationDim*4);
    for(int i=0;i<4;i++) {
      weight[i].resize(dims[i+1],dims[i]);bias[i].resize(dims[i+1]);
      f.read(reinterpret_cast<char*>(weight[i].data()),dims[i+1]*dims[i]*4);
      f.read(reinterpret_cast<char*>(bias[i].data()),dims[i+1]*4);
    }
    if(!f)throw std::runtime_error("Truncated policy");
    auto out=sdf->Get<std::string>("output");
    if(sdf->HasElement("pd_hz"))pdHz=sdf->Get<double>("pd_hz");
    if(sdf->HasElement("action_clip"))actionClip=sdf->Get<float>("action_clip");
    if(!std::isfinite(actionClip)||actionClip<3.f||actionClip>5.f)throw std::runtime_error("Supported action clip range: 3 through 5");
    if(pdHz!=200 && pdHz!=1000)throw std::runtime_error("Supported PD rates: 200 or 1000 Hz");
    if(sdf->HasElement("pd_experiment"))experiment=sdf->Get<bool>("pd_experiment");
    if(sdf->HasElement("stair_height"))stairHeight=sdf->Get<double>("stair_height");
    if(sdf->HasElement("stair_count"))stairCount=sdf->Get<int>("stair_count");
    if(sdf->HasElement("stair_half_width"))stairHalfWidth=sdf->Get<double>("stair_half_width");
    if(sdf->HasElement("course_origin_x"))originX=sdf->Get<double>("course_origin_x");
    if(sdf->HasElement("course_origin_y"))originY=sdf->Get<double>("course_origin_y");
    if(sdf->HasElement("initial_phase"))phase=sdf->Get<double>("initial_phase");
    if(sdf->HasElement("terrain_file")){
      std::ifstream tf(sdf->Get<std::string>("terrain_file"));nlohmann::json terrain;tf>>terrain;surfaces=terrain.at("surfaces");
    }
    if(experiment) {
      telemetry.open(out+"/telemetry.csv");
      telemetry<<"t,pd_updates,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,cmd_x,cmd_y,cmd_yaw";
      for(auto prefix:{"q","dq","target","tau"})for(int j=0;j<12;j++)telemetry<<','<<prefix<<j;
      telemetry<<'\n';
    }
    trace.open(out+"/trace.csv");
    trace<<"t,x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,cmd_x,cmd_y,cmd_yaw,command_age,max_torque,contact_sensor_count,nonfoot_contact,nonfoot_force,FR_contact,FL_contact,RR_contact,RL_contact,contact_wrenches_available\n";
    context=std::make_shared<rclcpp::Context>();context->init(0,nullptr);
    node=std::make_shared<rclcpp::Node>("d1_policy",rclcpp::NodeOptions().context(context));
    rclcpp::ExecutorOptions eo;eo.context=context;
    executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>(eo);executor->add_node(node);
    input=node->create_subscription<geometry_msgs::msg::Twist>("/d1/cmd_vel",1,[this](const geometry_msgs::msg::Twist::SharedPtr m){
      if(experiment)return; // A/B commands depend only on simulation time.
      if(!std::isfinite(m->linear.x)||!std::isfinite(m->linear.y)||!std::isfinite(m->angular.z))return;
      cmd={std::clamp(m->linear.x,-.5,.5),std::clamp(m->linear.y,-.3,.3),std::clamp(m->angular.z,-.8,.8)};
      lastCommand=now;
    });
    odom=node->create_publisher<nav_msgs::msg::Odometry>("/d1/odom",10);
    jointPub=node->create_publisher<sensor_msgs::msg::JointState>("/d1/joint_states",10);
    clockPub=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::ClockQoS());
    ready=true;
    RCLCPP_INFO(node->get_logger(),"D1 ready: 50 Hz frozen policy -> %.0f Hz joint PD -> Fortress physics; experiment=%d",pdHz,experiment);
  }
  void PreUpdate(const dsim::UpdateInfo &info,dsim::EntityComponentManager &ecm) override {
    if(!ready||info.paused)return;
    now=std::chrono::duration<double>(info.simTime).count();
    if(!initialized) {
      for(int j=0;j<12;j++)joints[j].ResetPosition(ecm,{q0[j]});
      initialized=true;return;
    }
    auto p=base.WorldPose(ecm);auto v=base.WorldLinearVelocity(ecm);auto w=base.WorldAngularVelocity(ecm);
    if(!p||!v||!w)return;
    std::array<double,12> q,dq;
    for(int j=0;j<12;j++) {
      auto pos=joints[j].Position(ecm);auto vel=joints[j].Velocity(ecm);
      if(!pos||!vel||pos->empty()||vel->empty())return;
      q[j]=pos->at(0);dq[j]=vel->at(0);
    }
    if(now+1e-8>=nextPolicy) {
      nextPolicy=now+.02;executor->spin_some();
      if(now-lastCommand>.4)cmd={0,0,0};
      if(experiment) {
        cmd={0,0,0};
        if(now>=2 && now<14)cmd[0]=.4;
        if(now>=17 && now<21)cmd[2]=.4;
        lastCommand=now;
      }
      auto wb=p->Rot().RotateVectorReverse(*w);
      auto vb=p->Rot().RotateVectorReverse(*v);
      auto g=p->Rot().RotateVectorReverse(ignition::math::Vector3d(0,0,-1));
      if(now>=1.0) {
        Eigen::VectorXf obs(observationDim);
        for(int i=0;i<3;i++){obs[i]=wb[i]*.25;obs[3+i]=g[i];obs[6+i]=cmd[i];}
        for(int i=0;i<12;i++){obs[9+i]=q[i]-q0[i];obs[21+i]=dq[i]*.05;obs[33+i]=action[i];}
        if(observationDim==62) {
          obs[45]=std::sin(2*M_PI*phase);obs[46]=std::cos(2*M_PI*phase);
          for(int i=0;i<3;i++)obs[47+i]=vb[i];
          double yaw=p->Rot().Euler().Z();int probe=50;
          for(double x:{0.,.2,.4,.6})for(double y:{-.2,0.,.2}) {
            double wx=p->Pos().X()+std::cos(yaw)*x-std::sin(yaw)*y;
            double wy=p->Pos().Y()+std::sin(yaw)*x+std::cos(yaw)*y;
            double height=0.;
            if(stairHeight>0 && std::abs(wy-originY)<=stairHalfWidth) {
              int step=std::clamp(static_cast<int>(std::floor((wx-originX-stairStart)/stairTread))+1,0,stairCount);
              height=step*stairHeight;
            }
            if(!surfaces.empty()){
              height=-1e20;
              for(const auto &s:surfaces){auto b=s.at("bounds_xy");
                if(wx>=double(b[0])&&wx<=double(b[1])&&wy>=double(b[2])&&wy<=double(b[3]))height=std::max(height,double(s.at("top_z")));
              }
              if(height<-1e19)height=0.;
            }
            obs[probe++]=std::clamp((height-p->Pos().Z()+.32)*3.,-3.,3.);
          }
        }
        obs=obs.cwiseMax(-100.f).cwiseMin(100.f);
        Eigen::VectorXf y=(obs-mean).cwiseQuotient(stddev);
        for(int l=0;l<4;l++) {
          y=(weight[l]*y+bias[l]).eval();
          if(l<3)for(int j=0;j<y.size();j++)if(y[j]<0)y[j]=std::expm1(y[j]);
        }

        action=y.cwiseMax(-actionClip).cwiseMin(actionClip);
        for(int i=0;i<12;i++)target[i]=q0[i]+action[i]*(i%3==0?.125:.25);
        if(observationDim==62 && std::sqrt(cmd[0]*cmd[0]+cmd[1]*cmd[1]+cmd[2]*cmd[2])>.05)
          phase=std::fmod(phase+.02*1.8,1.);
      }
      builtin_interfaces::msg::Time stamp=rclcpp::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
      rosgraph_msgs::msg::Clock c;c.clock=stamp;clockPub->publish(c);
      nav_msgs::msg::Odometry o;o.header.stamp=stamp;o.header.frame_id="world";o.child_frame_id="BASE_LINK";
      o.pose.pose.position.x=p->Pos().X();o.pose.pose.position.y=p->Pos().Y();o.pose.pose.position.z=p->Pos().Z();
      o.pose.pose.orientation.w=p->Rot().W();o.pose.pose.orientation.x=p->Rot().X();o.pose.pose.orientation.y=p->Rot().Y();o.pose.pose.orientation.z=p->Rot().Z();
      o.twist.twist.linear.x=vb.X();o.twist.twist.linear.y=vb.Y();o.twist.twist.linear.z=vb.Z();
      o.twist.twist.angular.x=wb.X();o.twist.twist.angular.y=wb.Y();o.twist.twist.angular.z=wb.Z();odom->publish(o);
      sensor_msgs::msg::JointState js;js.header.stamp=stamp;js.name=names;
      js.position.assign(q.begin(),q.end());js.velocity.assign(dq.begin(),dq.end());js.effort.assign(torque.begin(),torque.end());jointPub->publish(js);
      double maxTau=0;for(auto t:torque)maxTau=std::max(maxTau,std::abs(t));
      auto rpy=p->Rot().Euler();
      trace<<std::setprecision(10)<<now<<','<<p->Pos().X()<<','<<p->Pos().Y()<<','<<p->Pos().Z()<<','<<rpy.X()<<','<<rpy.Y()<<','<<rpy.Z()<<','<<vb.X()<<','<<vb.Y()<<','<<vb.Z()<<','<<wb.X()<<','<<wb.Y()<<','<<wb.Z()<<','<<cmd[0]<<','<<cmd[1]<<','<<cmd[2]<<','<<now-lastCommand<<','<<maxTau<<','<<contactSensorCount<<','<<intervalNonfootContact<<','<<intervalNonfootForce;
      for(bool c:intervalFootContact)trace<<','<<c;
      trace<<','<<contactWrenchesAvailable<<'\n';intervalFootContact.fill(false);intervalNonfootContact=false;intervalNonfootForce=0.;
      trace.flush();network.flush();if(experiment)telemetry.flush();
    }
    if(now+1e-8>=nextPD) {
      // Common 200 Hz stand-up for paired experiments; only locomotion PD differs.
      nextPD=now+((experiment && now<1)? .005 : 1./pdHz);
      ++pdUpdates;
      for(int j=0;j<12;j++)torque[j]=std::clamp((now<1?80.:20.)*(target[j]-q[j])-(now<1?3.:.8)*dq[j],-33.5,33.5);
    }
    for(int j=0;j<12;j++)joints[j].SetForce(ecm,{torque[j]});
    if(experiment) {
      auto rpy=p->Rot().Euler();
      auto vb=p->Rot().RotateVectorReverse(*v);auto wb=p->Rot().RotateVectorReverse(*w);
      telemetry<<std::setprecision(10)<<now<<','<<pdUpdates<<','<<p->Pos().X()<<','<<p->Pos().Y()<<','<<p->Pos().Z()<<','<<rpy.X()<<','<<rpy.Y()<<','<<rpy.Z()<<','<<vb.X()<<','<<vb.Y()<<','<<vb.Z()<<','<<wb.X()<<','<<wb.Y()<<','<<wb.Z()<<','<<cmd[0]<<','<<cmd[1]<<','<<cmd[2];
      for(auto &a:{q,dq,target,torque})for(auto val:a)telemetry<<','<<val;
      telemetry<<'\n';
    }
  }
  void PostUpdate(const dsim::UpdateInfo &info,const dsim::EntityComponentManager &ecm) override {
    if(!ready||info.paused)return;
    int count=0;
    ecm.Each<dsim::components::ContactSensorData>([&](const dsim::Entity &,const dsim::components::ContactSensorData *data) {
      ++count;
      for(const auto &contact:data->Data().contact()) {
        if(contact.position_size()==0)continue;
        double force=0.;
        double fx=0,fy=0,fz=0,gx=0,gy=0,gz=0;
        for(const auto &w:contact.wrench()) {
          fx+=w.body_1_wrench().force().x();fy+=w.body_1_wrench().force().y();fz+=w.body_1_wrench().force().z();
          gx+=w.body_2_wrench().force().x();gy+=w.body_2_wrench().force().y();gz+=w.body_2_wrench().force().z();
        }
        force=std::max(std::sqrt(fx*fx+fy*fy+fz*fz),std::sqrt(gx*gx+gy*gy+gz*gz));
        contactWrenchesAvailable=contactWrenchesAvailable||contact.wrench_size()>0;
        for(const auto *collision:{&contact.collision1(),&contact.collision2()}) {
          std::string name=collision->name();
          if(name.empty()) {
            auto comp=ecm.Component<dsim::components::Name>(collision->id());
            if(comp)name=comp->Data();
          }
          if(name.find("_collision_")!=std::string::npos) {
            intervalNonfootContact=intervalNonfootContact||(contact.wrench_size()==0||force>5.);
            intervalNonfootForce=std::max(intervalNonfootForce,force);
          }
          if(name.find("_foot_")!=std::string::npos) {
            int leg=0;
            for(auto prefix:{"FR_","FL_","RR_","RL_"}) {
              if(name.find(prefix)!=std::string::npos)intervalFootContact[leg]=true;
              ++leg;
            }
          }
        }
      }
      return true;
    });
    contactSensorCount=count;
  }
};
IGNITION_ADD_PLUGIN(D1CompetitionPolicy,dsim::System,D1CompetitionPolicy::ISystemConfigure,D1CompetitionPolicy::ISystemPreUpdate,D1CompetitionPolicy::ISystemPostUpdate)
