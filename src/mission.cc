// Competition supervisor. D1 receives ROS 2 Twist only. UAV uses force PD.
// Released balls/parcels receive an initial velocity once, then free physics.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Root.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
#include <set>
#include <cmath>
#include <iostream>
using json=nlohmann::json;
namespace sim=ignition::gazebo;
using Vec=ignition::math::Vector3d;
using Pose=ignition::math::Pose3d;
static double clip(double x,double a,double b){return std::clamp(x,a,b);}
static double wrap(double x){return std::atan2(std::sin(x),std::cos(x));}
static double xy(const Vec &v){return std::hypot(v.X(),v.Y());}

class D1Mission: public sim::System,public sim::ISystemConfigure,public sim::ISystemPreUpdate,public sim::ISystemPostUpdate {
  sim::Entity world=sim::kNullEntity;
  sim::Model dog{sim::kNullEntity},uav{sim::kNullEntity};
  sim::Link base{sim::kNullEntity},air{sim::kNullEntity};
  std::unique_ptr<sim::SdfEntityCreator> creator;
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commands;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statePub;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services;
  bool student=false, studentBusy=false;
  std::string pendingAction;
  Vec currentPosition;
  double actionStarted=0;
  std::set<int> solved;
  json config,route,meta,events=json::array(),stages=json::array(),tasks=json::array();
  std::string out;int stage=0,active=-1,completed=0,delivered=0,waypoint=0;
  double now=0,start=0,next=0,nextLog=0,hold=0,landingHold=0,maxDrift=0;
  Vec stageStart,command;bool entered=false,finished=false,scout=false,landedUav=false;
  bool fired=false,dropped=false,hit=false,parcelLanded=false,landing=false;
  std::vector<bool> reported;std::vector<std::string> released;
  std::string ballName,parcelName;
  sim::Entity ball=sim::kNullEntity,parcel=sim::kNullEntity;
  Vec ballVelocity;bool initializeBall=false,initializeParcel=false;
  double ballSpawnTime=-100.;bool ballRetired=false;
  std::vector<sim::Entity> velocityCommands;
  std::ofstream actors,missionTrace;
  bool haveEntryPose=false,airEntering=false,dogEntering=false,airLeaving=false;
  bool airEntered=false,dogEntered=false,airExited=false;
  Vec previousAir,previousDog;
  int returnWaypoint=0;
  std::string airPhase="takeoff";
  bool requiresWindowExit()const{
    return config.value("landing_mode",std::string("outside"))!="upper_rally";
  }
  bool dogAtRally(const sim::EntityComponentManager &ecm)const{
    auto p=base.WorldPose(ecm);auto v=base.WorldLinearVelocity(ecm);auto q=config["rally_point"];
    return p&&v&&std::hypot(p->Pos().X()-double(q[0]),p->Pos().Y()-double(q[1]))<.15
      &&std::abs(p->Pos().Z()-(config.value("rally_surface_z",.8)+.32))<.15&&xy(*v)<.05;
  }
  bool insideBuilding(const Vec &p)const{
    const auto b=config["entry"]["bounds_xy"];
    return p.X()>double(b[0])&&p.X()<double(b[1])&&p.Y()>double(b[2])&&p.Y()<double(b[3]);
  }
  bool throughFront(const Vec &a,const Vec &b,bool entering)const{
    const double front=config["entry"]["bounds_xy"][2];
    return entering?(a.Y()<=front&&b.Y()>front):(a.Y()>=front&&b.Y()<front);
  }
  Vec facadeIntersection(const Vec &a,const Vec &b)const{
    double y=config["entry"]["bounds_xy"][2];
    return a+(b-a)*((y-a.Y())/(b.Y()-a.Y()));
  }
  bool inFireWindow(const Vec &p)const{
    const auto e=config["entry"],w=e["selected_window"];
    return std::abs(p.X()-double(w["center_x"]))<=double(w["width"])/2-double(e["uav_margin_xy"])
      &&std::abs(p.Z()-double(w["center_z"]))<=double(w["height"])/2-double(e["uav_margin_z"]);
  }
  void observeEntry(sim::EntityComponentManager &ecm){
    auto ap=air.WorldPose(ecm),dp=base.WorldPose(ecm);if(!ap||!dp||ap->Pos().Length()<.1||dp->Pos().Length()<.1)return;
    const Vec a=ap->Pos(),d=dp->Pos();const auto e=config["entry"],door=e["door"];
    const double front=e["bounds_xy"][2],halfWall=double(e["wall_thickness"])/2;
    if(haveEntryPose){
      if(!insideBuilding(previousAir)&&insideBuilding(a)){
        if(airEntered||!throughFront(previousAir,a,true)||!inFireWindow(facadeIntersection(previousAir,a))){finish("uav_invalid_window_entry",ecm);return;}
        airEntering=true;event("uav_window_crossed",{{"window_id",e["fire_window_id"]},{"position",{a.X(),a.Y(),a.Z()}}});
      }
      if(insideBuilding(previousAir)&&!insideBuilding(a)){
        if(!requiresWindowExit()||!airEntered||completed!=int(config["tasks"].size())||!throughFront(previousAir,a,false)||!inFireWindow(facadeIntersection(previousAir,a))){finish("uav_invalid_window_exit",ecm);return;}
        airLeaving=true;event("uav_exit_window_crossed",{{"window_id",e["fire_window_id"]}});
      }
      if(!insideBuilding(previousDog)&&insideBuilding(d)){
        double yaw=dp->Rot().Yaw();
        double radius=std::abs(std::cos(yaw))*double(e["dog_half_length"])+std::abs(std::sin(yaw))*double(e["dog_half_width"]);
        bool frontCross=throughFront(previousDog,d,true);auto p=frontCross?facadeIntersection(previousDog,d):d;
        if(!airEntered||!scout||dogEntered||!frontCross||std::abs(p.X()-double(door["center_x"]))+radius>double(door["width"])/2||p.Z()<.15||p.Z()+.44>double(door["height"])){
          finish("d1_invalid_door_entry",ecm);return;
        }
        dogEntering=true;event("d1_door_crossed",{{"position",{p.X(),p.Y(),p.Z()}}});
      }
      if(dogEntered&&insideBuilding(previousDog)&&!insideBuilding(d)){finish("d1_left_building",ecm);return;}
    }
    if(airEntering&&!airEntered&&a.Y()>front+halfWall+double(e["uav_margin_xy"])+.04){airEntered=true;event("uav_entered_building",{{"window_id",e["fire_window_id"]}});}
    if(dogEntering&&!dogEntered){
      double yaw=dp->Rot().Yaw();double radius=std::abs(std::sin(yaw))*double(e["dog_half_length"])+std::abs(std::cos(yaw))*double(e["dog_half_width"]);
      if(d.Y()>front+halfWall+radius+.04){dogEntered=true;event("d1_entered_building");}
    }
    if(airLeaving&&!airExited&&a.Y()<front-halfWall-double(e["uav_margin_xy"])-.04){airExited=true;event("uav_exited_building");}
    previousAir=a;previousDog=d;haveEntryPose=true;
  }
  void event(const std::string &name,json extra=json::object()){
    extra["event"]=name;extra["time"]=now;if(active>=0&&!extra.contains("task"))extra["task"]=active;
    events.push_back(extra);
  }
  sim::Entity named(const std::string &n,const sim::EntityComponentManager &ecm)const{
    return ecm.EntityByComponents(sim::components::Model(),sim::components::Name(n));
  }
  bool inModel(sim::Entity e,sim::Entity m,const sim::EntityComponentManager &ecm)const{
    for(int i=0;e!=sim::kNullEntity&&i<8;i++,e=ecm.ParentEntity(e))if(e==m)return true;
    return false;
  }
  bool contactModel(const ignition::msgs::Entity &c,sim::Entity m,const std::string &name,const sim::EntityComponentManager &ecm)const{
    return inModel(c.id(),m,ecm)||c.name().find(name+"::")!=std::string::npos;
  }
  bool contactLink(const ignition::msgs::Entity &c,const std::string &name,const sim::EntityComponentManager &ecm)const{
    for(auto e=sim::Entity(c.id());e!=sim::kNullEntity;e=ecm.ParentEntity(e)){
      auto n=ecm.Component<sim::components::Name>(e);if(n&&(n->Data()==name||n->Data()==name+"_collision"))return true;
    }
    return c.name()==name+"_collision"||c.name().find("::"+name+"::")!=std::string::npos;
  }
  double plateAngle(int task,const sim::EntityComponentManager &ecm)const{
    auto e=named("demo_target_"+std::to_string(int(config["tasks"][task]["slot"])),ecm);
    if(e==sim::kNullEntity)return 0.;
    auto p=sim::Joint(sim::Model(e).JointByName(ecm,"knockdown_hinge")).Position(ecm);
    return p&&!p->empty()?p->at(0):0.;
  }
  void enter(const sim::EntityComponentManager &ecm){
    if(stage>=int(route.size()))return;
    int task=meta[stage].value("task",-1);
    if(task>=0&&task!=active){active=task;fired=dropped=hit=parcelLanded=false;ball=parcel=sim::kNullEntity;}
    stageStart=base.WorldPose(ecm)->Pos();start=now;maxDrift=0;entered=true;
    std::cout<<std::fixed<<std::setprecision(2)<<now<<" stage "<<stage<<" "<<meta[stage]["label"]<<std::endl;
    event("stage_enter",{{"stage",stage},{"label",meta[stage]["label"]},{"kind",route[stage][0]}});
  }
  void finish(const std::string &reason,const sim::EntityComponentManager &ecm){
    if(finished)return;finished=true;command=Vec::Zero;
    geometry_msgs::msg::Twist msg;commands->publish(msg);
    auto p=base.WorldPose(ecm).value_or(Pose());
    bool entryPass=airEntered&&dogEntered&&(!requiresWindowExit()||airExited);
    json result={{"task_completed",reason.empty()},{"strict_pass",reason.empty()&&completed==int(config["tasks"].size())&&entryPass&&landedUav&&dogAtRally(ecm)},
      {"failure",reason},{"simulation_seconds",now},{"stage",stage},{"completed_count",completed},
      {"position",{p.Pos().X(),p.Pos().Y(),p.Pos().Z()}},{"yaw",p.Rot().Yaw()},
      {"task_results",tasks},{"mission_events",events},{"stages",stages},{"uav_landed",landedUav},
      {"entry_validation",{{"passed",entryPass},{"fire_window_id",config["entry"]["fire_window_id"]},{"uav_entered",airEntered},{"d1_entered",dogEntered},{"uav_exited",airExited},{"window_exit_required",requiresWindowExit()}}},
      {"rally_validation",{{"dog_at_rally",dogAtRally(ecm)},{"uav_landed",landedUav},{"dog_point",config["rally_point"]},{"uav_point",config["landing_point"]},{"uav_surface_z",config.value("landing_z",0.)}}},
      {"robot_pose_overrides_after_initialization",0},{"external_lift_forces",0},
      {"controller","frozen D1 phase_joint_v12 policy; 50 Hz inference / 1000 Hz PD"},
      {"mission_controller","preset waypoints and known-map observations; no onboard image autonomy"}};
    std::ofstream(out+"/result.json")<<result.dump(2)<<'\n';actors.flush();missionTrace.flush();
    std::cout<<"Mission finished: "<<reason<<" completed="<<completed<<std::endl;
  }
  sim::Entity spawn(const std::string &name,bool sphere,const Pose &pose,sim::EntityComponentManager &ecm){
    std::ostringstream s;
    s<<"<sdf version='1.8'><model name='"<<name<<"'><pose>"<<pose<<"</pose><link name='body'><inertial><mass>"<<(sphere?.003:.05)<<"</mass><inertia><ixx>.00001</ixx><iyy>.00001</iyy><izz>.00001</izz></inertia></inertial>";
    std::string shape=sphere?"<sphere><radius>.022</radius></sphere>":"<box><size>.05 .044 .03</size></box>";
    s<<"<collision name='object'><geometry>"<<shape<<"</geometry><surface><friction><ode><mu>.7</mu><mu2>.7</mu2></ode></friction></surface></collision><visual name='visible'><geometry>"<<shape<<"</geometry><material><ambient>1 .65 .05 1</ambient><diffuse>1 .65 .05 1</diffuse></material></visual></link></model></sdf>";
    sdf::Root root;auto errors=root.LoadSdfString(s.str());if(!errors.empty())throw std::runtime_error(errors.front().Message());
    auto entity=creator->CreateEntities(root.Model());creator->SetParent(entity,world);
    auto link=sim::Link(sim::Model(entity).LinkByName(ecm,"body"));link.EnableVelocityChecks(ecm);
    released.push_back(name);return entity;
  }
  void fire(sim::EntityComponentManager &ecm){
    fired=true;ballSpawnTime=now;ballRetired=false;auto p=base.WorldPose(ecm).value();ballName="foam_ball_"+std::to_string(active)+(student?"_"+std::to_string(int(now*1000)):"");
    Pose muzzle(p.CoordPositionAdd({.20,0,.3973565}),p.Rot());
    ballVelocity=p.Rot().RotateVector({4*std::cos(M_PI/18),0,4*std::sin(M_PI/18)});
    ball=spawn(ballName,true,muzzle,ecm);initializeBall=true;
    event("fire",{{"model",ballName},{"speed",4.}});
  }
  void drop(sim::EntityComponentManager &ecm){
    dropped=true;auto p=base.WorldPose(ecm).value();parcelName="cargo_"+std::to_string(active)+(student?"_"+std::to_string(int(now*1000)):"");
    parcel=spawn(parcelName,false,Pose(p.CoordPositionAdd({-.42,0,.237}),p.Rot()),ecm);
    initializeParcel=true;event("drop",{{"model",parcelName}});++delivered;
    auto visual=ecm.EntityByComponents(sim::components::Name("payload_parcel_"+std::to_string(delivered-1)));
    if(visual!=sim::kNullEntity)ecm.RequestRemoveEntity(visual,true);
  }
  void studentStep(sim::EntityComponentManager &ecm){
    if(!pendingAction.empty()){
      actionStarted=now;
      if(pendingAction=="fire")fire(ecm);else drop(ecm);
      pendingAction.clear();
    }
    if(studentBusy&&now-actionStarted>=5.){
      bool ok=false;double error=-1.;
      if(fired)ok=hit&&plateAngle(active,ecm)>=1.;
      if(dropped){
        auto p=sim::Link(sim::Model(parcel).LinkByName(ecm,"body")).WorldPose(ecm);
        if(p){auto t=config["tasks"][active]["delivery_point"];error=std::hypot(p->Pos().X()-double(t[0]),p->Pos().Y()-double(t[1]));ok=parcelLanded&&error<=.2;}
      }
      if(ok){solved.insert(active);++completed;tasks.push_back({{"task",active},{"success",true},{"time",now},{"delivery_error_m",error}});}
      event(ok?"task_complete":"task_attempt_failed",{{"delivery_error_m",error}});studentBusy=false;
    }
    if(now>900.)finish("time_limit",ecm);
    if(completed==int(config["tasks"].size())&&landedUav&&dogAtRally(ecm))finish("",ecm);
  }
  void requestAction(const std::string &kind,std_srvs::srv::Trigger::Response &res){
    if(!student||finished||studentBusy){res.message="Requires idle student mode";return;}
    if(!dogEntered){res.message="D1 must enter through the door after the first UAV target report";return;}
    if(kind=="drop"&&delivered>=3){res.message="No parcels remaining";return;}
    int best=-1;double distance=2.;
    for(int i=0;i<int(config["tasks"].size());++i){
      if(solved.count(i)||config["tasks"][i]["color"]!=(kind=="fire"?"red":"green"))continue;
      auto q=config["tasks"][i]["position"];
      if(std::abs(currentPosition.Z()-.32-double(q[2]))>.3)continue;
      double d=std::hypot(currentPosition.X()-double(q[0]),currentPosition.Y()-double(q[1]));
      if(d<distance){best=i;distance=d;}
    }
    if(best<0){res.message="No unfinished matching target within 2 m on this floor";return;}
    active=best;fired=dropped=hit=parcelLanded=false;ball=parcel=sim::kNullEntity;
    studentBusy=true;pendingAction=kind;res.success=true;res.message="Accepted for "+config["tasks"][best]["id"].get<std::string>()+"; inspect /competition/state after 5 simulated seconds";
  }
  void drive(sim::EntityComponentManager &ecm){
    if(!entered)enter(ecm);if(stage>=int(route.size())){finish("",ecm);return;}
    const auto r=route[stage];std::string kind=r[0],failure;bool done=false;
    double elapsed=now-start;auto p=base.WorldPose(ecm).value();double yaw=p.Rot().Yaw();command=Vec::Zero;
    if(kind=="walk"){
      double heading=r[3],cs=std::cos(heading),sn=std::sin(heading);
      double remain=(double(r[1])-p.Pos().X())*cs+(double(r[2])-p.Pos().Y())*sn;
      double cross=-(p.Pos().X()-double(r[1]))*sn+(p.Pos().Y()-double(r[2]))*cs;
      double tolerance=meta[stage].value("task",-1)>=0?.04:.08;
      maxDrift=std::max(maxDrift,std::abs(cross));done=remain<tolerance&&std::abs(cross)<.15;
      double speed=meta[stage].value("view",std::string("walk"))=="stairs"?.22:.30;
      double f=clip(remain*.8,.065,speed),side=clip(-cross*.7,-.10,.10);
      double wx=cs*f-sn*side,wy=sn*f+cs*side;
      command.Set(std::cos(yaw)*wx+std::sin(yaw)*wy,-std::sin(yaw)*wx+std::cos(yaw)*wy,clip(1.5*wrap(heading-yaw),-.5,.5));
      if(maxDrift>.45)failure="route_drift";if(elapsed>double(r[5]))failure="walk_timeout";
      if(done&&p.Pos().Z()<double(r[4])+.23)failure="wrong_floor";
    }else if(kind=="turn"){
      double e=wrap(double(r[1])-yaw);maxDrift=std::max(maxDrift,xy(p.Pos()-stageStart));
      double w=clip(1.5*e,-.5,.5);if(std::abs(e)>.055&&std::abs(w)<.13)w=std::copysign(.13,e);
      command.Z()=w;done=std::abs(e)<.055;
      if(maxDrift>.35)failure="turn_drift";if(elapsed>30)failure="turn_timeout";
    }else if(kind=="scout"){done=scout;if(elapsed>double(r[1]))failure="scout_timeout";}
    else if(kind=="await_target"){done=reported.at(int(r[1]));if(elapsed>double(r[2]))failure="target_report_timeout";}
    else if(kind=="wait")done=elapsed>=double(r[1]);
    else if(kind=="fire"||kind=="drop"){
      if(elapsed>1.){if(kind=="fire"&&!fired)fire(ecm);if(kind=="drop"&&!dropped)drop(ecm);}
      done=elapsed>=double(r[1]);
      if(done){
        if(kind=="fire"&&(!hit||plateAngle(active,ecm)<1.))failure="target_not_knocked_down";
        double error=-1.;
        if(kind=="drop"){
          auto pp=sim::Link(sim::Model(parcel).LinkByName(ecm,"body")).WorldPose(ecm);
          if(!pp||!parcelLanded)failure="parcel_not_landed";
          else {auto t=config["tasks"][active]["delivery_point"];error=std::hypot(pp->Pos().X()-double(t[0]),pp->Pos().Y()-double(t[1]));if(error>.2)failure="delivery_accuracy";}
        }
        if(failure.empty()){
          ++completed;tasks.push_back({{"task",active},{"success",true},{"time",now},{"color",config["tasks"][active]["color"]},{"delivery_error_m",error}});
          event("task_complete");
        }
      }
    }else if(kind=="finish"){
      done=landedUav&&elapsed>=3&&dogAtRally(ecm);
      if(elapsed>double(r[1]))failure="landing_timeout";
    }else {done=elapsed>=3;}
    if(done&&(kind=="walk"||kind=="turn")&&maxDrift>.2)failure="route_accuracy";
    if(!failure.empty()){finish(failure,ecm);return;}
    if(done){
      stages.push_back({{"stage",stage},{"kind",kind},{"duration",elapsed},{"max_drift",maxDrift},{"position",{p.Pos().X(),p.Pos().Y(),p.Pos().Z()}}});
      ++stage;entered=false;command=Vec::Zero;if(stage>=int(route.size()))finish("",ecm);else enter(ecm);
    }
    geometry_msgs::msg::Twist msg;msg.linear.x=command.X();msg.linear.y=command.Y();msg.angular.z=command.Z();commands->publish(msg);
  }
  void fly(double dt,sim::EntityComponentManager &ecm){
    auto p=air.WorldPose(ecm);auto v=air.WorldLinearVelocity(ecm);auto w=air.WorldAngularVelocity(ecm);if(!p||!v||!w)return;
    Vec goal(2.6,.85,3.45);landing=false;double speed=.8;
    if(now<1.){auto q=config["uav_initial_pose"];goal.Set(q[0],q[1],q[2]);}
    else if(waypoint<int(config["patrol"].size())){
      auto wp=config["patrol"][waypoint],q=wp["position"];goal.Set(q[0],q[1],q[2]);
      airPhase=wp.value("phase",std::string("indoor_search"));speed=wp.value("speed",.8);
      if((goal-p->Pos()).Length()<.07&&v->Length()<.08)hold+=dt;else hold=0;
      if(hold>=.7){
        event("uav_waypoint_reached",{{"index",waypoint}});int task=wp["task"];
        if(task>=0){if(!airEntered){finish("target_report_before_window_entry",ecm);return;}reported.at(task)=true;event("teacher_target_report",{{"task",task}});if(!scout){scout=true;event("scout_ready");}}
        ++waypoint;hold=0;
      }
    }else if(completed==int(config["tasks"].size())){
      const auto &path=config["egress"];int i=std::min(returnWaypoint,int(path.size())-1);
      auto wp=path[i],q=wp["position"];goal.Set(q[0],q[1],q[2]);speed=wp.value("speed",.8);airPhase=wp["phase"];
      landing=airPhase=="landing";
      if(!landing){
        if((goal-p->Pos()).Length()<.07&&v->Length()<.08)hold+=dt;else hold=0;
        if(hold>=.7){event("uav_return_waypoint_reached",{{"index",i},{"phase",airPhase}});++returnWaypoint;hold=0;}
      }
    }else if(active>=0){airPhase="indoor_escort";auto q=config["tasks"][active]["position"];goal.Set(q[0],q[1],3.45);}
    else{
      // A student may still be planning after the scout finishes its patrol.
      // Wait inside instead of falling back to the outdoor takeoff position.
      auto q=config["patrol"].back()["position"];goal.Set(q[0],q[1],q[2]);airPhase="indoor_waiting";
    }
    if(!landedUav){
      auto e=goal-p->Pos();Vec desired(clip(e.X(),-speed,speed),clip(e.Y(),-speed,speed),clip(e.Z(),-.45,.45));
      auto acc=3.5*(desired-*v);for(int i=0;i<3;i++)acc[i]=clip(acc[i],-2.,2.);
      // AddWorldWrench takes a world-frame torque. Euler roll/pitch errors
      // cannot be used directly after yawing the body to face the facade.
      constexpr double heading=M_PI/2;
      double forward=std::cos(heading)*acc.X()+std::sin(heading)*acc.Y();
      double left=-std::sin(heading)*acc.X()+std::cos(heading)*acc.Y();
      ignition::math::Quaterniond desiredAttitude(-left/9.80665,forward/9.80665,heading);
      auto error=desiredAttitude*p->Rot().Inverse();
      auto rotationError=2.*(error.W()<0?-1.:1.)*Vec(error.X(),error.Y(),error.Z());
      auto torque=.004*rotationError-.001*(*w);
      air.AddWorldWrench(ecm,config.value("uav_mass_kg",.085)*(acc+Vec(0,0,9.80665)),torque);
    }
    for(int i=0;i<4;i++){
      auto e=uav.JointByName(ecm,"rotor_"+std::to_string(i)+"_joint");if(e!=sim::kNullEntity)sim::Joint(e).SetVelocity(ecm,{landedUav?0.:(i<2?180.:-180.)});
    }
  }
  void log(const sim::EntityComponentManager &ecm){
    auto p=base.WorldPose(ecm);if(!p||p->Pos().Length()<.1)return;
    std::string phase=stage<int(route.size())?route[stage][0].get<std::string>():"complete";
    std::string view=stage<int(meta.size())?meta[stage].value("view",std::string("walk")):"finish";
    json state={{"time",now},{"stage",stage},{"phase",phase},{"view",view},{"stage_elapsed",now-start},{"active_task",active},
      {"completed_count",completed},{"delivered_count",delivered},{"hit",hit},{"dropped",dropped},{"landed",parcelLanded},{"uav_landed",landedUav},{"events",events}};
    state["mode"]=student?"student":"demo";state["targets"]=config["tasks"];state["reported"]=reported;state["task_results"]=tasks;state["action_busy"]=studentBusy;state["finished"]=finished;
    state["entry"]=config["entry"];state["uav_entered"]=airEntered;state["d1_entered"]=dogEntered;state["uav_exited"]=airExited;state["uav_phase"]=airPhase;
    std_msgs::msg::String feedback;feedback.data=state.dump();statePub->publish(feedback);
    std::ofstream(out+"/mission_state.tmp")<<state.dump()<<'\n';std::rename((out+"/mission_state.tmp").c_str(),(out+"/mission_state.json").c_str());
    json row={{"time",now},{"stage",stage},{"completed_count",completed},{"active_task",active}};
    std::vector<std::string> names=released;names.push_back("uav");names.push_back("d1");
    for(const auto &name:names){auto e=named(name,ecm);if(e==sim::kNullEntity)continue;sim::Model m(e);
      std::string ln=name=="d1"?"BASE_LINK":name=="uav"?"base_link":"body";
      auto pp=sim::Link(m.LinkByName(ecm,ln)).WorldPose(ecm);if(!pp||pp->Pos().Length()<.1)continue;
      row[name]={pp->Pos().X(),pp->Pos().Y(),pp->Pos().Z(),pp->Rot().W(),pp->Rot().X(),pp->Rot().Y(),pp->Rot().Z()};
    }
    row["plate_angles"]=json::array();for(int i=0;i<int(config["tasks"].size());i++)row["plate_angles"].push_back(plateAngle(i,ecm));
    actors<<row.dump()<<'\n';actors.flush();
    missionTrace<<std::setprecision(12)<<now<<','<<stage<<','<<p->Pos().X()<<','<<p->Pos().Y()<<','<<p->Pos().Z()<<','<<completed<<'\n';missionTrace.flush();
  }
 public:
  void Configure(const sim::Entity &e,const std::shared_ptr<const sdf::Element> &sdf,sim::EntityComponentManager &ecm,sim::EventManager &manager)override{
    world=e;std::ifstream(sdf->Get<std::string>("config"))>>config;out=config["output"];route=config["route"];meta=config["stage_meta"];
    student=config.value("mode",std::string("demo"))=="student";reported.resize(config["tasks"].size(),false);creator=std::make_unique<sim::SdfEntityCreator>(ecm,manager);
    context=std::make_shared<rclcpp::Context>();context->init(0,nullptr);node=std::make_shared<rclcpp::Node>("d1_competition",rclcpp::NodeOptions().context(context));
    rclcpp::ExecutorOptions options;options.context=context;
    executor=std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);executor->add_node(node);
    commands=node->create_publisher<geometry_msgs::msg::Twist>("/d1/cmd_vel",1);
    statePub=node->create_publisher<std_msgs::msg::String>("/competition/state",rclcpp::QoS(1).transient_local());
    for(std::string kind:{"fire","drop"})services.push_back(node->create_service<std_srvs::srv::Trigger>("/d1/"+kind,[this,kind](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){requestAction(kind,*response);}));
    actors.open(out+"/actors.jsonl");missionTrace.open(out+"/mission_trace.csv");missionTrace<<"time,stage,x,y,z,completed_count\n";
  }
  void PreUpdate(const sim::UpdateInfo &info,sim::EntityComponentManager &ecm)override{
    if(info.paused)return;now=std::chrono::duration<double>(info.simTime).count();double dt=std::chrono::duration<double>(info.dt).count();
    if(!base.Valid(ecm)){
      auto d=named("d1",ecm),a=named("uav",ecm);if(d==sim::kNullEntity||a==sim::kNullEntity)return;
      dog=sim::Model(d);uav=sim::Model(a);base=sim::Link(dog.LinkByName(ecm,"BASE_LINK"));air=sim::Link(uav.LinkByName(ecm,"base_link"));
      base.EnableVelocityChecks(ecm);air.EnableVelocityChecks(ecm);
      for(int i=0;i<int(config["tasks"].size());i++){
        auto t=named("demo_target_"+std::to_string(int(config["tasks"][i]["slot"])),ecm);
        if(t!=sim::kNullEntity)sim::Joint(sim::Model(t).JointByName(ecm,"knockdown_hinge")).EnablePositionCheck(ecm);
      }
    }
    if(!base.WorldPose(ecm)||!base.WorldLinearVelocity(ecm))return;
    if(!finished)observeEntry(ecm);
    // VelocityCmd is persistent in Fortress. Remove it on the next physics
    // tick so gravity and contact impulses control the released body freely.
    for(auto entity:velocityCommands)ecm.RemoveComponent<sim::components::LinearVelocityCmd>(entity);
    velocityCommands.clear();
    // Spent training balls expire after four simulated seconds. Keep the
    // complete ballistic flight, impact and knockdown; do not simulate stray
    // milligram projectiles indefinitely beyond the finite arena ground.
    if((hit||student)&&!ballRetired&&now-ballSpawnTime>4.&&ball!=sim::kNullEntity){ecm.RequestRemoveEntity(ball,true);ballRetired=true;event("spent_projectile_retired",{{"model",ballName},{"lifetime_s",4.}});}
    if(initializeBall){auto link=sim::Link(sim::Model(ball).LinkByName(ecm,"body"));auto p=link.WorldPose(ecm);if(p&&p->Pos().Length()>.1){link.SetLinearVelocity(ecm,p->Rot().RotateVectorReverse(ballVelocity));velocityCommands.push_back(link.Entity());initializeBall=false;event("ball_released",{{"model",ballName}});}}
    if(initializeParcel){auto link=sim::Link(sim::Model(parcel).LinkByName(ecm,"body"));auto p=link.WorldPose(ecm);if(p&&p->Pos().Length()>.1){link.SetLinearVelocity(ecm,p->Rot().RotateVectorReverse(base.WorldLinearVelocity(ecm).value_or(Vec::Zero)));velocityCommands.push_back(link.Entity());initializeParcel=false;event("parcel_released",{{"model",parcelName}});}}
    if(!finished){
      auto p=base.WorldPose(ecm).value();if(now>1.5&&(p.Rot().RotateVector(Vec(0,0,1)).Z()<.55||p.Pos().Z()<.15)){finish("fall",ecm);}
      fly(dt,ecm);if(!finished&&now+1e-8>=next){next=now+.02;currentPosition=base.WorldPose(ecm)->Pos();executor->spin_some();if(student)studentStep(ecm);else drive(ecm);}
    }
    if(now+1e-8>=nextLog){nextLog=now+.02;log(ecm);}
    // Only massless inventory visuals move on the conveyor. The parcel gets
    // its own physical body at the outlet and never receives pose commands.
    bool feeding=stage<int(route.size())&&route[stage][0]=="drop"&&!dropped;
    double feed=feeding?clip(now-start,0.,1.)*.093:0.;
    for(int i=delivered;i<3;i++){
      auto v=ecm.EntityByComponents(sim::components::Name("payload_parcel_"+std::to_string(i)));
      if(v!=sim::kNullEntity)ecm.SetComponentData<sim::components::Pose>(v,Pose(-.327+.09*(i-delivered)-feed,0,.237,0,0,0));
    }
    auto camera=named("dog_camera",ecm);if(camera!=sim::kNullEntity){
      auto target=base.WorldPose(ecm)->Pos()+Vec(0,0,.14);Vec offset(-1.25,-1.8,1.08);
      std::string view=stage<int(meta.size())?meta[stage].value("view",std::string("walk")):"finish";
      if(view=="stairs")offset.Set(2.,-2.8,1.8);
      if(view=="fire"&&active>=0){auto t=config["tasks"][active]["position"];target.Y()=(target.Y()+double(t[1])+.28)/2;target.Z()=double(t[2])+.6;offset.Set(-2.1,-1.3,1.15);}
      if(view=="drop")offset.Set(1.4,-1.55,.95);
      if(view=="finish")offset.Set(-1.8,-2.4,1.5);
      sim::Model(camera).SetWorldPoseCmd(ecm,Pose(target+offset,ignition::math::Quaterniond(0,std::atan2(offset.Z(),xy(offset)),std::atan2(-offset.Y(),-offset.X()))));
    }
  }
  void PostUpdate(const sim::UpdateInfo &info,const sim::EntityComponentManager &ecm)override{
    if(info.paused||finished||!base.Valid(ecm))return;
    bool touchGround=false,obstacle=false,dogWall=false;
    ecm.Each<sim::components::ContactSensorData>([&](const sim::Entity &,const sim::components::ContactSensorData *data){
      for(const auto &c:data->Data().contact()){
        if(c.position_size()==0)continue;const auto &a=c.collision1(),&b=c.collision2();
        auto pair=[&](sim::Entity x,const std::string &xn,sim::Entity y,const std::string &yn){return (contactModel(a,x,xn,ecm)&&contactModel(b,y,yn,ecm))||(contactModel(b,x,xn,ecm)&&contactModel(a,y,yn,ecm));};
        if(active>=0&&fired&&!hit){std::string target="demo_target_"+std::to_string(int(config["tasks"][active]["slot"]));
          if(pair(ball,ballName,named(target,ecm),target)){hit=true;event("ball_plate_contact",{{"model",ballName},{"target",target}});}}
        if(active>=0&&dropped&&!parcelLanded){
          std::string support=config["tasks"][active]["support_link"];
          if((contactModel(a,parcel,parcelName,ecm)&&contactLink(b,support,ecm))||(contactModel(b,parcel,parcelName,ecm)&&contactLink(a,support,ecm))){parcelLanded=true;event("parcel_floor_contact",{{"model",parcelName},{"surface",support},{"collision_1",a.name()},{"collision_2",b.name()}});}
        }
        bool airA=contactModel(a,uav.Entity(),"uav",ecm),airB=contactModel(b,uav.Entity(),"uav",ecm);
        if(airA||airB){
          const auto &other=airA?b:a;auto p=air.WorldPose(ecm);auto q=config["landing_point"];
          bool atPad=p&&std::hypot(p->Pos().X()-double(q[0]),p->Pos().Y()-double(q[1]))<.25
            &&std::abs(p->Pos().Z()-config.value("landing_z",0.))<.12;
          if(contactModel(other,named("arena_floor",ecm),"arena_floor",ecm))touchGround=touchGround||(requiresWindowExit()&&atPad);
          else if(landing&&atPad&&contactLink(other,config.value("landing_support_link",std::string("floor")),ecm))touchGround=true;
          else if(!contactModel(other,uav.Entity(),"uav",ecm))obstacle=true;
        }
        dogWall=dogWall||pair(dog.Entity(),"d1",named("building_envelope",ecm),"building_envelope");
      }return true;
    });
    if(obstacle&&now>1.){finish("uav_obstacle_contact",ecm);return;}
    if(dogWall&&now>1.){finish("d1_building_contact",ecm);return;}
    if(landing&&touchGround&&air.WorldLinearVelocity(ecm).value_or(Vec::Zero).Length()<.1)landingHold+=std::chrono::duration<double>(info.dt).count();else landingHold=0;
    if(!landedUav&&landingHold>.25){if(requiresWindowExit()&&!airExited){finish("landing_before_window_exit",ecm);return;}landedUav=true;event("uav_landed_rotors_stopped",{{"surface",config.value("landing_support_link",std::string("floor"))}});}
  }
};
IGNITION_ADD_PLUGIN(D1Mission,sim::System,D1Mission::ISystemConfigure,D1Mission::ISystemPreUpdate,D1Mission::ISystemPostUpdate)
