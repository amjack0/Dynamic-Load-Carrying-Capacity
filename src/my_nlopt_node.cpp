#include <iostream>
#include <nlopt.hpp>
#include <nlopt.h>
#include <vector>
#include <math.h>
#include <iomanip>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/velocityprofile.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/frames.hpp>
#include <error.h>
#include <rst-rt/geometry/Rotation.hpp>
#include <Eigen/Eigen>
#include <kdl/trajectory_segment.hpp>
#include <Eigen/Dense>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <ros/ros.h>
#include <kdl/jntarrayvel.hpp>
#include <kdl/framevel.hpp>
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <typeinfo>
#include <kdl/chainiksolver.hpp>
//#include "../matplotlibcpp.h"
#define N_JOINTS 6 

KDL::Frame _cartpos, goal_pose;
KDL::Vector grav(0,0, -9.80665);
KDL::Wrenches f_ext(N_JOINTS);
KDL::JntArray qdotdot_(N_JOINTS), tau_(N_JOINTS);


double blenRadius=0.1, _eqradius=0.1, Target_Velocity, Target_Acceleration, minf, _Energy, total_Energy = 0;
double delta_t = 0.001, jnt_number, motor_torque;
KDL::Chain mychain;
KDL::Tree mytree;
std::string root_name, tip_name;
std::string urdf_file_add = "/home/mujib/test_ws/src/universal_robot/ur_description/urdf/ur5_joint_test.urdf";
std::string joint_state_topic = "/urbot/joint_states";
KDL::JntArray jointPosCurrent(N_JOINTS), q_out(N_JOINTS), jointVelCurrent(N_JOINTS), q_in(N_JOINTS), jointVelCurrent_prev(N_JOINTS) ;
unsigned int map_joint_states[] = {2, 1, 0, 3, 4, 5};
std::vector<Eigen::VectorXd> desired_points;
geometry_msgs::Pose msg;
visualization_msgs::MarkerArray marker_arr;
unsigned int count = 0;
unsigned int total_count = 0;

geometry_msgs::PoseArray msg_traj;
ros::Publisher vis_pub, pose_pub;
std::vector<ros::Publisher> pose_multiple_pub;
KDL::Trajectory  *traj;
std::vector<std_msgs::Float64> msg_(N_JOINTS);
double qx, qy, qz, qw;
std::vector<double> a = {1, 1};

// provide your controller topic names
std::string tauTopicNames[] = {   
"/urbot/endeffector_frc_trq_controller_1/command",
"/urbot/endeffector_frc_trq_controller_2/command",
"/urbot/endeffector_frc_trq_controller_3/command",
"/urbot/endeffector_frc_trq_controller_4/command",
"/urbot/endeffector_frc_trq_controller_5/command",
"/urbot/endeffector_frc_trq_controller_6/command",
};

void my_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < N_JOINTS; i++)
    {
        jointPosCurrent(i) = msg->position[map_joint_states[i]];
        jointVelCurrent(i) = msg->velocity[map_joint_states[i]];
    }
//      std::cout << " [NLOPT] Joint Pos" << jointPosCurrent.data.transpose() << std::endl; 
}

typedef struct {
    double jnt_index, jnt_tau_max, jnt_mass;
} my_constraint_data;

typedef struct {
    double v_mass;
} my_vfunc_data;


double my_v_maxconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data )
{
    return x[0] - 2 ;
}

double my_a_maxconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data )
{
    return x[1] - 2 ;
}

/*double my_mass(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
  my_constraint_data *m = reinterpret_cast<my_constraint_data*>(data);
  double mass = m->jnt_mass ;
  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz, l= 0.05, r = l/2.0;

  Ixx = (1.0/12.0) * mass * ( 3*r*r + l*l);
  Iyy = Ixx;
  Izz =  (1.0/2.0) * mass * ( r*r );
  Ixy = Ixz= Iyz = 0;
  double I[6]={Ixx, Iyy, Izz, Ixy, Ixz, Iyz}; double offset[6] = {0, 0, 0, 0, 0, 0} ; std::string tool_name = "new_tool";
  KDL::Vector r_cog(0, 0, 0.05);
  KDL::Joint fixed_joint = KDL::Joint(KDL::Joint::None);
  KDL::Frame tip_frame = KDL::Frame(KDL::Rotation::RPY(offset[0],offset[1],offset[2]),
                          KDL::Vector(offset[3],offset[4],offset[5])) ;

  // rotational inertia in the cog
  KDL::RotationalInertia Inertia_cog = KDL::RotationalInertia(I[0], I[1], I[2], I[3], I[4], I[5]);
  KDL::RigidBodyInertia Inertia = KDL::RigidBodyInertia(mass, r_cog, Inertia_cog);
  KDL::Segment segment = KDL::Segment(tool_name, fixed_joint, tip_frame, Inertia);

  //parse kdl tree from Urdf
  kdl_parser::treeFromFile(urdf_file_add, mytree);

  //add segment to tree ( &mass, _Ixx, _cog 0,0,0 )
  //mytree.addSegment(  ,tip_name);
  //parse chain from tree

  if (!mytree.addSegment(segment, tip_name)) {
       std::cout << "[NLOPT] Could not add segment to kdl tree"<<std::endl;
       return -1;
   }

  mytree.getChain(root_name, tool_name, mychain);

  //chain id solver -> Inverse dynamics tau_
  KDL::ChainIdSolver_RNE my_solver(mychain,grav);
  my_solver.CartToJnt(q_out, jointVelCurrent, qdotdot_, f_ext, tau_);

  return 0;

}*/

/*constraints on tau (set torque constraint on each joint which comes from motor torque)*/

double my_tau_1(const std::vector<double> &x, std::vector<double> &grad, void *data )
{
    // BEGIN
  my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
  jnt_number = d->jnt_index ; motor_torque = d->jnt_tau_max; double mass = d->jnt_mass;
  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz, l= 0.05, r = l/2.0;
  std::cout << " In tau_constra : Mass -> " << mass << std::endl;

  Ixx = (1.0/12.0) * mass * ( 3*r*r + l*l);
  Iyy = Ixx;
  Izz =  (1.0/2.0) * mass * ( r*r );
  Ixy = Ixz= Iyz = 0;
  double I[6]={Ixx, Iyy, Izz, Ixy, Ixz, Iyz}; double offset[6] = {0, 0, 0, 0, 0, 0} ; std::string tool_name = "new_tool";
  KDL::Vector r_cog(0, 0, 0.05);
  KDL::Joint fixed_joint = KDL::Joint(KDL::Joint::None);
  KDL::Frame tip_frame = KDL::Frame(KDL::Rotation::RPY(offset[0],offset[1],offset[2]),
                          KDL::Vector(offset[3],offset[4],offset[5])) ;

  // rotational inertia in the cog
  KDL::RotationalInertia Inertia_cog = KDL::RotationalInertia(I[0], I[1], I[2], I[3], I[4], I[5]);
  KDL::RigidBodyInertia Inertia = KDL::RigidBodyInertia(mass, r_cog, Inertia_cog);
  KDL::Segment segment = KDL::Segment(tool_name, fixed_joint, tip_frame, Inertia);

  //parse kdl tree from Urdf
  if(!kdl_parser::treeFromFile(urdf_file_add, mytree)){
      std::cout << "[NLOPT] Could not parse kdl tree from Urdf" << std::endl;
      return -1;
  }

  if (!mytree.addSegment(segment, tip_name)) {
       std::cout << "[NLOPT] Could not add segment to kdl tree"<<std::endl;
       return -1;
   }

   mytree.getChain(root_name, tool_name, mychain);

    // END

    Target_Velocity = x[0];
    Target_Acceleration = x[1];
    double tau_1_max=0.0;
          std::unique_ptr<KDL::Path_RoundedComposite> path(
    new KDL::Path_RoundedComposite(blenRadius, _eqradius, new KDL::RotationalInterpolation_SingleAxis()));
    for (uint i = 0; i < desired_points.size(); i++)
    {
        auto Item = desired_points.at(i);
     try
    {
        path->Add ( KDL::Frame(KDL::Rotation::RPY(Item[3], Item[4], Item[5]), KDL::Vector(Item[0], Item[1], Item[2])) );
    }
    catch (const std::exception& e)
    {

        std::cout <<"[NLOPT] Failed to generate a trajectory:" << e.what() << std::endl;
    }
   }
    path->Finish();
    std::unique_ptr<KDL::VelocityProfile> velpref(new KDL::VelocityProfile_Trap(Target_Velocity, Target_Acceleration));
    velpref->SetProfile(0, path->PathLength());
    traj= new KDL::Trajectory_Segment(path.release(), velpref.release());
    KDL::ChainIkSolverVel_pinv iksolver_v (mychain);
    KDL::ChainFkSolverPos_recursive fk_solver(mychain);
    KDL::ChainIkSolverPos_NR iksolver_p (mychain, fk_solver, iksolver_v);
    KDL::ChainIdSolver_RNE my_solver(mychain,grav);

    for (double t_current = 0; t_current < traj->Duration(); t_current+=delta_t)
    {
        goal_pose = traj->Pos(t_current);
        iksolver_p.CartToJnt(q_in, goal_pose, q_out);
        /* reading q_dot from trajectory, calculate joint vel from cartesian vel */

        iksolver_v.CartToJnt(q_in, traj->Vel(t_current) ,jointVelCurrent);
        iksolver_v.CartToJnt(q_in, traj->Vel(t_current-delta_t) ,jointVelCurrent_prev);
        qdotdot_.data = (jointVelCurrent.data - jointVelCurrent_prev.data)/delta_t;
        /* Now, calculate joint acc from joint vel */
        my_solver.CartToJnt(q_out, jointVelCurrent, qdotdot_, f_ext, tau_);
        // taking the maximum torque out of all tau_(0)'s
        if (abs(tau_(jnt_number)) > tau_1_max){     //Norm
          tau_1_max = abs(tau_(jnt_number));
        }

        /*std::cout << " jointVelCurrent: " << jointVelCurrent.data << std::endl;
        std::cout << " jointVelCurrentPrev: " << jointVelCurrent_prev.data << std::endl;
        std::cout << " qdotdot_: " << qdotdot_.data << std::endl;
        std::cout << " tau_max: " << tau_1_max << std::endl;*/
    }
    std::cout << " tau_max: " << tau_1_max << std::endl;
    std::cout << " motor_torque: " << motor_torque << std::endl;
    return tau_1_max - motor_torque;
}

/* remove duration return _Energy, calculate trajectory inside constraint and add duration as costraint. */

double mytaufunc(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
  /* constraint on Duration || read jointvelcurrent and qdotdot_ from trajectory */
  Target_Velocity = x[0];
  Target_Acceleration = x[1];

          std::unique_ptr<KDL::Path_RoundedComposite> path(
  new KDL::Path_RoundedComposite(blenRadius, _eqradius, new KDL::RotationalInterpolation_SingleAxis()));

  for (uint i = 0; i < desired_points.size(); i++)
  {
  auto Item = desired_points.at(i);
  try
  {
     path->Add ( KDL::Frame(KDL::Rotation::RPY(Item[3], Item[4], Item[5]), KDL::Vector(Item[0], Item[1], Item[2])) );
  }
  catch (const std::exception& e)
   {
       std::cout <<"[NLOPT] Failed to generate a trajectory:" << e.what() << std::endl;
   }
  }
  path->Finish();
  std::unique_ptr<KDL::VelocityProfile> velpref(new KDL::VelocityProfile_Trap(Target_Velocity, Target_Acceleration));

  velpref->SetProfile(0, path->PathLength());
  traj= new KDL::Trajectory_Segment(path.release(), velpref.release());
  //std::cout << "[NLOPT] Cart trajectory duration: " << traj->Duration() << std::endl;
  return traj->Duration() - 0.2 ;
}


double myvfunc( const std::vector<double> &x, std::vector<double> &grad, void *data) 
{
  ++count;
  // BEGIN
  my_vfunc_data *v = reinterpret_cast<my_vfunc_data*>(data);
  double mass = v->v_mass;

  //std::cout << " In objective fun : Mass -> " << mass << std::endl;
  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz, l= 0.05, r = l/2.0;
  Ixx = (1.0/12.0) * mass * ( 3*r*r + l*l);
  Iyy = Ixx;
  Izz =  (1.0/2.0) * mass * ( r*r );
  Ixy = Ixz= Iyz = 0;
  double I[6]={Ixx, Iyy, Izz, Ixy, Ixz, Iyz}; double offset[6] = {0, 0, 0, 0, 0, 0} ; std::string tool_name = "new_tool";
  KDL::Vector r_cog(0, 0, 0.05);
  KDL::Joint fixed_joint = KDL::Joint(KDL::Joint::None);
  KDL::Frame tip_frame = KDL::Frame(KDL::Rotation::RPY(offset[0],offset[1],offset[2]),
                          KDL::Vector(offset[3],offset[4],offset[5]));

  // rotational inertia in the cog
  KDL::RotationalInertia Inertia_cog = KDL::RotationalInertia(I[0], I[1], I[2], I[3], I[4], I[5]);
  KDL::RigidBodyInertia Inertia = KDL::RigidBodyInertia(mass, r_cog, Inertia_cog);
  KDL::Segment segment = KDL::Segment(tool_name, fixed_joint, tip_frame, Inertia);

  //parse kdl tree from Urdf
  if(!kdl_parser::treeFromFile(urdf_file_add, mytree)){
      std::cout << "[NLOPT] Could not parse kdl tree from Urdf" << std::endl;
      return -1;
  }

  if (!mytree.addSegment(segment, tip_name)) {
       std::cout << "[NLOPT] Could not add segment to kdl tree"<<std::endl;
       return -1;
   }
   mytree.getChain(root_name, tool_name, mychain);
  // END

        total_Energy = _Energy = 0;
        Target_Velocity = x[0];
        Target_Acceleration = x[1];
        std::cout <<"[NLOPT] #### Target_Velocity " <<  Target_Velocity <<std::endl;
        std::cout <<"[NLOPT] #### Target_Acceleration " <<  Target_Acceleration <<std::endl;
        
                std::unique_ptr<KDL::Path_RoundedComposite> path(
        new KDL::Path_RoundedComposite(blenRadius, _eqradius, new KDL::RotationalInterpolation_SingleAxis())); 
    
        for (uint i = 0; i < desired_points.size(); i++)
        {
        auto Item = desired_points.at(i);
        try
        {
           path->Add ( KDL::Frame(KDL::Rotation::RPY(Item[3], Item[4], Item[5]), KDL::Vector(Item[0], Item[1], Item[2])) );
        }
        catch (const std::exception& e)
         {
             std::cout <<"[NLOPT] Failed to generate a trajectory:" << e.what() << std::endl;
         }
        }
        path->Finish();

        std::unique_ptr<KDL::VelocityProfile> velpref(new KDL::VelocityProfile_Trap(Target_Velocity, Target_Acceleration));
        
        velpref->SetProfile(0, path->PathLength());
        traj= new KDL::Trajectory_Segment(path.release(), velpref.release());
        std::cout << "[NLOPT] Cart trajectory duration: " << traj->Duration() << std::endl;
        
        KDL::ChainIkSolverVel_pinv iksolver_v (mychain);
        KDL::ChainFkSolverPos_recursive fk_solver(mychain);
        KDL::ChainIkSolverPos_NR iksolver_p (mychain, fk_solver, iksolver_v);
        KDL::ChainIdSolver_RNE my_solver(mychain,grav);
        f_ext.resize(mychain.getNrOfSegments());

        for (double t_current = 0; t_current < traj->Duration(); t_current+=delta_t)
        {
            goal_pose = traj->Pos(t_current);
            // TODO: read the previous joint angles as q_in
            iksolver_p.CartToJnt(q_in, goal_pose, q_out);
            iksolver_v.CartToJnt(q_in, traj->Vel(t_current) ,jointVelCurrent);
            iksolver_v.CartToJnt(q_in, traj->Vel(t_current-delta_t) ,jointVelCurrent_prev);
            qdotdot_.data = (jointVelCurrent.data - jointVelCurrent_prev.data)/delta_t;
            my_solver.CartToJnt(q_out, jointVelCurrent, qdotdot_, f_ext, tau_);
            //std::cout << "tau: " << tau_.data << "Mass: " << mass <<std::endl;
            _Energy = tau_.data.transpose()*tau_.data; // Norm

            // is Inverse dynamics torque changes with mass ?
            total_Energy = total_Energy + _Energy * delta_t ;
        }

        std::cout << "total energy: " << total_Energy << std::endl;
        return total_Energy;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "my_nlopt");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    ros::Subscriber my_states_sub = n.subscribe(joint_state_topic,20, my_callback);
    vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualize",20);

    root_name = "base_link";
    tip_name = "wrist_3_link";
    
    if(!kdl_parser::treeFromFile(urdf_file_add, mytree)) 
    {
    std::cout <<"[NLOPT] Failed to construct kdl tree for ur5 !" << std::endl;
    return -1;
    }
    
    if (!mytree.getChain(root_name, tip_name, mychain)) {
    std::cout << "[NLOPT] Could not initialize chain object\n"<<std::endl;
    return -1;
    }
    
    unsigned int nj, ns;
    nj =  mytree.getNrOfJoints();
    ns = mychain.getNrOfSegments(); 
    
    if (ns == 0 || nj == 0){
        std::cout <<"[NLOPT] Nr. of segments are: " <<  ns << std::endl;
        std::cout <<"[NLOPT] Nr. of Joints are: " <<  nj << std::endl;
        return -1;
    }
    
    f_ext.resize(ns);
    if(jointPosCurrent.rows()!=nj || jointVelCurrent.rows()!=nj || qdotdot_.rows()!=nj || tau_.rows()!=nj || f_ext.size()!=ns)
    {
        std::cout <<"[NLOPT] ERROR in size ! "  << std::endl;
        return -1;
    }
    
        for (int j = 0; j < N_JOINTS; j ++)
    {
    pose_pub = n.advertise<std_msgs::Float64>(tauTopicNames[j], 1);
    pose_multiple_pub.push_back(pose_pub);
    }
    
    q_in(0) = 0; q_in(1) = -1.5; q_in(2) = 2.0; q_in(3) = -0.5; q_in(4) = 0; q_in(5) = 0.5;
    Eigen::VectorXd v(6), b(6);
    v << 0.3754913603, 0.1090735799, 0.2286157252, -3.141572462, -0.4346034829, -3.140608804;
    desired_points.push_back(v);

    b << 0.4521725224, 0.1090586988, 0.6420975865, -3.11305549, 0.3821461179, -3.111322805;
    desired_points.push_back(b);
    
    std::cout << " [NLOPT] Hello World !" << std::endl;

    //for (double m = 0.0 ; m < 5.0 ; m+=0.5)
    double m = 0.0;
    while(EXIT_FAILURE){

    std::cout << std::endl;
    std::cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" << std::endl;
    std::cout << " mass : " << m << std::endl;
    nlopt::opt opt("LN_COBYLA", 2); /* algorithm and the number of optimization parameters */
    std::vector<double> lb(2);
    lb[0] = 0 ; lb[1] = 0 ; // lowerbound as 0
    opt.set_lower_bounds(lb);

    std::vector<double> ub(2);
    ub[0] = 3 ; ub[1] = 3 ;
    opt.set_upper_bounds(ub);
    //opt.set_initial_step(0.1);

    my_constraint_data data[9]={{0}, {0}, {0,0}, {0.0,0.1, m}, {1.0,1.0, m}, {2.0,1.0, m}, {3.0,1.0, m}, {4.0,0.0, m}, {5.0,0.0, m}};
    my_vfunc_data vdata[1] = { {m} };
    opt.set_min_objective(myvfunc, &vdata[0]);  //min_objective/max_objective
    
    //my_constraint_data data[9] = {{0}, {0}, {0,0}, {0.0,150,0}, {1.0,150,0}, {2.0,150,0}, {3.0,28,0}, {4.0,28,0}, {5.0,28,0}};

    opt.add_inequality_constraint(my_v_maxconstraint, &data[0], 1e-2);
    opt.add_inequality_constraint(my_a_maxconstraint, &data[1], 1e-2);
    opt.add_inequality_constraint(mytaufunc, &data[2], 1e-2);
    opt.add_inequality_constraint(my_tau_1, &data[3], 1e-2);
    opt.add_inequality_constraint(my_tau_1, &data[4], 1e-2);
    opt.add_inequality_constraint(my_tau_1, &data[5], 1e-2);
    opt.add_inequality_constraint(my_tau_1, &data[6], 1e-2);
    opt.add_inequality_constraint(my_tau_1, &data[7], 1e-2);
    opt.add_inequality_constraint(my_tau_1, &data[8], 1e-2);
    opt.set_xtol_rel(1e-2);
    opt.set_stopval(sqrt(8./27.)+1e-3);
    std::vector<double> x(2);

    //x[0] = 1.0; x[1] = 1.0; //starting with some initial guess:
    x[0] = a[0]; x[1] = a[1]; // iterations remains same

      try{
          std::cout << "RETURN VALUE: " << opt.optimize(x,minf) << std::endl;
          std::cout << "[NLOPT] Minimum found at: f(" << x[0] << "," << x[1] << ") = "
                  << std::setprecision(10) << minf <<std::endl;
          a[0] = x[0] ; a[1] = x[1] ;
          //std::cout << "[NLOPT] Nr. of iterations: " << count << std::endl;
          total_count += count;
          std::cout << "[NLOPT] Total Nr. of iter: " << total_count << std::endl;
          //return EXIT_SUCCESS;
      }

      catch(std::exception &e){
        std::cout << "[NLOPT] nlopt failed due to " << e.what() << std::endl;
        return EXIT_FAILURE;
      }
      m+=0.5;
}
    unsigned int counter = 0;

    while (ros::ok())
    {
        
        /*fk_solver.JntToCart(jointPosCurrent, _cartpos);
        std::cout <<"end effector cartesian pose: \n" << _cartpos << std::endl;
        double r,p,y;
        _cartpos.M.GetRPY(r,p,y);
        std::cout << "r :" << r << std::endl;
        std::cout << "p :" << p << std::endl;
        std::cout << "y :" << y << std::endl;*/
        
       /* int counter = 0;  BUG adds the same id counter */
       // TODO change the rate/iterations time to match the traj, Duration 0.00001
       
       for (double t_current = 0; t_current < traj->Duration(); t_current+=1.3e-4){
           
           //std::cout << "[NLOPT] Cart trajectory duration: " << traj->Duration() << std::endl;
           msg.position.x = traj->Pos(t_current).p.x();
           msg.position.y = traj->Pos(t_current).p.y();
           msg.position.z = traj->Pos(t_current).p.z();
           traj->Pos(t_current).M.GetQuaternion(qx, qy, qz, qw);
           msg.orientation.x = qx;
           msg.orientation.y = qy;
           msg.orientation.z = qz;
           msg.orientation.w = qw;
           msg_traj.poses.push_back(msg);

           goal_pose = traj->Pos(t_current);
           //std::cout <<"goal pose: \n" << goal_pose << std::endl;

           KDL::ChainIkSolverVel_pinv iksolver_v (mychain);
           KDL::ChainFkSolverPos_recursive fk_solver(mychain);
           KDL::ChainIkSolverPos_NR iksolver_p (mychain, fk_solver, iksolver_v);
           KDL::ChainIdSolver_RNE my_solver(mychain,grav);

           iksolver_p.CartToJnt(q_in, goal_pose, q_out);
           //std::cout <<"out joint positions: \n" << q_out.data << std::endl;

           for (int j = 0; j < N_JOINTS; j++)  // publish desired pose on joints in reihenfolge
            {
            msg_[j].data = q_out(j);
            pose_multiple_pub[j].publish(msg_[j]);
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.ns = "urbot";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            //marker.lifetime = 20;
            
            marker.id = counter;
            marker.pose.position.x = msg.position.x;
            marker.pose.position.y = msg.position.y;
            marker.pose.position.z = msg.position.z;
            marker.pose.orientation.x = msg.orientation.x;
            marker.pose.orientation.y = msg.orientation.y;
            marker.pose.orientation.z = msg.orientation.z;
            marker.pose.orientation.w = msg.orientation.w;
            
            marker_arr.markers.push_back(marker);
            counter++;            
       }
            /*vis_pub.publish(marker_arr);

        ros::spinOnce();
        loop_rate.sleep();*/

    }

    return 0;
}
