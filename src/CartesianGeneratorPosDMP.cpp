// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2006 Ruben Smits <ruben.smits@mech.kuleuven.ac.be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "CartesianGeneratorPosDMP.hpp"

using namespace RTT;
using namespace KDL;
using namespace std;
using namespace tf;

namespace MotionControl
{

  CartesianGeneratorPosDMP::CartesianGeneratorPosDMP(string name) :
	  TaskContext(name, PreOperational), m_motion_profile(6,
			  VelocityProfile_Trap(0, 0)), 
	  move_started_event("e_"+name+"_move_started"), 
	  move_finished_event("e_"+name+"_move_finished"),
	  m_is_moving(false)
  {
	  //Creating TaskContext

	  //Adding Ports
	  this->addPort("jacobian_svd_outport", jacobian_svd_outport);
	  this->addEventPort("jacobian_svd_inport", jacobian_svd_inport);
	  this->addPort("CartesianPoseMsr", m_position_meas_port);
	  this->addPort("CartesianPoseDes", m_position_desi_port);
	  this->addPort("CartesianTwistDes", m_velocity_desi_port);
	  this->addPort("events", event_port);
	  this->addPort("lastCommandOutport", last_command_outport);
	  this->addEventPort("jacobian_inport", jacobian_inport);
	  this->ports()->addPort( "desiredStiffnessOutport",desired_stiffness_outport ).doc("desired stiffness output port");
	  //Adding Properties
	  this->addProperty("max_vel", m_gm_maximum_velocity).doc(
			  "Maximum Velocity in Trajectory");
	  this->addProperty("max_acc", m_gm_maximum_acceleration).doc(
			  "Maximum Acceleration in Trajectory");

	  //Adding Commands
	  this->addOperation("movePeriodic", &CartesianGeneratorPosDMP::movePeriodic, this,
			  OwnThread) .doc("Start periodic motion");
	  this->addOperation("moveToDMP", &CartesianGeneratorPosDMP::moveToDMP, this,
			  OwnThread) .doc("Set the position setpoint") .arg("setpoint",
			  "position setpoint for end effector") .arg("time",
			  "minimum time to execute trajectory");
	  this->addOperation("setWeight",&CartesianGeneratorPosDMP::setWeight,this,OwnThread)
			    .doc("Set the DMP weights");
	  this->addOperation("resetWeight",&CartesianGeneratorPosDMP::resetWeight,this,OwnThread)
			    .doc("Reset the DMP weights to zero");
	  //Adding Methods
	  this->addOperation("resetPosition", &CartesianGeneratorPosDMP::resetPosition,this, OwnThread).doc("Reset generator's position");
	  this->addOperation("resetPositionActual", &CartesianGeneratorPosDMP::resetPositionActual,this, OwnThread).doc("Reset generator's position");
	  this->addOperation("setCartesianStiffness", &CartesianGeneratorPosDMP::setCartesianStiffness,this, OwnThread).doc("Set the stiffness to desired values");
	  this->addOperation("startKinesthetic", &CartesianGeneratorPosDMP::start_kinesthetic,this, OwnThread).doc("Start kinesthetic guidance. mirror the actual pose in desired pose");
	  this->addOperation("setCartesianStiffnessCalibration", &CartesianGeneratorPosDMP::setCartesianStiffnessCalibration,this, OwnThread).doc("Set the stiffness for calibration");
	  this->addOperation("startKinestheticCalibration", &CartesianGeneratorPosDMP::start_kinesthetic_calibration,this, OwnThread).doc("Start kinesthetic for calibration, only in XY plane");
  }

  CartesianGeneratorPosDMP::~CartesianGeneratorPosDMP() { }

  bool CartesianGeneratorPosDMP::configureHook()
  {
	  //log(Warning)<<"CartesianGeneratorPos::configureHook() DFGDGDFGDFGDFGDF "<<endlog();
	  TwistMsgToKDL(m_gm_maximum_velocity, m_maximum_velocity);
	  TwistMsgToKDL(m_gm_maximum_acceleration, m_maximum_acceleration);
  // 	log(Info)<<"CartesianGeneratorPos::configureHook()  m_gm_maximum_velocity "<<m_gm_maximum_velocity<<endlog();
  // 	log(Info)<<"CartesianGeneratorPos::configureHook()  m_maximum_velocity "<<m_maximum_velocity.vel(0)<<endlog();
  // 	log(Info)<<"CartesianGeneratorPos::configureHook()  m_gm_maximum_acceleration "<<m_gm_maximum_acceleration<<endlog();
  // 	log(Info)<<"CartesianGeneratorPos::configureHook()  m_maximum_acceleration "<<m_maximum_acceleration.vel(0)<<endlog();

	  for (unsigned int i = 0; i < 3; i++) {
		  m_motion_profile[i].SetMax(m_maximum_velocity.vel[i],
				  m_maximum_acceleration.vel[i]);
		  m_motion_profile[i + 3].SetMax(m_maximum_velocity.rot[i],
				  m_maximum_acceleration.rot[i]);
	  }
	  /// DMP parameters
	  dmp = new DMP[4];
	  
	  err_pos_ok=0.0005; /// acceptable error in position 
	  err_angle_ok= 0.1*PI/180;//0.5; /// acceptable error in angle teta 
	  step_time=0.01;//0.005; ///step time 5ms
	  //kernel_number=50; 
	  //kernel_width=0.01;
	  //time_scale = 1;
	  pos_ok = angle_ok = false;
	  x_dmp = y_dmp = z_dmp = theta_dmp =theta_eq =0;
	  axis_eq.x(0);  axis_eq.y(0);  axis_eq.z(0);
	  /// init DMP Weights to zero
	  //std::vector<float> zero_weights(kernel_number,0);
	  //std::vector<float> cosine_weights(kernel_number,0);
	  //std::vector<float> sine_weights(kernel_number,0);
  // 	for (int i=0;i<kernel_number;i++)
  // 	{
  // 	  cosine_weights.at(i) = 0.024*0.6*cos(2*M_PI*i/(kernel_number-1))-1.0; /// cosine weights 
  // 	  sine_weights.at(i) = 0.024*0.6*sin(2*M_PI*i/(kernel_number-1));     /// sine  weights 
  // 	}
	  //Weight_x = zero_weights;
	  //Weight_y = zero_weights;
	  //Weight_z = zero_weights;
	  amp_x = amp_y = amp_z = 0;
	  omega_x = omega_y = omega_z = 0;
	  periodic_active = false;
	  kinestetic_active = false;
	  calibration_active = false;
	  return true;

  }

  bool CartesianGeneratorPosDMP::startHook()
  {
  // 	log(Info)<<"CartesianGeneratorPos::startHook()"<<endlog();
	  m_is_moving = false;
	  //initialize
	  geometry_msgs::Pose gm_starting_pose; Frame starting_pose;
	  geometry_msgs::Twist gm_starting_twist; Twist starting_twist;

	  if(m_position_meas_port.read(gm_starting_pose)==NoData){
		  log(Error) << this->getName() << " cannot start if " <<
			  m_position_meas_port.getName()<<" has no input data."<<endlog();
		  return false;
	  }
  //        log(Info) << "CartesianGeneratorPos::startHook()  Starting gm_pose is= "<<gm_starting_pose.position<<endlog();
	  last_command = gm_starting_pose;
	  last_command_outport.write(last_command);
	  m_position_desi_port.write(gm_starting_pose);
	  PoseMsgToKDL(gm_starting_pose, starting_pose);
  // 	log(Info) << "CartesianGeneratorPos::startHook()  Starting pose is= "
  // 	<<starting_pose.p(0)<<" "<<starting_pose.p(1)<<" "<<starting_pose.p(2)<<endlog();
	  starting_twist = Twist::Zero();
	  TwistKDLToMsg(starting_twist, gm_starting_twist);
	  m_velocity_desi_port.write(gm_starting_twist);
	  return true;
  }

  void CartesianGeneratorPosDMP::updateHook()
  {
  // 	log(Info)<<"CartesianGeneratorPos::updateHook() "<<m_time_passed<<"/"<<m_max_duration<<endlog();
	  geometry_msgs::Pose gm_pos_dsr;
	  geometry_msgs::Twist gm_vel_dsr;
	  /// note: Jacobian is always in stiffness.TOOL frame of FRI. (like the force)
	  if(jacobian_inport.read(current_jacobian)==NewData)
	  {
	    int jacobian_rows = current_jacobian.rows();
	    int jacobian_cols = current_jacobian.columns();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> received new Jacobian"<<current_jacobian.data<<" \nrows="<<
// 	      jacobian_rows<<"  cols= "<<jacobian_cols<<endlog();
	    
	    MatrixXd U(jacobian_rows,jacobian_cols);
	    MatrixXd V(jacobian_cols,jacobian_cols);
	    int number_diagonal = jacobian_cols;
	    if(jacobian_rows>jacobian_cols)
	      number_diagonal = jacobian_rows;
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> number_diagonal= "<<number_diagonal<<endlog();
	    VectorXd S(number_diagonal);
	    
// 	    for(int i=0; i<number_diagonal;i++)
// 	    {
// 	      log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> S at i="<<i<<" is="<<S(i)<<endlog();
// 	    }
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> 2"<<endlog();
	    VectorXd temp = S;
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> 3"<<endlog();
	    MatrixXd A= current_jacobian.data;
// 	    U = A;
// 	    V = A;
	    
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> A="<<A<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> U="<<U<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> V="<<V<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> temp="<<temp<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> S="<<S<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> calculate SVD"<<endlog();
	    int retval= KDL::svd_eigen_HH(A,U,S,V,temp);
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> S="<<S<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> calculate SVD DONE!  retval="<<retval<<endlog();
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> S="<<S<<endlog();
	    jacobian_svd.clear();
	    for(int i=0; i<number_diagonal;i++)
	    {
	      jacobian_svd.push_back(S(i));
	    }
	    jacobian_svd_outport.write(jacobian_svd);
// 	    log(Warning)<<"CartesianGeneratorPosDMP::updateHook >> singular values="<<S(0)<<endlog();
	  }

	  if (m_is_moving)
	  {
	    
		  m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);
  // 		log(Warning)<<"t= "<<m_time_passed<<endlog();
		  /// Get the current DMP values
		  x_dmp=dmp[0].get_y();
		  y_dmp=dmp[1].get_y();
		  z_dmp=dmp[2].get_y();
		  theta_dmp = dmp[3].get_y(); /// in Radians
// 		  log(Warning)<<"t= "<<m_time_passed<<" x_dmp= "<<x_dmp<<" y_dmp= "<<y_dmp<<" z_dmp= "<<z_dmp<<" "<<theta_dmp<<endlog();
		  /// update current pose
		  if(m_position_meas_port.read(current_pose)==NoData)
		  {
		    log(Error) << this->getName() << " cannot move if " <<m_position_meas_port.getName()<<" has no input data."<<endlog();
		    m_is_moving = false;
		  }
		  else
		  {
		   //cout<<current_pose.position.x<<" "<<current_pose.position.y<<" "<<current_pose.position.z<<" "<<x_dmp<<" "<<y_dmp<<" "<<z_dmp<<endl; 
		  }
		  /// calculate errors
		  err_pos=sqrt((m_traject_end.p.x()-x_dmp)*(m_traject_end.p.x()-x_dmp) +
		  (m_traject_end.p.y()-y_dmp)*(m_traject_end.p.y()-y_dmp) +
		  (m_traject_end.p.z()-z_dmp)*(m_traject_end.p.z()-z_dmp)	);
		  err_angle=sqrt((theta_dmp-theta_eq)*(theta_dmp-theta_eq));
	
		  /// set error flags
		  if (err_angle<err_angle_ok&& m_time_passed>5*(time_scale*m_duration-1)/6)  {angle_ok=true;}
		  if (err_pos<err_pos_ok && m_time_passed>5*time_scale*m_duration/6) {pos_ok=true;}
		  /// convert from table to robot coordinates (30 degree)
		  float x_robot = x_dmp;//cos(M_PI/6.0)*x_dmp - sin(M_PI/6.0)*y_dmp;
		  float y_robot = y_dmp;//sin(M_PI/6.0)*x_dmp + cos(M_PI/6.0)*y_dmp;
		  float z_robot = z_dmp;
		  
		  /// check to avoid NAN
		  if (isnan(x_robot)||isnan(y_robot)||isnan(z_dmp))
		  {
		    log(Error)<<"At least one of the positions is NAN !!!!! @t="<<m_time_passed<<" "<<x_robot<<" "<<y_robot<<" "<<z_dmp;
		    m_is_moving = false;
		    this->stop();
		    
		  }
		  else /// update position values
		  {
		    m_position_desi_local.p.x(x_robot);
		    m_position_desi_local.p.y(y_robot);
		    m_position_desi_local.p.z(z_robot);
		  }
		  double quat_x, quat_y, quat_z, quat_w;
		  double angle_a,angle_b,angle_c;
		  m_position_desi_local.M.GetQuaternion(quat_x,quat_y,quat_z,quat_w);
		  m_position_desi_local.M.GetEulerZYX(angle_a,angle_b,angle_c);
  // 		log(Warning) << "CartesianGeneratorPos::updateHook()  desired pose is= "
  // 		<<m_position_desi_local.p.x()<<" "<<m_position_desi_local.p.y()<<" "<<m_position_desi_local.p.z()
  // 		<<" quaternion= "<<quat_x<<" "<<quat_y<<" "<<quat_z<<" "<<quat_w<<" Euler Angles= "<<angle_a<<" "<<angle_b<<" "<<angle_c<<endlog();
		  /// calculate the new commanded orientation
		  Rotation Rotation_dmp = Rotation::Rot(axis_eq,theta_dmp);
		  Rotation Rotation_next = Rotation_dmp*m_traject_begin.M;
		  m_position_desi_local.M = Rotation_next;
// 		  double a,b,c;
// 		  Rotation_next.GetEulerZYX(a,b,c);
// 		  a = a*180/PI;
// 		  b = b*180/PI;
// 		  c = c*180/PI;
		  
		  //std::cout<<a<<" "<<b<<" "<<c<<std::endl;

		  // convert to geometry msgs and send.
		  PoseKDLToMsg(m_position_desi_local, gm_pos_dsr);
  // 		log(Info) << "CartesianGeneratorPos::updateHook()  desired gm_pose is= "<<gm_pos_dsr.position<<endlog();
		  //TwistKDLToMsg(m_velocity_desi_local, gm_vel_dsr);
		  last_command = gm_pos_dsr;
		  last_command_outport.write(last_command);
		  m_position_desi_port.write(gm_pos_dsr);
		  /// if not in the goal pose and time is not up, calculate_one_step_dmp
		  if(pos_ok && angle_ok)
		  {
		    m_is_moving=false;
		    //log(Warning)<<"Error position is= "<<err_pos<<" < "<<err_pos_ok<<" And Error Angle is= "<<err_angle<<" < "<<err_angle_ok<<endlog();
		    event_port.write(move_finished_event);
		  }
		  if (!pos_ok)
		    if(m_time_passed<1.1*time_scale*m_duration)
		    {
		      dmp[0].calculate_one_step_dmp(m_time_passed);
		      dmp[1].calculate_one_step_dmp(m_time_passed);
		      dmp[2].calculate_one_step_dmp(m_time_passed);
		    }
		  if (!angle_ok)
		    if(m_time_passed<1.1*time_scale*m_duration)
		      dmp[3].calculate_one_step_dmp(m_time_passed);
		  //m_velocity_desi_port.write(gm_vel_dsr);
	  }
	  else if(periodic_active)
	  {
	    /// get the current position
	    Frame current_pose;
	    geometry_msgs::Pose gm_pose_msr;
	    m_position_meas_port.read(gm_pose_msr);
	    PoseMsgToKDL(gm_pose_msr, current_pose);
	    ///
	    m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);
	    Frame goal_frame = m_traject_begin;
	    geometry_msgs::Pose pose;
	    m_position_meas_port.read(pose);
	    goal_frame.p.data[0] += amp_x *sin(omega_x*m_time_passed);
	    goal_frame.p.data[1] += amp_y *sin(omega_y*m_time_passed);
  // 	  goal_frame.p.data[2] = pose.position.z;
	    goal_frame.p.data[2] += amp_z *sin(omega_z*m_time_passed);
	    m_position_desi_local = goal_frame;
			  // position
  // 			m_velocity_delta = Twist( Vector( m_motion_profile[0].Pos(m_time_passed),
  // 							  m_motion_profile[1].Pos(m_time_passed),
  // 							  m_motion_profile[2].Pos(m_time_passed)),
  // 						  Vector( m_motion_profile[3].Pos(m_time_passed),
  // 							  m_motion_profile[4].Pos(m_time_passed),
  // 							  m_motion_profile[5].Pos(m_time_passed)));
  // 			log(Info)<<"CartesianGeneratorPos::updateHook()  m_velocity_delta.rot= "
  // 			<<m_velocity_delta.rot(0)<<" "<<m_velocity_delta.rot(1)<<" "<<m_velocity_delta.rot(2)<<" "<<endlog();
  // 			log(Info)<<"CartesianGeneratorPos::updateHook()  m_velocity_delta.vel= "
  // 			<<m_velocity_delta.vel(0)<<" "<<m_velocity_delta.vel(1)<<" "<<m_velocity_delta.vel(2)<<" "<<endlog();

  // 			m_position_desi_local = Frame( m_traject_begin.M *
  // 						       Rot( m_traject_begin.M.Inverse(m_velocity_delta.rot)),
  // 						       m_traject_begin.p + m_velocity_delta.vel);

			  // velocity
  // 			for (unsigned int i = 0; i < 6; i++)
  // 				m_velocity_desi_local(i) = m_motion_profile[i].Vel(m_time_passed);

		  
  // 		log(Info) << "CartesianGeneratorPos::updateHook()  desired pose is= "
  // 		<<m_position_desi_local.p(0)<<" "<<m_position_desi_local.p(1)<<" "<<m_position_desi_local.p(2)<<endlog();

		  // convert to geometry msgs and send.
		  PoseKDLToMsg(m_position_desi_local, gm_pos_dsr);
  // 		log(Info) << "CartesianGeneratorPos::updateHook()  desired gm_pose is= "<<gm_pos_dsr.position<<endlog();
  // 		TwistKDLToMsg(m_velocity_desi_local, gm_vel_dsr);
		  last_command = gm_pos_dsr;
		  last_command_outport.write(last_command);
		  m_position_desi_port.write(gm_pos_dsr);
  // 		m_velocity_desi_port.write(gm_vel_dsr);
	    ///
	    
	  }
	  else if(kinestetic_active)
	  {
	    geometry_msgs::Pose pose;
	    m_position_meas_port.read(pose);
	    m_position_desi_port.write(pose);
	    
	  }
	  else if(calibration_active)
	  {
	    /// measure current pose_robot
	    geometry_msgs::Pose pose_robot;
	    m_position_meas_port.read(pose_robot);
	    /// transform pose to frame
	    KDL::Frame frame_robot;
	    tf::PoseMsgToKDL(pose_robot,frame_robot);
	    /// transform frame_robot to frame_table
	    KDL::Frame frame_table = table_robot_frame*frame_robot;
	    /// overwrite the measured frame_table with orientation and Z of calib_frame_table
	    frame_table.M = calib_frame_table.M;
	    //frame_table.p.data[2] = calib_frame_table.p.data[2];
	    /// convert new frame_table to frame_robot
	    frame_robot = table_robot_frame.Inverse() * frame_table;
	    /// convert frame_robot to pose_robot
	    tf::poseKDLToMsg(frame_robot,pose_robot);
	    /// send the new pose_robot as a command to robot    
	    m_position_desi_port.write(pose_robot);
	    
	  }
  }

  void CartesianGeneratorPosDMP::stopHook() { }

  void CartesianGeneratorPosDMP::cleanupHook() { }
  bool CartesianGeneratorPosDMP::setWeight(std::vector< float > Wx, std::vector< float > Wy, std::vector< float > Wz)
  {
    Weight_x = Wx;
    Weight_y = Wy;
    Weight_z = Wz;
  }
  bool CartesianGeneratorPosDMP::resetWeight(int kernel_number)
  {
    if(kernel_number>0)
    {
      std::vector<float> zero_weights(kernel_number,0);
      Weight_x = zero_weights;
      Weight_y = zero_weights;
      Weight_z = zero_weights;
      return true;
    }
    else
    {
      log(Error)<<"CartesianGeneratorPosDMP::resetWeight >> kernel_number must be a positive integer, it is= "<<kernel_number<<endlog();
      return false;
    }

    
  
  }
  /// implements a moveTo function using DMP
  bool CartesianGeneratorPosDMP::moveToDMP(geometry_msgs::Pose gm_pose, double time, double t_scale, double k_number, double k_width)
  {
	  log(Warning)<<"CartesianGeneratorPosDMP::moveToDMP() >> started ..."<<endlog();
	  time_scale = t_scale;
	  kernel_number = k_number;
	  kernel_width = k_width;
	  Frame pose;
	  geometry_msgs::Pose gm_pose_msr;
	  PoseMsgToKDL(gm_pose, pose);

	  if(!this->isRunning()){
		  log(Error)<<this->getName()<<" is not running yet."<<endlog();
		  return false;
	  }
	  log(Warning)<<"Moving to pose  x= "<<gm_pose.position.x<<" y="<<gm_pose.position.y<<" z="<<gm_pose.position.z<<endlog();
	  m_max_duration = 0;
	  m_duration = time;

	  m_traject_end=pose;

	  // get current position
	  /// old way : set desired as actual
	  m_position_meas_port.read(gm_pose_msr);
	  //PoseMsgToKDL(gm_pose_msr, m_traject_begin);
	  /// new way : keep the last desired
	  PoseMsgToKDL(last_command, m_traject_begin);
	  x_dmp = m_traject_begin.p.x();
	  y_dmp = m_traject_begin.p.y();
	  z_dmp = m_traject_begin.p.z();
	  theta_dmp =0;
	  //log(Warning)<<"Initializing the dmp outputs to: "<<x_dmp<<" "<<y_dmp<<" "<<z_dmp<<" "<<theta_dmp<<endlog();
	  dmp[0].init_dmp(m_traject_begin.p.x(),m_traject_end.p.x(), time, step_time, time_scale, kernel_number, kernel_width);
	  dmp[1].init_dmp(m_traject_begin.p.y(),m_traject_end.p.y(), time, step_time, time_scale, kernel_number, kernel_width);
	  dmp[2].init_dmp(m_traject_begin.p.z(),m_traject_end.p.z(), time, step_time, time_scale, kernel_number, kernel_width);
	  //log(Warning)<<"CartesianGeneratorPosDMP::moveToDMP() >> before applying the weights"<<endlog();
	  /// apply weights to DMPs
	  for (int i=0; i<kernel_number; i++)
	  {
	    dmp[0].set_w(i, Weight_x.at(i));
	    dmp[1].set_w(i, Weight_y.at(i));
	    dmp[2].set_w(i, Weight_z.at(i));
	  }
	  //log(Warning)<<"CartesianGeneratorPosDMP::moveToDMP() >> before resetting the weights"<<endlog();
	  /// reset weights to zero, for next time! (maybe not the best solution, esp if kernel_number is not constant)
	  std::vector<float> zero_weights(kernel_number,0);
	  Weight_x = zero_weights;
	  Weight_y = zero_weights;
	  Weight_z = zero_weights;
  // 	log(Info)<<"CartesianGeneratorPos::moveTo()  m_traject_begin= "
  // 	<<m_traject_begin.p(0)<<" "<<m_traject_begin.p(1)<<" "<<m_traject_begin.p(2)<<endlog();
	  //log(Warning)<<"CartesianGeneratorPosDMP::moveToDMP() >> before orientation"<<endlog();
	  /// Orientation
	  KDL::Rotation Rotation_eq;
	  Rotation_eq = m_traject_end.M*m_traject_begin.M.Inverse();
	  
	  theta_eq= Rotation_eq.GetRotAngle(axis_eq);
	  dmp[3].init_dmp(0,theta_eq, time, step_time, time_scale, kernel_number, kernel_width);

	  

	  // Rescale trajectories to maximal duration
	  

	  m_time_begin = os::TimeService::Instance()->getTicks();
	  m_time_passed = 0;
	  //log(Warning)<<"CartesianGeneratorPosDMP::moveToDMP() >> before moving"<<endlog();

	  m_is_moving = true;
	  // send move_started_event )
	  event_port.write(move_started_event);
	  m_position_desi_local = m_traject_begin;
	  pos_ok = angle_ok = false;

	  return true;
  }
  bool CartesianGeneratorPosDMP::movePeriodic(KDL::Vector & amplitude_vector, float wx, float wy, float wz)
  {
    
    amp_x = amplitude_vector.data[0];
    amp_y = amplitude_vector.data[1];
    amp_z = amplitude_vector.data[2];
    omega_x = wx;
    omega_y = wy;
    omega_z = wz;
    periodic_active = true;
    m_is_moving = false;
    ///
    log(Warning)<<"CartesianGeneratorPos::movePeriodic >> started ..."<<endlog();
	  Frame pose;
	  
  // 	PoseMsgToKDL(gm_pose, pose);

	  if(!this->isRunning()){
		  log(Error)<<this->getName()<<" is not running yet."<<endlog();
		  return false;
	  }
  // 	log(Warning)<<"Moving to pose  x= "<<gm_pose.position.x<<" y="<<gm_pose.position.y<<" z="<<gm_pose.position.z<<endlog();
	  m_max_duration = 0;

  // 	m_traject_end=pose;

	  // get current position
	  geometry_msgs::Pose gm_pose_msr;
	  m_position_meas_port.read(gm_pose_msr);
	  PoseMsgToKDL(gm_pose_msr, m_traject_begin);
  // 	log(Info)<<"CartesianGeneratorPos::moveTo()  m_traject_begin= "
  // 	<<m_traject_begin.p(0)<<" "<<m_traject_begin.p(1)<<" "<<m_traject_begin.p(2)<<endlog();
  // 	m_velocity_begin_end = diff(m_traject_begin, m_traject_end);

	  // Set motion profiles
  // 	for (unsigned int i = 0; i < 6; i++) {
  // 		m_motion_profile[i].SetProfileDuration(0, m_velocity_begin_end(i), time);
  // 		m_max_duration = max(m_max_duration, m_motion_profile[i].Duration());
  // // 		log(Info)<<"CartesianGeneratorPos::moveTo()  m_max_duration"<<m_max_duration<<endlog();
  // 	}

	  // Rescale trajectories to maximal duration
  // 	for (unsigned int i = 0; i < 6; i++)
  // 		m_motion_profile[i].SetProfileDuration(0, m_velocity_begin_end(i),
  // 				m_max_duration);
  // 
	  m_time_begin = os::TimeService::Instance()->getTicks();
	  m_time_passed = 0;

	  periodic_active = true;
	  // send move_started_event )
  // 	event_port.write(move_started_event);
	  log(Warning)<<"Periodic Move command is sent"<<endlog();

	  return true;
    ///
  }
  void CartesianGeneratorPosDMP::resetPosition()
  {
	  geometry_msgs::Pose pose;
	  geometry_msgs::Twist twist;

	  //m_position_meas_port.read(pose);
	  m_position_desi_port.write(last_command);

	  SetToZero(m_velocity_desi_local);
	  TwistKDLToMsg(m_velocity_desi_local, twist);
	  m_velocity_desi_port.write(twist);

	  m_is_moving = false;
	  periodic_active = false;
	  kinestetic_active = false;
	  calibration_active = false;
  }
  void CartesianGeneratorPosDMP::resetPositionActual()
  {
	  geometry_msgs::Pose pose;
	  geometry_msgs::Twist twist;

	  m_position_meas_port.read(pose);
	  m_position_desi_port.write(pose);
	  last_command = pose;

	  SetToZero(m_velocity_desi_local);
	  TwistKDLToMsg(m_velocity_desi_local, twist);
	  m_velocity_desi_port.write(twist);

	  m_is_moving = false;
	  periodic_active = false;
	  kinestetic_active = false;
	  calibration_active = false;
  }
  void CartesianGeneratorPosDMP::setCartesianStiffness(float linear_stiffness, float angular_stiffness,float linear_damping, float angular_damping)
  {
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 1"<<endlog();
	  lwr_fri::CartesianImpedance desired_stiffness;
	  desired_stiffness.stiffness.linear.x = linear_stiffness;
	  desired_stiffness.stiffness.linear.y = linear_stiffness;
	  desired_stiffness.stiffness.linear.z = linear_stiffness;
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 2"<<endlog();
	  desired_stiffness.stiffness.angular.x = angular_stiffness;
	  desired_stiffness.stiffness.angular.y = angular_stiffness;
	  desired_stiffness.stiffness.angular.z = angular_stiffness;
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 3"<<endlog();
	  desired_stiffness.damping.linear.x = linear_damping;
	  desired_stiffness.damping.linear.y = linear_damping;
	  desired_stiffness.damping.linear.z = linear_damping;
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 4"<<endlog();
	  desired_stiffness.damping.angular.x = angular_damping;
	  desired_stiffness.damping.angular.y = angular_damping;
	  desired_stiffness.damping.angular.z = angular_damping;
	  
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> setting the stiffness to "<<desired_stiffness<<endlog();
	  desired_stiffness_outport.write(desired_stiffness);
  }
  void CartesianGeneratorPosDMP::setCartesianStiffnessCalibration()
  {
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 1"<<endlog();
	  lwr_fri::CartesianImpedance desired_stiffness;
	  desired_stiffness.stiffness.linear.x = 2;
	  desired_stiffness.stiffness.linear.y = 2;
	  desired_stiffness.stiffness.linear.z = 3000;
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 2"<<endlog();
	  desired_stiffness.stiffness.angular.x = 300;
	  desired_stiffness.stiffness.angular.y = 300;
	  desired_stiffness.stiffness.angular.z = 300;
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 3"<<endlog();
	  desired_stiffness.damping.linear.x = 0.7;
	  desired_stiffness.damping.linear.y = 0.7;
	  desired_stiffness.damping.linear.z = 0.7;
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> 4"<<endlog();
	  desired_stiffness.damping.angular.x = 0.7;
	  desired_stiffness.damping.angular.y = 0.7;
	  desired_stiffness.damping.angular.z = 0.7;
	  
	  log(Warning)<<"CartesianGeneratorPosDMP::setCartesianStiffness >> setting the stiffness to "<<desired_stiffness<<endlog();
	  desired_stiffness_outport.write(desired_stiffness);
  }
  bool CartesianGeneratorPosDMP::start_kinesthetic()
  {
    log(Warning)<<"CartesianGeneratorPos::start_kinesthetic >>  kinestetic guidance is activated "<<endlog();

	  if(!this->isRunning()){
		  log(Error)<<this->getName()<<" is not running yet."<<endlog();
		  return false;
	  }
	  periodic_active = false;
	  kinestetic_active = true;
	  m_is_moving = false;
	  calibration_active = false;

	  return true;
  }
  bool CartesianGeneratorPosDMP::start_kinesthetic_calibration(KDL::Frame table_robot_transform)
  {
    log(Warning)<<"CartesianGeneratorPos::start_kinesthetic_calibration >>  kinestetic guidance is activated for calibration "<<endlog();

	  if(!this->isRunning())
	  {
		  log(Error)<<this->getName()<<" is not running yet."<<endlog();
		  return false;
	  }
	  periodic_active = false;
	  kinestetic_active = false;
	  m_is_moving = false;
	  calibration_active = true;
	  table_robot_frame = table_robot_transform;
	  geometry_msgs::Pose calib_pose;
	  m_position_meas_port.read(calib_pose);
	  KDL::Frame calib_frame_robot;
	  tf::poseMsgToKDL(calib_pose,calib_frame_robot);
	  calib_frame_table = table_robot_frame * calib_frame_robot;

	  return true;
  }
} //namespace
