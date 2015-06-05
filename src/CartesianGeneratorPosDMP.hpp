// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2010 Ruben Smits <ruben.smits@mech.kuleuven.be>
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

#ifndef __CARTESIAN_GENERATOR_POS_DMP_H__
#define __CARTESIAN_GENERATOR_POS_DMP_H__
#define BOOST_FUSION_INVOKE_MAX_ARITY 10
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>

#include <geometry_msgs/typekit/Types.h>
#include <tf_conversions/tf_kdl.h>

#include <kdl/velocityprofile_trap.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>

#include <lwr_fri/typekit/Types.hpp>

#include <string>
#include "dmp.h"
namespace MotionControl
{
    /**
     * This class implements a TaskContext that creates a path in
     * Cartesian space between the current cartesian position and a
     * new desired cartesian position. It uses trapezoidal
     * velocity-profiles for every dof using a maximum velocity and a
     * maximum acceleration. It generates frame and twist setpoints
     * which can be used by MotionControl::CartesianControllerPos,
     * MotionControl::CartesianControllerPosVel or MotionControl::CartesianControllerVel.
     *
     */
    class CartesianGeneratorPosDMP : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class.
         *
         * @param name name of the TaskContext
         */
        CartesianGeneratorPosDMP(std::string name);
        virtual ~CartesianGeneratorPosDMP();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

    private:
        bool moveToDMP(geometry_msgs::Pose pose, double time, double t_scale, double k_number, double k_width);
	bool setWeight(std::vector<float> Wx,std::vector<float> Wy,std::vector<float> Wz);
	bool movePeriodic(KDL::Vector & amplitude_vector, float wx, float wy, float wz);
	bool moveCircle(KDL::Vector& amplitude_vector, float wx, float wy, float wz, KDL::Frame robot_table_transform);
	bool start_kinesthetic();
	bool start_kinesthetic_calibration(KDL::Frame table_robot_transform);
	bool resetWeight(int kernel_number);
	void setCartesianStiffness(float linear_stiffness, float angular_stiffness,float linear_damping, float angular_damping);
	void setCartesianStiffnessCalibration();
	std::vector<float> Weight_x,Weight_y,Weight_z;
        void resetPosition();
	void resetPositionActual();
	DMP *dmp;
	float err_pos_ok,err_angle_ok,step_time,kernel_width,time_scale;
	int kernel_number;
	bool pos_ok,angle_ok;
	float err_pos,err_angle;
	float x_dmp, y_dmp, z_dmp, theta_dmp, theta_eq;
	KDL::Vector axis_eq;
	float amp_x,amp_y, amp_z, omega_x, omega_y,omega_z;
        bool periodic_active,kinestetic_active,calibration_active,circle_active;

      KDL::Frame m_traject_end, m_traject_begin;
      KDL::Frame m_position_desi_local;
      KDL::Twist m_velocity_desi_local, m_velocity_begin_end, m_velocity_delta;
      KDL::Twist m_maximum_velocity, m_maximum_acceleration;
      geometry_msgs::Twist m_gm_maximum_velocity, m_gm_maximum_acceleration;
      geometry_msgs::Pose last_command;
      KDL::Frame table_robot_frame,calib_frame_table,robot_table_frame;
      bool last_command_real;
      geometry_msgs::Pose gm_starting_pose;

      std::vector<KDL::VelocityProfile_Trap>      m_motion_profile;
      RTT::os::TimeService::ticks                     m_time_begin;
      RTT::os::TimeService::Seconds                   m_time_passed;
      double                                      m_max_duration,m_duration;

      bool                                        m_is_moving,m_once;
      std::string								  move_started_event; 
      std::string								  move_finished_event;  
      KDL::Jacobian current_jacobian;
    protected:
      /// Dataport containing the current measured end-effector
      /// frame, shared with MotionControl::CartesianSensor
      RTT::InputPort< geometry_msgs::Pose >   m_position_meas_port;

      /// Dataport containing the current desired end-effector
      /// frame, shared with MotionControl::CartesianControllerPos,
      /// MotionControl::CartesianControllerPosVel
      RTT::OutputPort< geometry_msgs::Pose >  m_position_desi_port;
      RTT::OutputPort< geometry_msgs::Pose >  last_command_outport;
      /// Dataport containing the current desired end-effector
      /// twist, shared with MotionControl::CartesianControllerPosVel,
      /// MotionControl::CartesianControllerVel
      RTT::OutputPort< geometry_msgs::Twist >  m_velocity_desi_port;

      RTT::OutputPort<std::string> event_port;
      RTT::OutputPort<lwr_fri::CartesianImpedance> desired_stiffness_outport;
      RTT::InputPort <KDL::Jacobian > jacobian_inport;
      RTT::OutputPort<std::vector<double> > jacobian_svd_outport;
      RTT::InputPort<std::vector<double> > jacobian_svd_inport;
      std::vector<double> jacobian_svd;
      geometry_msgs::Pose current_pose;

  }; // class
} //namespace

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( MotionControl::CartesianGeneratorPosDMP )

#endif // __CARTESIAN_GENERATOR_POS_DMP_H__

