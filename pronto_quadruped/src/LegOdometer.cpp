/* Copyright (c) 2015-2021
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "pronto_quadruped/LegOdometer.hpp"
#include <iit/rbd/utils.h>
#include <boost/math/distributions/normal.hpp>
#include <sstream>
#include <cmath>

namespace pronto {

using namespace pronto::quadruped;

LegOdometer::LegOdometer(FeetJacobians &feet_jacobians,
                         ForwardKinematics& forward_kinematics,
                         bool debug, SigmaMode s_mode, AverageMode a_mode) :
    feet_jacobians_(feet_jacobians),
    forward_kinematics_(forward_kinematics),
    debug_(debug),
    s_mode_(s_mode),
    a_mode_(a_mode),
    xd_b_(Eigen::Vector3d::Zero()),
    speed_limit_(12.42) // set speed limit to Usain Bolt's sprint record, Berlin August 16th, 2009
{
  moving_average_.resize(WINDOW);
  mov_ave.setZero();
}

LegOdometer::~LegOdometer() {
}

void LegOdometer::setMode(const SigmaMode s_mode, const AverageMode a_mode) {
  std::cerr << "[ LegOdometer ] Sigma Mode changed to: ";

  switch(s_mode){
  case SigmaMode::STATIC_SIGMA:
    std::cerr << "Static Sigma" << std::endl;
    break;
  case SigmaMode::VAR_SIGMA:
    std::cerr << "Var Sigma" << std::endl;
    break;
  case SigmaMode::VAR_AND_IMPACT_SIGMA:
    std::cerr << "Var and Impact Sigma" << std::endl;
    break;
  case SigmaMode::IMPACT_SIGMA:
    std::cerr << "Impact Sigma" << std::endl;
    break;
  default:
    throw std::runtime_error("Unknown SigmaMode");
    break;
  }

  s_mode_ = s_mode;
  std::cerr << "[ LegOdometer ] Average Mode changed to: ";
  switch (a_mode) {
  case AverageMode::SIMPLE_AVG:
    std::cerr << "Simple Average" << std::endl;
    break;
  case AverageMode::WEIGHTED_AVG:
    std::cerr << "Weighted Average" << std::endl;
    break;
  default:
    throw std::runtime_error("Unknown AverageMode");
    break;
  }
  a_mode_ = a_mode;
}

void LegOdometer::getMode(SigmaMode& s_mode, AverageMode& a_mode) {
    s_mode = s_mode_;
    a_mode = a_mode_;
}
std::string LegOdometer::printMode() {
    std::stringstream ss;

    ss << "[ LegOdometer ] Sigma Mode: ";
    switch(s_mode_){
    case SigmaMode::STATIC_SIGMA:
      ss << "Static Sigma" << std::endl;
      break;
    case SigmaMode::VAR_SIGMA:
      ss << "Var Sigma" << std::endl;
      break;
    case SigmaMode::VAR_AND_IMPACT_SIGMA:
      ss << "Var and Impact Sigma" << std::endl;
      break;
    case SigmaMode::IMPACT_SIGMA:
      ss << "Impact Sigma" << std::endl;
      break;
    default:
      break;
    }

    ss << "[ LegOdometer ] Average Mode: ";
    switch (a_mode_) {
    case AverageMode::SIMPLE_AVG:
      ss << "Simple Average" << std::endl;
      break;
    case AverageMode::WEIGHTED_AVG:
      ss << "Weighted Average" << std::endl;
      break;
    default:
      break;
    }
    return ss.str();
}

void LegOdometer::setInitVelocityCov(const Eigen::Matrix3d& vel_cov){
    initial_vel_cov_ = vel_cov;
}

void LegOdometer::setInitVelocityStd(const Eigen::Vector3d& vel_std){
    initial_vel_std_ = vel_std;
    vel_std_ = initial_vel_std_;
    initial_vel_cov_ = vel_std.array().square().matrix().asDiagonal();
    Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");
    std::cerr << "Set Initial standard deviation: " << std::endl;
    std::cerr << initial_vel_std_.format(clean_fmt) << std::endl;
    std::cerr << "Set Initial covariance: " << std::endl;
    std::cerr << initial_vel_cov_.format(clean_fmt) << std::endl;
}

void LegOdometer::setInitPositionCov(const Eigen::Matrix3d& pos_cov){
    pos_cov_ = pos_cov;
}

void LegOdometer::getVelocitiesFromLegs(LegVectorMap &vd) {
    vd = base_vel_leg_;
}

void LegOdometer::getFeetPositions(LegVectorMap &jd) {
    jd = foot_pos_;
}

bool LegOdometer::estimateVelocity(const uint64_t utime,
                                   const JointState &q,
                                   const JointState &qd,
                                   const Vector3d &omega,
                                   const LegBoolMap &stance_legs,
                                   const LegScalarMap &stance_prob,
                                   Vector3d &velocity,
                                   Matrix3d &covariance)
{
    // Eigen::Vector3d mov_ave;
    // mov_ave.setZero();
    vel_cov_ = initial_vel_cov_;

    // Recording foot position and base velocity from legs
    // JointState prova = JointState(1,2,3,4,5,6,7,8,9,10,11,12);
    // for(int leg = LF; leg <= RH; leg++)
    // {
      
    // }
    // std::cerr << "the qdot estimate vel are "<< qd.transpose()<<std::endl;
    foot_pos_ = forward_kinematics_.getFeetPos(q);
    for(int leg = LF; leg <= RH; leg++){
        // auto qd_block = qd.block<3,1>(leg * 3, 0);
        

        base_vel_leg_[LegID(leg)] = - feet_jacobians_.getFootJacobian(q, LegID(leg))
                            * qd.block<3,1>(leg * 3, 0) ; //+ Eigen::Vector3d(0.4,0.0,0.0);
                              - omega.cross(foot_pos_[LegID(leg)]);
       
        // base_vel_leg_[LegID(leg)] = -feet_jacobians_.getFootJacobian(q,LegID(leg)).block
        //                       - omega.cross(foot_pos_[LegID(leg)]);
        // std::cerr<< " the "<<leg<<"-th jacbian is "<<std::endl
        // for(int p = 0 ; p< 3; p++)
        // {
        //   base_vel_leg_[LegID(leg)](p) = std::fabs( base_vel_leg_[LegID(leg)](p));
        // }
    // std::cerr << " the "<<leg<<"-th jacobian is "<<std::endl<<feet_jacobians_.getFootJacobian(q, LegID(leg))<<std::endl;
    // if(leg == LegID::LF)
    //   std::cerr<<"the block data "<<leg <<"are "<<std::endl<<qd.block<3,1>(leg * 3, 0)<<std::endl;
    // std::cerr<<"the computation is "<<leg <<"is "<<std::endl<<-((base_vel_leg_[LegID(leg)]))<std::endl;
    // std::cerr << " the "<< leg<<"-th leg est_vel is "<<base_vel_leg_[LegID(leg)].transpose()<< "and the stance prob is "<< stance_legs[leg]<<std::endl;
    // std::cerr << " the "<<leg<<"-th jacobian is "<<std::endl<<feet_jacobians_.getFootJacobian(q, LegID(leg))<<std::endl;
    // std::cerr << " the "<<leg<<"-th qdot are "<<std::endl<<qd.block<3,1>(leg * 3, 0)<<std::endl;
    // std::cerr << " the "<<leg<<"-th q are "<<std::endl<<q.block<3,1>(leg * 3, 0)<<std::endl;
    // std::cerr << "the FK of "<<leg<<"-th leg is "<< foot_pos_[leg].transpose()<<std::endl;
    // std::cerr<< leg<< "-th the jacobian part is " <<  feet_jacobians_.getFootJacobian(q, LegID(leg))<<std::endl<<std::endl;
    }
    
    // std::cerr<< "FL correction by Jacobian " << base_vel_leg_[LegID(0)].transpose()<< std::endl;
    // std::cerr<< "FR correction by Jacobian " << base_vel_leg_[LegID(1)].transpose()<< std::endl<< std::endl;
    Eigen::Vector3d old_xd_b = xd_b_;
    xd_b_.setZero();
    old_leg_count = leg_count;
    // std::cerr<<"old is "<<old_leg_count<<" "<<leg_count<<std::endl;
    // If we want to perform weighted average over legs depending on the
    // probabilities of contact
    leg_count = 0;
    

    Eigen::Vector3d var_velocity = Eigen::Vector3d::Zero();

    if(a_mode_ == AverageMode::WEIGHTED_AVG) {
        double sum = 0;
        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                leg_count++;
                sum += stance_prob[i];
            }
        }

        if(leg_count == 0 || leg_count == 1) {
            return false;
        }

        // Computing weighted average velocity
        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                xd_b_ += stance_prob[i] * base_vel_leg_[i] / sum;
            }
        }

        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                // Computing weighted the standard deviation
                for(int j = 0; j < 3; j++) {
                    var_velocity(j) += stance_prob[i] * pow(xd_b_(j) - base_vel_leg_[i](j), 2) / sum;
                }
            }
        }

    } else if(a_mode_ == AverageMode::SIMPLE_AVG){
        // Computing average velocity
        for(int i = 0; i < 4; i++) {
            // std::cerr<<" the stance leg "<<i<<"-th leg is "<< stance_legs[i]<<std::endl;
            if(stance_legs[i]) {
                xd_b_ += base_vel_leg_[i];
                leg_count++;
                
            }
            // std::cerr<<"iteration is "<<i<<std::endl;
            
            // std::cerr<<" xd_b_ is "<< xd_b_ << std::endl;
        }
        // if(leg_count != 0)
        //   std::cerr<<" leg count is "<< leg_count<<std::endl;
        
        
        // if(leg_count == 1 || leg_count == 3) {
        //     xd_b_ = old_xd_b;
        // }
        // std::cerr<< "the leg count is "<< leg_count<<std::endl;
        // if(leg_count != 2 && leg_count != 4)
        // {
        //   std::cerr<<" the old leg count is "<<old_leg_count<<std::endl;
        //   if(old_leg_count == 2 || old_leg_count == 4)
        //   {
        //     xd_b_peak = old_xd_b; 
        //     std::cerr<<" x_peak has been updated : "<<xd_b_peak<<std::endl;
        //   }
        //   xd_b_ = xd_b_peak;
        // }

        // std::cerr << stance_legs << std::endl << stance_legs[0] + stance_legs[1] + stance_legs[2] + stance_legs[3]<< std::endl;
      
        // std::cerr<< "PDPDPPDPDPDPPDPDPD"<<std::endl;
        // if(stance_legs[0] + stance_legs[1] + stance_legs[2] + stance_legs[3] == 4)
        // {
        //   std::cerr << "leg count è uno devo tornare false PDPDPDPDPD"<<std::endl;
          
        // }
        if(leg_count == 0) { 
            // if(leg_count == 1)
            // {
            //   std::cerr<<"at time "
            // }

            // if(count == 0)
            // {
            //   xd_b_peak = old_xd_b;
            //   count ++;
            // }
            // xd_b_ = xd_b_peak;
            return false;
        }

        else if(leg_count == 1)
        {
          
          return false;
        }

        else
        {
          count = 0;
          xd_b_ /= (double)leg_count;
        }


        
        // mov_ave = xd_b_;
        // update_moving_mean(mov_ave);
        // // xd_b_ = mov_ave;
        // // std::cerr << " average value is "<< mov_ave(0)<<" meanwhile xb value is "<< xd_b_(0)<<" mean substitution index "<< claro<< std::endl<<std::endl;
        // if(std::abs(xd_b_(0) - old_xd_b(0)) > 0.01)
        // {
        //   if(count == 0)
        //   {
        //       mov_ave_peak = mov_ave;
        //       std::cerr<< "update peak average"<< mov_ave_peak.norm() <<std::endl;
        //       count ++;
        //   }
        //   // if((old_leg_count == 2 || old_leg_count == 4))
        //   // {
        //   //   xd_b_peak = old_xd_b;
        //   // }
        //   // std::cerr<< "hold the old correction "<< claro<< " act value.norm: "<<xd_b_.norm()<< " old value: "<<old_xd_b.norm()  <<std::endl;
      
        //   xd_b_ = mov_ave_peak;
        //   claro++;
        // }
        // else
        //   count = 0;
        // std::cerr<< "hold the old correction "<< claro<< " act value.norm: "<<xd_b_.norm()<< " old value: "<<old_xd_b.norm()  <<std::endl;

        // if(xd_b_.norm() > 0.05)
        // {
      
        // }
        if(xd_b_.norm() > 10){
          std::cerr << "+++++++++++++++++++++ABNORMAL VELOCITY: " << std::endl;
          Eigen::IOFormat clean(4, 0, ", ", "\n", "[", "]");
          std::cerr << xd_b_.format(clean) << std::endl;
          std::cerr << "Stance: " << stance_legs << std::endl;
          std::cerr << "Leg velocities: " << base_vel_leg_ << std::endl;
        }

        for(int i = 0; i < 4; i++) {
            if(stance_legs[i]) {
                // Computing the standard deviation
                for(int j = 0; j < 3; j++) {
                    var_velocity(j) += pow(xd_b_(j) - base_vel_leg_[i](j), 2);
                }
            }
        }
        var_velocity /= (double)leg_count; 
    }
    // std::cerr << " mean Odometry correction "<<xd_b_.transpose()<<std::endl;
    double alpha = 0.4;
    double beta = 0.3;
    double gamma = 0.8;
    double delta = 0.5;
    if(s_mode_ == SigmaMode::VAR_SIGMA) {
        // Compute the new sigma based on the covariance over stance legs.
        // leave unchanged if only one leg is on the ground!
        if(leg_count != 1) {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) +  (1 - beta) * sqrt(var_velocity(0))),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * sqrt(var_velocity(1))),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * sqrt(var_velocity(2)));
        }
    }

    if(s_mode_ == SigmaMode::IMPACT_SIGMA || s_mode_ == SigmaMode::VAR_AND_IMPACT_SIGMA) {
        // Featuring Jean-Claude Van Damme, twice
        double impact =  2 * 0.00109375 * grf_delta_.abs().mean();
        if(impact < 0.001 || std::isnan(impact)) {
            impact = 0.0;
            beta = 1;
            gamma = 1;
        }
        if(s_mode_ == SigmaMode::VAR_AND_IMPACT_SIGMA) {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta) * (delta * impact + (1 - delta) * sqrt(var_velocity(0)))),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(1)))),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(2))));

        } else {
            vel_std_ << vel_std_(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta)* impact),
                  vel_std_(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma)* impact),
                  vel_std_(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma)* impact);
        }

        // if the difference with the previous velocity is beyond the initial
        // value for the standard deviation, reject the measurement
        if((a_mode_ == AverageMode::WEIGHTED_AVG) && ((old_xd_b - xd_b_)(0) > initial_vel_std_(0) || (old_xd_b - xd_b_)(0) < -initial_vel_std_(0))) {
            old_xd_b = xd_b_;
            return false;
        }
    }
    
    var_velocity << vel_std_(0) * vel_std_(0), vel_std_(1) * vel_std_(1), vel_std_(2) * vel_std_(2);

    if(debug_) {
        double impact =  2 * 0.00109375 * std::abs(grf_delta_.mean());  // TODO: What is the magic number
        if(impact < 0.001 || std::isnan(impact)) {
            impact = 0.0;
            beta = 1;
            gamma = 1;
        }

        r_kse_var_debug << r_kse_var_debug(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) +  (1 - beta) * sqrt(var_velocity(0))),
                        r_kse_var_debug(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * sqrt(var_velocity(1))),
                        r_kse_var_debug(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * sqrt(var_velocity(2)));

        r_kse_impact_debug << r_kse_impact_debug(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta)* impact),
                           r_kse_impact_debug(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma)* impact),
                           r_kse_impact_debug(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma)* impact);

        r_kse_var_impact_debug << r_kse_var_impact_debug(0) * alpha + (1 - alpha) * (beta * initial_vel_std_(0) + (1 - beta) * (delta * impact + (1 - delta) * sqrt(var_velocity(0)))),
                               r_kse_var_impact_debug(1) * alpha + (1 - alpha) * (gamma * initial_vel_std_(1) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(1)))),
                               r_kse_var_impact_debug(2) * alpha + (1 - alpha) * (gamma * initial_vel_std_(2) + (1 - gamma) * (delta * impact + (1 - delta) * sqrt(var_velocity(2))));
    }
    

    vel_cov_ = var_velocity.asDiagonal();

    // Checks if the computed values are all finite before using them
    if(!xd_b_.allFinite()){
      return false;
    }

    // if we hit the speed limit, we reject the measurement
    if(xd_b_.norm() > speed_limit_){
      std::cerr << "Speed limit hit: " << xd_b_.norm() << " > " << speed_limit_ << std::endl;
      return false;
    }

    // Checks if the computed values are all finite before using them
    if(!vel_cov_.allFinite()){
        vel_cov_ = initial_vel_cov_;
    }
    // std::cerr << " mean Odometry correction "<<xd_b_.transpose()<<std::endl;
    velocity = xd_b_;
    
    covariance = vel_cov_;
    return true;
}
void LegOdometer::get_foot_corr(int i, Eigen::Vector3d& vec)
{
    vec = base_vel_leg_[LegID(i)];
}

LegVectorMap LegOdometer::getFootPos() {
    return foot_pos_;
}

void LegOdometer::setSpeedLimit(const double &limit){
  speed_limit_ = limit;
}

void LegOdometer::setGrf(const LegVectorMap &grf){
  Eigen::Array4d prev_grf_ = grf_;
  grf_ << grf[LF](2), grf[RF](2), grf[LH](2), grf[RH](2);
  grf_delta_ = grf_ - prev_grf_;
}

void LegOdometer::update_moving_mean(Eigen::Vector3d& vec)
{
    if(moving_average_elem_ < WINDOW)
    {
      moving_average_[index_] = vec;
      moving_average_elem_ ++;
      index_ ++;
      // std::cerr << "insert vec at index "<< index_<< " vec value is "<< vec.transpose()<<std::endl;
      vec.setZero();
      
    }
    else
    {
      // std::cerr<<"the new value is "<<vec.transpose()<<std::endl;
      index_ = (index_+1)%WINDOW;
      start_index_ = (start_index_+1)%WINDOW;
      moving_average_[index_] = vec;
      vec.setZero();
      for(int i = 0; i < WINDOW; i++)
      {
        // std::cerr<<"index "<< i << "vector is "<<moving_average_[i].transpose()<<std::endl;
        vec += moving_average_[i];
      }
      vec /= (double)WINDOW;
      // std::cerr << "insert vec at index "<< index_<< " average value is "<< vec.norm()<<std::endl;
    }

}

}  // namespace pronto
