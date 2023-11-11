// MIT License

// Copyright (c) 2023 Mingjie

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * This file is 
 */


#ifndef CLASS_HAWA_REEDSSHEPP_FINDER_H
#define CLASS_HAWA_REEDSSHEPP_FINDER_H

#include <iostream>
#include <string>
#include <queue>
#include <unordered_map>
#include <map>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <limits>
#include <math.h>

#include "hawa_reeds_shepp_curves/reedshepp_path.h" 
#include "reedsshepp_tools.h"

#include "../utils/hawa_conversion_tools.h"


#include "hawa_reeds_shepp_curves/rs_LSL.h"
#include "hawa_reeds_shepp_curves/rs_LSR.h"
#include "hawa_reeds_shepp_curves/rs_LRL.h"
#include "hawa_reeds_shepp_curves/rs_RLR.h"




/*
This is my implementation of Reeds-Sheep curve. 
Most parts are following the paper. 
One exceptation is that L+R+L+ L-R-L- R+L+R+ R-L-R- are also included;
so here are 52=48+4 types of curves are being considered;
And the CCC types are computed using Dubins Curve formula. 

The result is a vector of my custom path type, defined in another header file. 
Each 'path' contains the type-word, unitless length, sampled waypoints (x,y,yaw), valid (bool).
Sorted in length increasing order. 
*/


class ReedsSheppClass
{
private:

    StructPoseReal m_start_pose_;
    StructPoseReal m_goal_pose_;
    StructPoseReal m_goal_pose_processed_;

    bool check_collision_;

    StructSamplingProperties m_sampling_properites_;

    double calc_weighted_length(std::vector<double> list_in);

public:
    ReedsSheppClass();

    ~ReedsSheppClass();

    ClassReedSheppPath::PathCollection all_possible_paths_;

    std::vector<StructRSPathResult> results_;  

    void setup( StructPoseReal start_pose, StructPoseReal goal_pose );

    void search( );

    void get_path( std::array<double,3> start_pose, std::array<double,3> goal_pose , std::string path_type, std::vector<double>& path_angle, bool& valid);

    std::array<double, 2> calc_fix_frame_dxdy(const double angle_change, const double pose_yaw );

};


void ReedsSheppClass::setup(  StructPoseReal start_pose, StructPoseReal goal_pose )
{
    m_start_pose_ = start_pose;
    m_goal_pose_ = goal_pose;

    m_sampling_properites_.angular_step_size = 0.2;
    m_sampling_properites_.turning_radius = 0.5;
    m_sampling_properites_.linear_step_size = 0.1;

    double _dy = m_goal_pose_.y - m_start_pose_.y;
    double _dx = m_goal_pose_.x - m_start_pose_.x;
    double _cos_theta1 = std::cos(m_start_pose_.yaw);
    double _sin_theta1 = std::sin(m_start_pose_.yaw);

    m_goal_pose_processed_.x = ( _dx * _cos_theta1 + _dy * _sin_theta1) / m_sampling_properites_.turning_radius;
    m_goal_pose_processed_.y = (-_dx * _sin_theta1 + _dy * _cos_theta1) / m_sampling_properites_.turning_radius;
    m_goal_pose_processed_.yaw = mod2pi( m_goal_pose_.yaw - m_start_pose_.yaw );

    all_possible_paths_.resetAllPaths();
    while ( results_.size() > 0 ){
        results_.pop_back();
    }
}


void ReedsSheppClass::search(  )
{
    // CSC

    solveLpSpLp(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LpSpLp), results_);

    solveLmSmLm(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LmRmLm), results_);

    solveRpSpRp(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RpSpRp), results_);

    solveRmSmRm(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RmSmRm), results_);


    solveLpSpRp(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.LpSpRp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LpSpRp), results_);

    solveLmSmRm(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.LmSmRm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LmSmRm), results_);

    solveRpSpLp(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.RpSpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RpSpLp), results_);

    solveRmSmLm(m_goal_pose_processed_, m_start_pose_, &(all_possible_paths_.RmSmLm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RmSmLm), results_);

    // CCC LRL

    StructDubinsCurveCCCvalcollection _ccc_lrl_parameters;
    _ccc_lrl_parameters = prepareCCCLRL(m_goal_pose_, m_start_pose_, m_sampling_properites_.turning_radius);

    solveLpRpLp(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LpRpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LpRpLp), results_);

    solveLpRpLm(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LpRpLm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LpRpLm), results_);

    solveLpRmLp(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LpRmLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LpRmLp), results_);

    solveLpRmLm(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LpRmLm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LpRmLm), results_);

    solveLmRpLp(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LmRpLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LmRpLp), results_);

    solveLmRpLm(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LmRpLm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LmRpLm), results_);

    solveLmRmLp(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LmRmLp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LmRmLp), results_);

    solveLmRmLm(m_goal_pose_, m_start_pose_, 
                &_ccc_lrl_parameters, &(all_possible_paths_.LmRmLm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.LmRmLm), results_);

    // CCC RLR

    StructDubinsCurveCCCvalcollection _ccc_rlr_parameters;
    _ccc_rlr_parameters = prepareCCCRLR(m_goal_pose_, m_start_pose_, m_sampling_properites_.turning_radius);

    solveRpLpRp(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RpLpRp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RpLpRp), results_);

    solveRpLpRm(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RpLpRm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RpLpRm), results_);

    solveRpLmRp(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RpLmRp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RpLmRp), results_);

    solveRpLmRm(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RpLmRm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RpLmRm), results_);

    solveRmLpRp(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RmLpRp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RmLpRp), results_);

    solveRmLpRm(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RmLpRm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RmLpRm), results_);

    solveRmLmRp(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RmLmRp), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RmLmRp), results_);

    solveRmLmRm(m_goal_pose_, m_start_pose_, 
                &_ccc_rlr_parameters, &(all_possible_paths_.RmLmRm), &m_sampling_properites_);
    add_sort_path(&(all_possible_paths_.RmLmRm), results_);


    // TODO:

    // LpRupLumRm( ); // use (x,y,yaw)  orginal 
    // LmRumLupRp( ); // use (-x,y,-yaw) when m <=> p 
    // RpLupRumLm( ); // use (x,-y,-yaw) when R <=> L
    // RmLumRupLp( ); // use (-x,-y,yaw) when both of above happens 

    // LpRumLumRp( );
    // LmRupLupRm( );
    // RpLumRumLp( );
    // RmLupRupLm( );

    // LpRm90SmRm();
    // LmRp90SpRp();
    // RpLm90SmLm();
    // RmLp90SpLp();

    // LpRm90SmLm();
    // LmRp90SpLp();
    // RpLm90SmRm();
    // RmLp90SpRp();

    // RpSpLp90Rm();
    // RmSmLm90Rp();
    // LpSpRp90Lm();
    // LmSmRm90Lp();

    // LpSpLp90Rm();
    // LmSmLm90Rp();
    // RpSpRp90Lm();
    // RmSmRm90Lp();

    // LpRm90SmLm90Rp();
    // LmRp90SpLp90Rm();
    // RpLm90SmRm90Lp();
    // RmLp90SpRp90Lm();

    // if( results_.size() >= 1){
    //     std::cout << "\n\nResults" << std::endl;
    //     for( auto p : results_ ){
    //         std::cout << p.path_word << "    "  << p.path_length_unitless << std::endl;
    //     }
    // } 

}




// // 8.7
// void ReedsSheppClass::LpRupLumRm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
// {
//     double xi = x + sin(phi);
//     double eta = y - 1.0 - cos(phi);
//     double rho = 0.25 * (2.0 + sqrt(xi * xi + eta * eta));

//     if ( rho > 1.0 || rho < 0.0 ){
//         valid = false;
//         return;
//     }
//     valid = false;
//     u = acos(rho);
//     tauOmega(u, -u, xi, eta, phi, t, v);
//     if (t >= 0.0 && v <= 0.0 ){
//         valid = true;
//     }
// }


// void ReedsSheppClass::LpRupLumRm( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( x, y, phi, t, u, v, L, valid);
//     all_possible_paths_.LpRupLumRm.path_word = "LpRupLumRm";
//     all_possible_paths_.LpRupLumRm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.LpRupLumRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
//         get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRupLumRm);
//         all_possible_paths_.LpRupLumRm.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, -u, v});
//         add_sort_path( all_possible_paths_.LpRupLumRm );
//     }
//     // if(valid) std::cout << "LpRupLumRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::LmRumLupRp( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( -x, y, -phi, t, u, v, L, valid);
//     all_possible_paths_.LmRumLupRp.path_word = "LmRumLupRp";
//     all_possible_paths_.LmRumLupRp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.LmRumLupRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
//         get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRumLupRp);
//         all_possible_paths_.LmRumLupRp.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, u, -v});
//         add_sort_path( all_possible_paths_.LmRumLupRp );
//     }
//     // if(valid) std::cout << "LmRumLupRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::RpLupRumLm( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( x, -y, -phi, t, u, v, L, valid);
//     all_possible_paths_.RpLupRumLm.path_word = "RpLupRumLm";
//     all_possible_paths_.RpLupRumLm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.RpLupRumLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
//         get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLupRumLm);
//         all_possible_paths_.RpLupRumLm.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, -u, v});
//         add_sort_path( all_possible_paths_.RpLupRumLm );
//     }
//     // if(valid) std::cout << "RpLupRumLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::RmLumRupLp( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( -x, -y, phi, t, u, v, L, valid);
//     all_possible_paths_.RmLumRupLp.path_word = "RmLumRupLp";
//     all_possible_paths_.RmLumRupLp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.RmLumRupLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
//         get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLumRupLp);
//         all_possible_paths_.RmLumRupLp.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, u, -v});
//         add_sort_path( all_possible_paths_.RmLumRupLp );
//     }
//     // if(valid) std::cout << "RmLumRupLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// // 8.8
// void ReedsSheppClass::LpRumLumRp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
// {
//     double xi = x + sin(phi);
//     double eta = y - 1.0 - cos(phi);
//     double rho = (20.0 - xi * xi - eta * eta) / 16.0;

//     valid = false;
//     if (rho >= 0.0 && rho <= 1.0)
//     {
//         u = -acos(rho);
//         if (u >= -0.5 * M_PI)
//         {
//             tauOmega(u, u, xi, eta, phi, t, v);
//             valid = (t >= 0.0 && v >= 0.0);
//         }
//     }

// }


// void ReedsSheppClass::LpRumLumRp( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( x, y, phi, t, u, v, L, valid);
//     all_possible_paths_.LpRumLumRp.path_word = "LpRumLumRp";
//     all_possible_paths_.LpRumLumRp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.LpRumLumRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
//         get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
//         get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRumLumRp);
//         all_possible_paths_.LpRumLumRp.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, u, v});
//         add_sort_path( all_possible_paths_.LpRumLumRp );
//     }
//     // if(valid) std::cout << "LpRumLumRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::LmRupLupRm( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( -x, y, -phi, t, u, v, L, valid);
//     all_possible_paths_.LmRupLupRm.path_word = "LmRupLupRm";
//     all_possible_paths_.LmRupLupRm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.LmRupLupRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRupLupRm);
//         all_possible_paths_.LmRupLupRm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, -u, -v});
//         add_sort_path( all_possible_paths_.LmRupLupRm );
//     }
//     // if(valid) std::cout << "LmRupLupRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RpLumRumLp( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( x, -y, -phi, t, u, v, L, valid);
//     all_possible_paths_.RpLumRumLp.path_word = "RpLumRumLp";
//     all_possible_paths_.RpLumRumLp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.RpLumRumLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
//         get_samples_L( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
//         get_samples_R( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLumRumLp);
//         all_possible_paths_.RpLumRumLp.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, u, v});
//         add_sort_path( all_possible_paths_.RpLumRumLp );
//     }
//     // if(valid) std::cout << "RpLumRumLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RmLupRupLm( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( -x, -y, phi, t, u, v, L, valid);
//     all_possible_paths_.RmLupRupLm.path_word = "RmLupRupLm";
//     all_possible_paths_.RmLupRupLm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         all_possible_paths_.RmLupRupLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLupRupLm);
//         all_possible_paths_.RmLupRupLm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, -u, -v});
//         add_sort_path( all_possible_paths_.RmLupRupLm );
//     }
//     // if(valid) std::cout << "RmLupRupLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::LpRm90SmLm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
// {
//     double xi = x - sin(phi);
//     double eta = y - 1.0 + cos(phi);
//     valid = false;
//     double theta, rho;
//     polar( xi, eta, rho, theta);

//     if (rho >= 2.0){
//         double A = sqrt(rho * rho - 4.0);
//         u = 2.0 - A;
//         t = mod2pi(theta + atan2(A, -2.0));
//         v = mod2pi(phi - M_PI/2.0 - t);
//         if( t > 0.0 && u < 0.0 && v < 0.0 ){
//             valid = true;
//             L = std::abs(t) + M_PI/2.0 + std::abs(u) + std::abs(v) ; 
//         }
//         else{
//             valid = false;
//         }
//     }
//     else{
//         valid = false;
//         return;
//     }
// }


// void ReedsSheppClass::LpRm90SmLm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( x, y, phi, t, u, v, L, valid);
//     all_possible_paths_.LpRm90SmLm.path_word = "LpRm90SmLm";
//     all_possible_paths_.LpRm90SmLm.valid = valid;
//     if( valid ){
//         all_possible_paths_.LpRm90SmLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm);
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm);
//         all_possible_paths_.LpRm90SmLm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.LpRm90SmLm );
//     }
//     // if(valid) std::cout << "LpRm90SmLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ReedsSheppClass::LmRp90SpLp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( -x, y, -phi, t, u, v, L, valid);
//     all_possible_paths_.LmRp90SpLp.path_word = "LmRp90SpLp";
//     all_possible_paths_.LmRp90SpLp.valid = valid;
//     if( valid ){
//         all_possible_paths_.LmRp90SpLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp);
//         all_possible_paths_.LmRp90SpLp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.LmRp90SpLp );
//     }
//     // if(valid) std::cout << "LmRp90SpLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RpLm90SmRm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( x, -y, -phi, t, u, v, L, valid);
//     all_possible_paths_.RpLm90SmRm.path_word = "RpLm90SmRm";
//     all_possible_paths_.RpLm90SmRm.valid = valid;
//     if( valid ){
//         all_possible_paths_.RpLm90SmRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm);
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm);
//         all_possible_paths_.RpLm90SmRm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.RpLm90SmRm );
//     }
//     // if(valid) std::cout << "RpLm90SmRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RmLp90SpRp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( -x, -y, phi, t, u, v, L, valid);
//     all_possible_paths_.RmLp90SpRp.path_word = "RmLp90SpRp";
//     all_possible_paths_.RmLp90SpRp.valid = valid;
//     if( valid ){
//         all_possible_paths_.RmLp90SpRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp);
//         all_possible_paths_.RmLp90SpRp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.RmLp90SpRp );
//     }
//     // if(valid) std::cout << "RmLp90SpRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }





// void ReedsSheppClass::LpRm90SmRm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
// {
//     double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
//     polar(-eta, xi, rho, theta);
//     if (rho >= 2.)
//     {
//         t = theta;
//         u = 2. - rho;
//         v = mod2pi(t + .5 * M_PI - phi);

//         if (t >= 0.0 && u <= 0.0 && v <= 0.0){
//             valid = true;
//             L = std::abs(t) + M_PI/2.0 + std::abs(u) + std::abs(v) ; 
//         }
//         else{
//             valid = false;
//         }
//     }
//     else{
//         valid = false;
//     }

// }


// void ReedsSheppClass::LpRm90SmRm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( x, y, phi, t, u, v, L, valid);
//     all_possible_paths_.LpRm90SmRm.path_word = "LpRm90SmRm";
//     all_possible_paths_.LpRm90SmRm.valid = valid;
//     if( valid ){
//         all_possible_paths_.LpRm90SmRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmRm);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmRm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmRm);
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmRm);
//         all_possible_paths_.LpRm90SmRm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.LpRm90SmRm );
//     }
//     // if(valid) std::cout << "LpRm90SmRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::LmRp90SpRp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( -x, y, -phi, t, u, v, L, valid);
//     all_possible_paths_.LmRp90SpRp.path_word = "LmRp90SpRp";
//     all_possible_paths_.LmRp90SpRp.valid = valid;
//     if( valid ){
//         all_possible_paths_.LmRp90SpRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpRp);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpRp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpRp);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpRp);
//         all_possible_paths_.LmRp90SpRp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.LmRp90SpRp );
//     }
//     // if(valid) std::cout << "LmRp90SpRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RpLm90SmLm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( x, -y, -phi, t, u, v, L, valid);
//     all_possible_paths_.RpLm90SmLm.path_word = "RpLm90SmLm";
//     all_possible_paths_.RpLm90SmLm.valid = valid;
//     if( valid ){
//         all_possible_paths_.RpLm90SmLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmLm);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmLm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmLm);
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmLm);
//         all_possible_paths_.RpLm90SmLm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.RpLm90SmLm );
//     }
//     // if(valid) std::cout << "RpLm90SmLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RmLp90SpLp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( -x, -y, phi, t, u, v, L, valid);
//     all_possible_paths_.RmLp90SpLp.path_word = "RmLp90SpLp";
//     all_possible_paths_.RmLp90SpLp.valid = valid;
//     if( valid ){
//         all_possible_paths_.RmLp90SpLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpLp);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpLp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpLp);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpLp);
//         all_possible_paths_.RmLp90SpLp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.RmLp90SpLp );
//     }
//     // if(valid) std::cout << "RmLp90SpLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ReedsSheppClass::LmSmRm90Lp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( xb, yb, phi, t, u, v, L, valid);
//     all_possible_paths_.LmSmRm90Lp.path_word = "LmSmRm90Lp";
//     all_possible_paths_.LmSmRm90Lp.valid = valid;
//     if( valid ){
//         all_possible_paths_.LmSmRm90Lp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm90Lp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm90Lp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm90Lp);
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmRm90Lp);
//         all_possible_paths_.LmSmRm90Lp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.LmSmRm90Lp );
//     }
//     // if(valid) std::cout << "LmSmRm90Lp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::LpSpRp90Lm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( -xb, yb, -phi, t, u, v, L, valid);
//     all_possible_paths_.LpSpRp90Lm.path_word = "LpSpRp90Lm";
//     all_possible_paths_.LpSpRp90Lm.valid = valid;
//     if( valid ){
//         all_possible_paths_.LpSpRp90Lm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp90Lm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp90Lm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp90Lm);
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpRp90Lm);
//         all_possible_paths_.LpSpRp90Lm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.LpSpRp90Lm );
//     }
//     // if(valid) std::cout << "LpSpRp90Lm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::RmSmLm90Rp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( xb, -yb, -phi, t, u, v, L, valid);
//     all_possible_paths_.RmSmLm90Rp.path_word = "RmSmLm90Rp";
//     all_possible_paths_.RmSmLm90Rp.valid = valid;
//     if( valid ){
//         all_possible_paths_.RmSmLm90Rp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm90Rp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm90Rp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm90Rp);
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmLm90Rp);
//         all_possible_paths_.RmSmLm90Rp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.RmSmLm90Rp );
//     }
//     // if(valid) std::cout << "RmSmLm90Rp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ReedsSheppClass::RpSpLp90Rm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( -xb, -yb, phi, t, u, v, L, valid);
//     all_possible_paths_.RpSpLp90Rm.path_word = "RpSpLp90Rm";
//     all_possible_paths_.RpSpLp90Rm.valid = valid;
//     if( valid ){
//         all_possible_paths_.RpSpLp90Rm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp90Rm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp90Rm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp90Rm);
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpLp90Rm);
//         all_possible_paths_.RpSpLp90Rm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.RpSpLp90Rm );
//     }
//     // if(valid) std::cout << "RpSpLp90Rm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ReedsSheppClass::RmSmRm90Lp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( xb, yb, phi, t, u, v, L, valid);
//     all_possible_paths_.RmSmRm90Lp.path_word = "RmSmRm90Lp";
//     all_possible_paths_.RmSmRm90Lp.valid = valid;
//     if( valid ){
//         all_possible_paths_.RmSmRm90Lp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm90Lp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm90Lp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm90Lp);
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.RmSmRm90Lp);
//         all_possible_paths_.RmSmRm90Lp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.RmSmRm90Lp );
//     }
//     // if(valid) std::cout << "RmSmRm90Lp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RpSpRp90Lm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( -xb, yb, -phi, t, u, v, L, valid);
//     all_possible_paths_.RpSpRp90Lm.path_word = "RpSpRp90Lm";
//     all_possible_paths_.RpSpRp90Lm.valid = valid;
//     if( valid ){
//         all_possible_paths_.RpSpRp90Lm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp90Lm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp90Lm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp90Lm);
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RpSpRp90Lm);
//         all_possible_paths_.RpSpRp90Lm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.RpSpRp90Lm );
//     }
//     // if(valid) std::cout << "RpSpRp90Lm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::LmSmLm90Rp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( xb, -yb, -phi, t, u, v, L, valid);
//     all_possible_paths_.LmSmLm90Rp.path_word = "LmSmLm90Rp";
//     all_possible_paths_.LmSmLm90Rp.valid = valid;
//     if( valid ){
//         all_possible_paths_.LmSmLm90Rp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm90Rp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm90Rp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm90Rp);
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.LmSmLm90Rp);
//         all_possible_paths_.LmSmLm90Rp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( all_possible_paths_.LmSmLm90Rp );
//     }
//     // if(valid) std::cout << "LmSmLm90Rp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ReedsSheppClass::LpSpLp90Rm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( -xb, -yb, phi, t, u, v, L, valid);
//     all_possible_paths_.LpSpLp90Rm.path_word = "LpSpLp90Rm";
//     all_possible_paths_.LpSpLp90Rm.valid = valid;
//     if( valid ){
//         all_possible_paths_.LpSpLp90Rm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp90Rm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp90Rm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp90Rm);
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LpSpLp90Rm);
//         all_possible_paths_.LpSpLp90Rm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( all_possible_paths_.LpSpLp90Rm );
//     }
//     // if(valid) std::cout << "LpSpLp90Rm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }




// void ReedsSheppClass::LpRm90SmLm90Rp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
// {
//     double xi = x + sin(phi);
//     double eta = y - 1.0 - cos(phi);
//     double rho, theta;
//     polar(xi, eta, rho, theta);
//     if (rho < 2.0){
//         valid = false;
//         return;
//     }
//     u = 4.0 - sqrt(rho * rho - 4.0);
//     if (u <= 0.0){
//         t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
//         v = mod2pi(t - phi);
//         if (t >= 0.0 && v >= 0.0){
//             valid = true;
//             L = std::abs(t) + M_PI + std::abs(u) + std::abs(v) ; 
//         }
//         else{
//             valid= false;
//             return;
//         }
//     }
//     else{
//         valid = false;
//         return;
//     }

// }



// void ReedsSheppClass::LpRm90SmLm90Rp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( x, y, phi, t, u, v, L, valid);
//     all_possible_paths_.LpRm90SmLm90Rp.path_word = "LpRm90SmLm90Rp";
//     all_possible_paths_.LpRm90SmLm90Rp.valid = valid;
//     if( valid ){
//         all_possible_paths_.LpRm90SmLm90Rp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm90Rp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm90Rp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm90Rp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm90Rp);
//         get_samples_R( v, robot_theta, robot_x, robot_y, all_possible_paths_.LpRm90SmLm90Rp);
//         all_possible_paths_.LpRm90SmLm90Rp.path_length_unitless = calc_weighted_length(std::vector<double>{t, -M_PI/2.0, u, -M_PI/2.0, v});
//         add_sort_path( all_possible_paths_.LpRm90SmLm90Rp );
//     }
//     // if(valid) std::cout << "LpRm90SmLm90Rp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::LmRp90SpLp90Rm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( -x, y, -phi, t, u, v, L, valid);
//     all_possible_paths_.LmRp90SpLp90Rm.path_word = "LmRp90SpLp90Rm";
//     all_possible_paths_.LmRp90SpLp90Rm.valid = valid;
//     if( valid ){
//         all_possible_paths_.LmRp90SpLp90Rm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp90Rm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp90Rm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp90Rm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp90Rm);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, all_possible_paths_.LmRp90SpLp90Rm);
//         all_possible_paths_.LmRp90SpLp90Rm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, M_PI/2.0, -u, M_PI/2.0, -v});
//         add_sort_path( all_possible_paths_.LmRp90SpLp90Rm );
//     }
//     // if(valid) std::cout << "LmRp90SpLp90Rm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RpLm90SmRm90Lp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( x, -y, -phi, t, u, v, L, valid);
//     all_possible_paths_.RpLm90SmRm90Lp.path_word = "RpLm90SmRm90Lp";
//     all_possible_paths_.RpLm90SmRm90Lp.valid = valid;
//     if( valid ){
//         all_possible_paths_.RpLm90SmRm90Lp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm90Lp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm90Lp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm90Lp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm90Lp);
//         get_samples_L( v, robot_theta, robot_x, robot_y, all_possible_paths_.RpLm90SmRm90Lp);
//         all_possible_paths_.RpLm90SmRm90Lp.path_length_unitless = calc_weighted_length(std::vector<double>{t, -M_PI/2.0, u, -M_PI/2.0, v});
//         add_sort_path( all_possible_paths_.RpLm90SmRm90Lp );
//     }
//     // if(valid) std::cout << "RpLm90SmRm90Lp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ReedsSheppClass::RmLp90SpRp90Lm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( -x, -y, phi, t, u, v, L, valid);
//     all_possible_paths_.RmLp90SpRp90Lm.path_word = "RmLp90SpRp90Lm";
//     all_possible_paths_.RmLp90SpRp90Lm.valid = valid;
//     if( valid ){
//         all_possible_paths_.RmLp90SpRp90Lm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp90Lm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp90Lm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp90Lm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp90Lm);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, all_possible_paths_.RmLp90SpRp90Lm);
//         all_possible_paths_.RmLp90SpRp90Lm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, M_PI/2.0, -u, M_PI/2.0, -v});
//         add_sort_path( all_possible_paths_.RmLp90SpRp90Lm );
//     }
//     // if(valid) std::cout << "RmLp90SpRp90Lm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


double ReedsSheppClass::calc_weighted_length(std::vector<double> list_in)
{
    double _result = 99999999;
    if (list_in.size() < 1)
    {
        return _result;
    }

    for (double element : list_in)
    {
        if (element < 0.0)
        {
            _result += std::abs(element * 1.5);
        }
        else{
            _result += std::abs(element);
        }
    }

    return _result;
}


std::array<double, 2> ReedsSheppClass::calc_fix_frame_dxdy(const double angle_change, const double pose_yaw )
{
    double _body_frame_dx = m_sampling_properites_.turning_radius * sin(angle_change);
    double _body_frame_dy = -m_sampling_properites_.turning_radius 
                                + m_sampling_properites_.turning_radius * cos(angle_change);
    double _fix_frame_dx = _body_frame_dx * cos(pose_yaw) - _body_frame_dy * cos(M_PI/2.0 - pose_yaw);
    double _fix_frame_dy = _body_frame_dx * cos(M_PI/2.0 - pose_yaw) + _body_frame_dy * cos(pose_yaw);
    return std::array<double, 2>{_fix_frame_dx, _fix_frame_dy};
}


ReedsSheppClass::ReedsSheppClass(  )
{
}


ReedsSheppClass::~ReedsSheppClass()
{
}


#endif