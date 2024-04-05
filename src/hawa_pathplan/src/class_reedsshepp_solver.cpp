
#include "class_reedsshepp_solver.h"

namespace hawa
{


void ClassReedsSheppSolver::setup(  StructPoseReal start_pose, StructPoseReal goal_pose )
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

    m_all_curves_.resetAllPaths();
    while ( m_vector_path_results_.size() > 0 ){
        m_vector_path_results_.pop_back();
    }
}

/**
 * @brief The main function to run. It will try to solve every allowed curve combination. Any option listed
 * in this function can be disabled by commenting it below.   
*/
void ClassReedsSheppSolver::search(  )
{
    // CSC

    solveLpSpLp(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LpSpLp), m_vector_path_results_);

    solveLmSmLm(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LmRmLm), m_vector_path_results_);

    solveRpSpRp(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RpSpRp), m_vector_path_results_);

    solveRmSmRm(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.LpSpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RmSmRm), m_vector_path_results_);


    solveLpSpRp(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.LpSpRp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LpSpRp), m_vector_path_results_);

    solveLmSmRm(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.LmSmRm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LmSmRm), m_vector_path_results_);

    solveRpSpLp(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.RpSpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RpSpLp), m_vector_path_results_);

    solveRmSmLm(m_goal_pose_processed_, m_start_pose_, &(m_all_curves_.RmSmLm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RmSmLm), m_vector_path_results_);

    // // CCC LRL

    StructDubinsCurveCCCvalcollection _ccc_lrl_parameters;
    _ccc_lrl_parameters = prepareCCCLRL(m_goal_pose_, m_start_pose_, m_sampling_properites_.turning_radius);

    solveLpRpLp(m_start_pose_, 
                &_ccc_lrl_parameters, &(m_all_curves_.LpRpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LpRpLp), m_vector_path_results_);

    solveLpRpLm(m_start_pose_, 
                &_ccc_lrl_parameters, &(m_all_curves_.LpRpLm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LpRpLm), m_vector_path_results_);

    // solveLpRmLp(m_start_pose_, 
    //             &_ccc_lrl_parameters, &(m_all_curves_.LpRmLp), &m_sampling_properites_);
    // add_sort_path(&(m_all_curves_.LpRmLp), m_vector_path_results_);

    solveLpRmLm(m_start_pose_, 
                &_ccc_lrl_parameters, &(m_all_curves_.LpRmLm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LpRmLm), m_vector_path_results_);

    solveLmRpLp(m_start_pose_, 
                &_ccc_lrl_parameters, &(m_all_curves_.LmRpLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LmRpLp), m_vector_path_results_);

    // solveLmRpLm(m_start_pose_, 
    //             &_ccc_lrl_parameters, &(m_all_curves_.LmRpLm), &m_sampling_properites_);
    // add_sort_path(&(m_all_curves_.LmRpLm), m_vector_path_results_);

    solveLmRmLp(m_start_pose_, 
                &_ccc_lrl_parameters, &(m_all_curves_.LmRmLp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LmRmLp), m_vector_path_results_);

    solveLmRmLm(m_start_pose_, 
                &_ccc_lrl_parameters, &(m_all_curves_.LmRmLm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.LmRmLm), m_vector_path_results_);

    // CCC RLR

    StructDubinsCurveCCCvalcollection _ccc_rlr_parameters;
    _ccc_rlr_parameters = prepareCCCRLR(m_goal_pose_, m_start_pose_, m_sampling_properites_.turning_radius);

    solveRpLpRp(m_start_pose_, 
                &_ccc_rlr_parameters, &(m_all_curves_.RpLpRp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RpLpRp), m_vector_path_results_);

    solveRpLpRm(m_start_pose_, 
                &_ccc_rlr_parameters, &(m_all_curves_.RpLpRm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RpLpRm), m_vector_path_results_);

    // solveRpLmRp(m_start_pose_, 
    //             &_ccc_rlr_parameters, &(m_all_curves_.RpLmRp), &m_sampling_properites_);
    // add_sort_path(&(m_all_curves_.RpLmRp), m_vector_path_results_);

    solveRpLmRm(m_start_pose_, 
                &_ccc_rlr_parameters, &(m_all_curves_.RpLmRm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RpLmRm), m_vector_path_results_);

    solveRmLpRp(m_start_pose_, 
                &_ccc_rlr_parameters, &(m_all_curves_.RmLpRp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RmLpRp), m_vector_path_results_);

    // solveRmLpRm(m_start_pose_, 
    //             &_ccc_rlr_parameters, &(m_all_curves_.RmLpRm), &m_sampling_properites_);
    // add_sort_path(&(m_all_curves_.RmLpRm), m_vector_path_results_);

    solveRmLmRp(m_start_pose_, 
                &_ccc_rlr_parameters, &(m_all_curves_.RmLmRp), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RmLmRp), m_vector_path_results_);

    solveRmLmRm(m_start_pose_, 
                &_ccc_rlr_parameters, &(m_all_curves_.RmLmRm), &m_sampling_properites_);
    add_sort_path(&(m_all_curves_.RmLmRm), m_vector_path_results_);


    // THe remaining will be refactored in the future:

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

    // if( m_vector_path_results_.size() >= 1){
    //     std::cout << "\n\nResults" << std::endl;
    //     for( auto p : m_vector_path_results_ ){
    //         std::cout << p.path_word << "    "  << p.path_length_unitless << std::endl;
    //     }
    // } 

}




// // 8.7
// void ClassReedsSheppSolver::LpRupLumRm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
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


// void ClassReedsSheppSolver::LpRupLumRm( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( x, y, phi, t, u, v, L, valid);
//     m_all_curves_.LpRupLumRm.path_word = "LpRupLumRm";
//     m_all_curves_.LpRupLumRm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.LpRupLumRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.LpRupLumRm);
//         get_samples_R( u, robot_theta, robot_x, robot_y, m_all_curves_.LpRupLumRm);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, m_all_curves_.LpRupLumRm);
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.LpRupLumRm);
//         m_all_curves_.LpRupLumRm.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, -u, v});
//         add_sort_path( m_all_curves_.LpRupLumRm );
//     }
//     // if(valid) std::cout << "LpRupLumRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::LmRumLupRp( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( -x, y, -phi, t, u, v, L, valid);
//     m_all_curves_.LmRumLupRp.path_word = "LmRumLupRp";
//     m_all_curves_.LmRumLupRp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.LmRumLupRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.LmRumLupRp);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, m_all_curves_.LmRumLupRp);
//         get_samples_L( u, robot_theta, robot_x, robot_y, m_all_curves_.LmRumLupRp);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.LmRumLupRp);
//         m_all_curves_.LmRumLupRp.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, u, -v});
//         add_sort_path( m_all_curves_.LmRumLupRp );
//     }
//     // if(valid) std::cout << "LmRumLupRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::RpLupRumLm( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( x, -y, -phi, t, u, v, L, valid);
//     m_all_curves_.RpLupRumLm.path_word = "RpLupRumLm";
//     m_all_curves_.RpLupRumLm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.RpLupRumLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.RpLupRumLm);
//         get_samples_L( u, robot_theta, robot_x, robot_y, m_all_curves_.RpLupRumLm);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, m_all_curves_.RpLupRumLm);
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.RpLupRumLm);
//         m_all_curves_.RpLupRumLm.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, -u, v});
//         add_sort_path( m_all_curves_.RpLupRumLm );
//     }
//     // if(valid) std::cout << "RpLupRumLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::RmLumRupLp( ){
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRupLumRm_base( -x, -y, phi, t, u, v, L, valid);
//     m_all_curves_.RmLumRupLp.path_word = "RmLumRupLp";
//     m_all_curves_.RmLumRupLp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.RmLumRupLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.RmLumRupLp);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, m_all_curves_.RmLumRupLp);
//         get_samples_R( u, robot_theta, robot_x, robot_y, m_all_curves_.RmLumRupLp);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.RmLumRupLp);
//         m_all_curves_.RmLumRupLp.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, u, -v});
//         add_sort_path( m_all_curves_.RmLumRupLp );
//     }
//     // if(valid) std::cout << "RmLumRupLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// // 8.8
// void ClassReedsSheppSolver::LpRumLumRp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
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


// void ClassReedsSheppSolver::LpRumLumRp( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( x, y, phi, t, u, v, L, valid);
//     m_all_curves_.LpRumLumRp.path_word = "LpRumLumRp";
//     m_all_curves_.LpRumLumRp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.LpRumLumRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.LpRumLumRp);
//         get_samples_R( u, robot_theta, robot_x, robot_y, m_all_curves_.LpRumLumRp);
//         get_samples_L( u, robot_theta, robot_x, robot_y, m_all_curves_.LpRumLumRp);
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.LpRumLumRp);
//         m_all_curves_.LpRumLumRp.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, u, v});
//         add_sort_path( m_all_curves_.LpRumLumRp );
//     }
//     // if(valid) std::cout << "LpRumLumRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::LmRupLupRm( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( -x, y, -phi, t, u, v, L, valid);
//     m_all_curves_.LmRupLupRm.path_word = "LmRupLupRm";
//     m_all_curves_.LmRupLupRm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.LmRupLupRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.LmRupLupRm);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, m_all_curves_.LmRupLupRm);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, m_all_curves_.LmRupLupRm);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.LmRupLupRm);
//         m_all_curves_.LmRupLupRm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, -u, -v});
//         add_sort_path( m_all_curves_.LmRupLupRm );
//     }
//     // if(valid) std::cout << "LmRupLupRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RpLumRumLp( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( x, -y, -phi, t, u, v, L, valid);
//     m_all_curves_.RpLumRumLp.path_word = "RpLumRumLp";
//     m_all_curves_.RpLumRumLp.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.RpLumRumLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.RpLumRumLp);
//         get_samples_L( u, robot_theta, robot_x, robot_y, m_all_curves_.RpLumRumLp);
//         get_samples_R( u, robot_theta, robot_x, robot_y, m_all_curves_.RpLumRumLp);
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.RpLumRumLp);
//         m_all_curves_.RpLumRumLp.path_length_unitless = calc_weighted_length(std::vector<double>{t, u, u, v});
//         add_sort_path( m_all_curves_.RpLumRumLp );
//     }
//     // if(valid) std::cout << "RpLumRumLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RmLupRupLm( )
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRumLumRp_base( -x, -y, phi, t, u, v, L, valid);
//     m_all_curves_.RmLupRupLm.path_word = "RmLupRupLm";
//     m_all_curves_.RmLupRupLm.valid = valid;
//     if( valid ){
//         L = std::abs(t) + std::abs(u)*2.0 + std::abs(v) ; 
//         m_all_curves_.RmLupRupLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.RmLupRupLm);
//         get_samples_L( -u, robot_theta, robot_x, robot_y, m_all_curves_.RmLupRupLm);
//         get_samples_R( -u, robot_theta, robot_x, robot_y, m_all_curves_.RmLupRupLm);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.RmLupRupLm);
//         m_all_curves_.RmLupRupLm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, -u, -u, -v});
//         add_sort_path( m_all_curves_.RmLupRupLm );
//     }
//     // if(valid) std::cout << "RmLupRupLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::LpRm90SmLm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
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


// void ClassReedsSheppSolver::LpRm90SmLm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( x, y, phi, t, u, v, L, valid);
//     m_all_curves_.LpRm90SmLm.path_word = "LpRm90SmLm";
//     m_all_curves_.LpRm90SmLm.valid = valid;
//     if( valid ){
//         m_all_curves_.LpRm90SmLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm);
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm);
//         m_all_curves_.LpRm90SmLm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.LpRm90SmLm );
//     }
//     // if(valid) std::cout << "LpRm90SmLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ClassReedsSheppSolver::LmRp90SpLp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( -x, y, -phi, t, u, v, L, valid);
//     m_all_curves_.LmRp90SpLp.path_word = "LmRp90SpLp";
//     m_all_curves_.LmRp90SpLp.valid = valid;
//     if( valid ){
//         m_all_curves_.LmRp90SpLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp);
//         m_all_curves_.LmRp90SpLp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.LmRp90SpLp );
//     }
//     // if(valid) std::cout << "LmRp90SpLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RpLm90SmRm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( x, -y, -phi, t, u, v, L, valid);
//     m_all_curves_.RpLm90SmRm.path_word = "RpLm90SmRm";
//     m_all_curves_.RpLm90SmRm.valid = valid;
//     if( valid ){
//         m_all_curves_.RpLm90SmRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm);
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm);
//         m_all_curves_.RpLm90SmRm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.RpLm90SmRm );
//     }
//     // if(valid) std::cout << "RpLm90SmRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RmLp90SpRp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm_base( -x, -y, phi, t, u, v, L, valid);
//     m_all_curves_.RmLp90SpRp.path_word = "RmLp90SpRp";
//     m_all_curves_.RmLp90SpRp.valid = valid;
//     if( valid ){
//         m_all_curves_.RmLp90SpRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp);
//         m_all_curves_.RmLp90SpRp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.RmLp90SpRp );
//     }
//     // if(valid) std::cout << "RmLp90SpRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }





// void ClassReedsSheppSolver::LpRm90SmRm_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
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


// void ClassReedsSheppSolver::LpRm90SmRm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( x, y, phi, t, u, v, L, valid);
//     m_all_curves_.LpRm90SmRm.path_word = "LpRm90SmRm";
//     m_all_curves_.LpRm90SmRm.valid = valid;
//     if( valid ){
//         m_all_curves_.LpRm90SmRm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmRm);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmRm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmRm);
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmRm);
//         m_all_curves_.LpRm90SmRm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.LpRm90SmRm );
//     }
//     // if(valid) std::cout << "LpRm90SmRm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::LmRp90SpRp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( -x, y, -phi, t, u, v, L, valid);
//     m_all_curves_.LmRp90SpRp.path_word = "LmRp90SpRp";
//     m_all_curves_.LmRp90SpRp.valid = valid;
//     if( valid ){
//         m_all_curves_.LmRp90SpRp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpRp);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpRp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpRp);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpRp);
//         m_all_curves_.LmRp90SpRp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.LmRp90SpRp );
//     }
//     // if(valid) std::cout << "LmRp90SpRp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RpLm90SmLm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( x, -y, -phi, t, u, v, L, valid);
//     m_all_curves_.RpLm90SmLm.path_word = "RpLm90SmLm";
//     m_all_curves_.RpLm90SmLm.valid = valid;
//     if( valid ){
//         m_all_curves_.RpLm90SmLm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmLm);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmLm);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmLm);
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmLm);
//         m_all_curves_.RpLm90SmLm.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.RpLm90SmLm );
//     }
//     // if(valid) std::cout << "RpLm90SmLm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RmLp90SpLp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmRm_base( -x, -y, phi, t, u, v, L, valid);
//     m_all_curves_.RmLp90SpLp.path_word = "RmLp90SpLp";
//     m_all_curves_.RmLp90SpLp.valid = valid;
//     if( valid ){
//         m_all_curves_.RmLp90SpLp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpLp);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpLp);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpLp);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpLp);
//         m_all_curves_.RmLp90SpLp.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.RmLp90SpLp );
//     }
//     // if(valid) std::cout << "RmLp90SpLp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ClassReedsSheppSolver::LmSmRm90Lp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( xb, yb, phi, t, u, v, L, valid);
//     m_all_curves_.LmSmRm90Lp.path_word = "LmSmRm90Lp";
//     m_all_curves_.LmSmRm90Lp.valid = valid;
//     if( valid ){
//         m_all_curves_.LmSmRm90Lp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.LmSmRm90Lp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.LmSmRm90Lp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LmSmRm90Lp);
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.LmSmRm90Lp);
//         m_all_curves_.LmSmRm90Lp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.LmSmRm90Lp );
//     }
//     // if(valid) std::cout << "LmSmRm90Lp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::LpSpRp90Lm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( -xb, yb, -phi, t, u, v, L, valid);
//     m_all_curves_.LpSpRp90Lm.path_word = "LpSpRp90Lm";
//     m_all_curves_.LpSpRp90Lm.valid = valid;
//     if( valid ){
//         m_all_curves_.LpSpRp90Lm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.LpSpRp90Lm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.LpSpRp90Lm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LpSpRp90Lm);
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.LpSpRp90Lm);
//         m_all_curves_.LpSpRp90Lm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.LpSpRp90Lm );
//     }
//     // if(valid) std::cout << "LpSpRp90Lm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::RmSmLm90Rp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( xb, -yb, -phi, t, u, v, L, valid);
//     m_all_curves_.RmSmLm90Rp.path_word = "RmSmLm90Rp";
//     m_all_curves_.RmSmLm90Rp.valid = valid;
//     if( valid ){
//         m_all_curves_.RmSmLm90Rp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.RmSmLm90Rp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.RmSmLm90Rp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RmSmLm90Rp);
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.RmSmLm90Rp);
//         m_all_curves_.RmSmLm90Rp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.RmSmLm90Rp );
//     }
//     // if(valid) std::cout << "RmSmLm90Rp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


// void ClassReedsSheppSolver::RpSpLp90Rm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmLm_base( -xb, -yb, phi, t, u, v, L, valid);
//     m_all_curves_.RpSpLp90Rm.path_word = "RpSpLp90Rm";
//     m_all_curves_.RpSpLp90Rm.valid = valid;
//     if( valid ){
//         m_all_curves_.RpSpLp90Rm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.RpSpLp90Rm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.RpSpLp90Rm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RpSpLp90Rm);
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.RpSpLp90Rm);
//         m_all_curves_.RpSpLp90Rm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.RpSpLp90Rm );
//     }
//     // if(valid) std::cout << "RpSpLp90Rm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ClassReedsSheppSolver::RmSmRm90Lp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( xb, yb, phi, t, u, v, L, valid);
//     m_all_curves_.RmSmRm90Lp.path_word = "RmSmRm90Lp";
//     m_all_curves_.RmSmRm90Lp.valid = valid;
//     if( valid ){
//         m_all_curves_.RmSmRm90Lp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.RmSmRm90Lp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.RmSmRm90Lp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RmSmRm90Lp);
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.RmSmRm90Lp);
//         m_all_curves_.RmSmRm90Lp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.RmSmRm90Lp );
//     }
//     // if(valid) std::cout << "RmSmRm90Lp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RpSpRp90Lm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( -xb, yb, -phi, t, u, v, L, valid);
//     m_all_curves_.RpSpRp90Lm.path_word = "RpSpRp90Lm";
//     m_all_curves_.RpSpRp90Lm.valid = valid;
//     if( valid ){
//         m_all_curves_.RpSpRp90Lm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.RpSpRp90Lm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.RpSpRp90Lm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RpSpRp90Lm);
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.RpSpRp90Lm);
//         m_all_curves_.RpSpRp90Lm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.RpSpRp90Lm );
//     }
//     // if(valid) std::cout << "RpSpRp90Lm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::LmSmLm90Rp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( xb, -yb, -phi, t, u, v, L, valid);
//     m_all_curves_.LmSmLm90Rp.path_word = "LmSmLm90Rp";
//     m_all_curves_.LmSmLm90Rp.valid = valid;
//     if( valid ){
//         m_all_curves_.LmSmLm90Rp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.LmSmLm90Rp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.LmSmLm90Rp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LmSmLm90Rp);
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.LmSmLm90Rp);
//         m_all_curves_.LmSmLm90Rp.path_length_unitless = calc_weighted_length(std::vector<double>{v, u, -M_PI/2.0, t});
//         add_sort_path( m_all_curves_.LmSmLm90Rp );
//     }
//     // if(valid) std::cout << "LmSmLm90Rp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }



// void ClassReedsSheppSolver::LpSpLp90Rm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     double xb = x * cos(phi) + y * sin(phi);
//     double yb = x * sin(phi) - y * cos(phi);
//     LpRm90SmRm_base( -xb, -yb, phi, t, u, v, L, valid);
//     m_all_curves_.LpSpLp90Rm.path_word = "LpSpLp90Rm";
//     m_all_curves_.LpSpLp90Rm.valid = valid;
//     if( valid ){
//         m_all_curves_.LpSpLp90Rm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.LpSpLp90Rm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.LpSpLp90Rm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LpSpLp90Rm);
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.LpSpLp90Rm);
//         m_all_curves_.LpSpLp90Rm.path_length_unitless = calc_weighted_length(std::vector<double>{-v, -u, M_PI/2.0, -t});
//         add_sort_path( m_all_curves_.LpSpLp90Rm );
//     }
//     // if(valid) std::cout << "LpSpLp90Rm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }




// void ClassReedsSheppSolver::LpRm90SmLm90Rp_base(double x, double y, double phi, double &t, double &u, double &v, double& L, bool& valid)
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



// void ClassReedsSheppSolver::LpRm90SmLm90Rp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( x, y, phi, t, u, v, L, valid);
//     m_all_curves_.LpRm90SmLm90Rp.path_word = "LpRm90SmLm90Rp";
//     m_all_curves_.LpRm90SmLm90Rp.valid = valid;
//     if( valid ){
//         m_all_curves_.LpRm90SmLm90Rp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( t, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm90Rp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm90Rp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm90Rp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm90Rp);
//         get_samples_R( v, robot_theta, robot_x, robot_y, m_all_curves_.LpRm90SmLm90Rp);
//         m_all_curves_.LpRm90SmLm90Rp.path_length_unitless = calc_weighted_length(std::vector<double>{t, -M_PI/2.0, u, -M_PI/2.0, v});
//         add_sort_path( m_all_curves_.LpRm90SmLm90Rp );
//     }
//     // if(valid) std::cout << "LpRm90SmLm90Rp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::LmRp90SpLp90Rm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( -x, y, -phi, t, u, v, L, valid);
//     m_all_curves_.LmRp90SpLp90Rm.path_word = "LmRp90SpLp90Rm";
//     m_all_curves_.LmRp90SpLp90Rm.valid = valid;
//     if( valid ){
//         m_all_curves_.LmRp90SpLp90Rm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_L( -t, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp90Rm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp90Rm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp90Rm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp90Rm);
//         get_samples_R( -v, robot_theta, robot_x, robot_y, m_all_curves_.LmRp90SpLp90Rm);
//         m_all_curves_.LmRp90SpLp90Rm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, M_PI/2.0, -u, M_PI/2.0, -v});
//         add_sort_path( m_all_curves_.LmRp90SpLp90Rm );
//     }
//     // if(valid) std::cout << "LmRp90SpLp90Rm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RpLm90SmRm90Lp()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( x, -y, -phi, t, u, v, L, valid);
//     m_all_curves_.RpLm90SmRm90Lp.path_word = "RpLm90SmRm90Lp";
//     m_all_curves_.RpLm90SmRm90Lp.valid = valid;
//     if( valid ){
//         m_all_curves_.RpLm90SmRm90Lp.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( t, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm90Lp);
//         get_samples_L( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm90Lp);
//         get_samples_S( u, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm90Lp);
//         get_samples_R( -M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm90Lp);
//         get_samples_L( v, robot_theta, robot_x, robot_y, m_all_curves_.RpLm90SmRm90Lp);
//         m_all_curves_.RpLm90SmRm90Lp.path_length_unitless = calc_weighted_length(std::vector<double>{t, -M_PI/2.0, u, -M_PI/2.0, v});
//         add_sort_path( m_all_curves_.RpLm90SmRm90Lp );
//     }
//     // if(valid) std::cout << "RpLm90SmRm90Lp  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }

// void ClassReedsSheppSolver::RmLp90SpRp90Lm()
// {
//     double x   = goal_pose_processed_[0];
//     double y   = goal_pose_processed_[1];
//     double phi = goal_pose_processed_[2];
//     double t, u, v, L; bool valid;
//     LpRm90SmLm90Rp_base( -x, -y, phi, t, u, v, L, valid);
//     m_all_curves_.RmLp90SpRp90Lm.path_word = "RmLp90SpRp90Lm";
//     m_all_curves_.RmLp90SpRp90Lm.valid = valid;
//     if( valid ){
//         m_all_curves_.RmLp90SpRp90Lm.path_length_unitless = L; 
//         double robot_x     = start_pose_[0] ;
//         double robot_y     = start_pose_[1] ;
//         double robot_theta = start_pose_[2] ;
//         get_samples_R( -t, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp90Lm);
//         get_samples_L( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp90Lm);
//         get_samples_S( -u, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp90Lm);
//         get_samples_R( M_PI/2.0, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp90Lm);
//         get_samples_L( -v, robot_theta, robot_x, robot_y, m_all_curves_.RmLp90SpRp90Lm);
//         m_all_curves_.RmLp90SpRp90Lm.path_length_unitless = calc_weighted_length(std::vector<double>{-t, M_PI/2.0, -u, M_PI/2.0, -v});
//         add_sort_path( m_all_curves_.RmLp90SpRp90Lm );
//     }
//     // if(valid) std::cout << "RmLp90SpRp90Lm  " << valid  << "  t:" << t << "  u:" << u << "  v:" << v << std::endl;
// }


double ClassReedsSheppSolver::calc_weighted_length(std::vector<double> list_in)
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


std::array<double, 2> ClassReedsSheppSolver::calcFixFrameDxDy(const double angle_change, const double pose_yaw )
{
    double _body_frame_dx = m_sampling_properites_.turning_radius * sin(angle_change);
    double _body_frame_dy = -m_sampling_properites_.turning_radius 
                                + m_sampling_properites_.turning_radius * cos(angle_change);
    double _fix_frame_dx = _body_frame_dx * cos(pose_yaw) - _body_frame_dy * cos(M_PI/2.0 - pose_yaw);
    double _fix_frame_dy = _body_frame_dx * cos(M_PI/2.0 - pose_yaw) + _body_frame_dy * cos(pose_yaw);
    return std::array<double, 2>{_fix_frame_dx, _fix_frame_dy};
}



}

