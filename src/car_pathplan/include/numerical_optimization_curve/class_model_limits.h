#ifndef CLASS_MODEL_LIMITS_H
#define CLASS_MODEL_LIMITS_H

using CppAD::AD;


extern const AD<double> velocity_forward_limit_ = 0.3;
extern const AD<double> velocity_reverse_limit_ = -0.3;

extern const AD<double> acc_positive_limit_ = 0.2;
extern const AD<double> acc_negative_limit_ = -0.2;

// extern const AD<double> jerk_positive_limit_ = 0.1;
// extern const AD<double> jerk_negative_limit_ = 0.1;

// extern const AD<double> angular_positive_limit_ = 0.5;
// extern const AD<double> angular_negative_limit_ = -0.5;

extern const AD<double> steer_angle_positive_limit_ = 0.8;
extern const AD<double> steer_angle_negative_limit_ = -0.8;

extern const AD<double> steer_rate_positive_limit_ = 0.5;
extern const AD<double> steer_rate_negative_limit_ = -0.5;

extern const AD<double> axle_distance_ = 0.25;


// extern const double velocity_forward_limit_ = 0.2;
// extern const double velocity_reverse_limit_ = -0.2;

// extern const double acc_positive_limit_ = 0.2;
// extern const double acc_negative_limit_ = 0.2;

// // extern const double jerk_positive_limit_ = 0.1;
// // extern const double jerk_negative_limit_ = 0.1;

// // extern const double angular_positive_limit_ = 0.5;
// // extern const double angular_negative_limit_ = -0.5;

// extern const double steer_angle_positive_limit_ = 0.5;
// extern const double steer_angle_negative_limit_ = -0.5;

// extern const double steer_rate_positive_limit_ = 0.5;
// extern const double steer_rate_negative_limit_ = -0.5;

// extern const double axle_distance_ = 0.25;



// class ClassModelLimits
// {
// private:
    
// public:
//     ClassModelLimits();
//     ~ClassModelLimits();
// };

// ClassModelLimits::ClassModelLimits()
// {
// }

// ClassModelLimits::~ClassModelLimits()
// {
// }



#endif