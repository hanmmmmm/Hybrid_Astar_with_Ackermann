
#include "utils/class_utils__converters.h"


namespace hawa
{   

// double ClassUtilsConverters::pi2 = M_PI * 2.0;

/**
 * @brief For a given yaw angle, generate the tf quaternion, by assuming the roll and pitch are 0.
 * @param yaw_rad The given yaw angle.
 * @return the tf quaternion.
*/
tf2::Quaternion ClassUtilsConverters::twodYawToTf2qua(double yaw_rad)
{
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw_rad);
    myQuaternion = myQuaternion.normalize();
    return myQuaternion;
}

/**
 * @brief Convert a given tf2 quaternion to geometry_msgs quaternion.
*/
geometry_msgs::msg::Quaternion ClassUtilsConverters::tf2quaToGeoQua(tf2::Quaternion tf2quaternion)
{
    geometry_msgs::msg::Quaternion myQuaternion;
    myQuaternion.w = tf2quaternion.getW();
    myQuaternion.x = tf2quaternion.getX();
    myQuaternion.y = tf2quaternion.getY();
    myQuaternion.z = tf2quaternion.getZ();
    return myQuaternion;
}

/**
 * @brief For a given tf stamped transform, find out the yaw value in it. 
 * @param ptr_tftrans The pointer to the transform. 
 * @return The yaw value. 
*/
double ClassUtilsConverters::StampedTransformToYaw(const geometry_msgs::msg::TransformStamped * ptr_trans)
{
    tf2::Quaternion q(ptr_trans->transform.rotation.x,
                      ptr_trans->transform.rotation.y,
                      ptr_trans->transform.rotation.z,
                      ptr_trans->transform.rotation.w);
    double yaw, pitch, roll;
    tf2::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod2pi(yaw);
}

/**
 * @brief For a given geometry quaternion, find out the yaw value in it. 
 * @param ptr_geoqua The pointer to the quaternion. 
 * @return The yaw value. 
*/
double ClassUtilsConverters::geoQuaToYaw(const geometry_msgs::msg::Quaternion* ptr_geoqua)
{
    tf2::Quaternion q(ptr_geoqua->x,
                      ptr_geoqua->y,
                      ptr_geoqua->z,
                      ptr_geoqua->w);
    double yaw, pitch, roll;
    tf2::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    return mod2pi(yaw);
}


/**
 * @brief Convert a StructPoseReal object to a std::array<double, 3> format.
*/
std::array<double, 3> ClassUtilsConverters::cvtStructPoseReal2Array3(const StructPoseReal& pose)
{
    return std::array<double, 3> {pose.x, pose.y, pose.yaw};
}

std::array<double, 3> ClassUtilsConverters::cvtStructPoseReal2Array3(StructPoseReal* pose)
{
    return std::array<double, 3> {pose->x, pose->y, pose->yaw};
}



/**
 * @brief Reset the values in a StructPoseReal back to 0.
*/
inline void ClassUtilsConverters::resetStructPoseReal(StructPoseReal* ptr_target)
{
    ptr_target->x = 0;
    ptr_target->y = 0;
    ptr_target->yaw = 0;
}

inline void ClassUtilsConverters::resetStructPoseReal(StructPoseReal& r_target)
{
    r_target.x = 0;
    r_target.y = 0;
    r_target.yaw = 0;
}


static inline std::array<int, 3> cvtStructPoseGrid2Array3(const StructPoseGrid& pose)
{
    return std::array<int, 3> {pose.x, pose.y, pose.yaw};
}

static inline std::array<int, 3> cvtStructPoseGrid2Array3(StructPoseGrid* pose)
{
    return std::array<int, 3> {pose->x, pose->y, pose->yaw};
}

static inline std::array<int, 2> cvtStructPoseGrid2Array2(const StructPoseGrid& pose)
{
    return std::array<int, 2> {pose.x, pose.y};
}

static inline std::array<int, 3> cvtStructPoseGrid2Array2(StructPoseGrid* pose)
{
    return std::array<int, 3> {pose->x, pose->y};
}


}

