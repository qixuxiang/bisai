#ifndef ALL_EKF_POSE_H
#define ALL_EKF_POSE_H
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
using namespace std;

class All_EKF_Pose {
public:
    //constructor
    All_EKF_Pose(){
        gps_init_flag = false;
        vel_init_flag = false;

        X = Eigen::VectorXd::Zero(5);
        P = Eigen::MatrixXd::Zero(5,5);
    }
    //destructor
    ~All_EKF_Pose(){}

    void gpsStateUpdate(const Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps, const double time);
    void slamStateUpdate(const Eigen::Vector3d &Z_slam, const Eigen::Matrix3d &R_slam, const double time);
    void velStateUpdate(const Eigen::Vector2d &Z_vel, const Eigen::Matrix2d &R_vel, const double time);
    Eigen::VectorXd readX(double time);
    Eigen::MatrixXd readP(double time);

private:
    bool gps_init_flag;
    bool vel_init_flag;
    double time_now;
    Eigen::VectorXd X;//x1:pos_x,x2:pos_y,x3:yaw,x4:v;x5:w
    Eigen::MatrixXd P;
    void statePrediction(double dt);
    void constrainRadian(double &x);
};
#endif
