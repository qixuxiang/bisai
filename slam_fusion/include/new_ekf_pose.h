#ifndef NEW_EKF_POSE_H
#define NEW_EKF_POSE_H
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
using namespace std;

class New_EKF_Pose {
public:
    //constructor
    New_EKF_Pose(){
        gps_init_flag = false;
        vel_init_flag = false;

        X = Eigen::VectorXd::Zero(5);
        P = Eigen::MatrixXd::Zero(5,5);
    }
    //destructor
    ~New_EKF_Pose(){}

    void gpsStateUpdate(const Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps, const double time);
    void velStateUpdate(const Eigen::Vector2d &Z_vel, const Eigen::Matrix2d &R_vel, const double time);
    Eigen::VectorXd readX();
    Eigen::MatrixXd readP();

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
