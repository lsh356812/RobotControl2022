/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;


namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        //* Index setting for each joint
        
        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}

//get transform I0
MatrixXd getTransformI0(){
    MatrixXd tmp_m(4, 4);   
    
    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, 0, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}

MatrixXd getTransform6E(){
    MatrixXd tmp_m(4,4);   
    
    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, -0.09, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}

MatrixXd JointToTransform01(VectorXd q){  //yaw
    //q : generalized cordinates(q1,q2,q3)
    
    
    MatrixXd tmp_m(4,4);   
    double qq = q(0);
    
    tmp_m << cos(qq), -sin(qq), 0, 0, \
             sin(qq), cos(qq), 0, 0.105, \
             0, 0, 1, -0.1512, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}
MatrixXd JointToTransform12(VectorXd q){   //roll
    //q : generalized cordinates(q1,q2,q3)...
    
    
    MatrixXd tmp_m(4,4);   
    double qq = q(1);
    
    tmp_m << 1, 0, 0, 0, \
             0, cos(qq), -sin(qq), 0, \
             0, sin(qq), cos(qq), 0, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}
MatrixXd JointToTransform23(VectorXd q){   //pitch
    //q : generalized cordinates(q1,q2,q3)
    
    
    MatrixXd tmp_m(4,4);   
    double qq = q(2);
    
    tmp_m << cos(qq), 0, sin(qq), 0, \
             0, 1, 0, 0, \
             -sin(qq), 0, cos(qq), 0, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}
MatrixXd JointToTransform34(VectorXd q){  //pitch
    //q : generalized cordinates(q1,q2,q3)
    
    
    MatrixXd tmp_m(4,4);   
    double qq = q(3);
    
    tmp_m << cos(qq), 0, sin(qq), 0, \
             0, 1, 0, 0, \
             -sin(qq), 0, cos(qq), -0.35, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}
MatrixXd JointToTransform45(VectorXd q){    //pitch
    //q : generalized cordinates(q1,q2,q3)
    
    
    MatrixXd tmp_m(4,4);   
    double qq = q(4);
    
    tmp_m << cos(qq), 0, sin(qq), 0, \
             0, 1, 0, 0, \
             -sin(qq), 0, cos(qq), -0.35, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}
MatrixXd JointToTransform56(VectorXd q){    //roll
    //q : generalized cordinates(q1,q2,q3)
    
    
   
    MatrixXd tmp_m(4,4);   
    double qq = q(5);
    
        tmp_m << 1, 0, 0, 0, \
             0, cos(qq), -sin(qq), 0, \
             0, sin(qq), cos(qq), 0, \
             0, 0, 0, 1;
    
    
    
    
    return tmp_m;
}

VectorXd jointToPosition(VectorXd q){
     MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);   
    T34 = JointToTransform34(q);   
    T45 = JointToTransform45(q);   
    T56 = JointToTransform56(q);   
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    Vector3d pos;
    
    pos = TIE.block(0,3,3,1);
    
    return pos;
}

MatrixXd jointToRotMat(VectorXd q){
    //VectorXd tmp_r = VectorXd::Zero(3);
   MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);   
    T34 = JointToTransform34(q);   
    T45 = JointToTransform45(q);   
    T56 = JointToTransform56(q);   
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    MatrixXd rot_m(3,3);
    rot_m<<TIE(0,0),TIE(0,1),TIE(0,2),\
          TIE(1,0),TIE(1,1),TIE(1,2),\
          TIE(2,0),TIE(2,1),TIE(2,2);
    return rot_m;
}

VectorXd rotToEuler(MatrixXd tmp_m){
    Vector3d q_e;
    q_e(0) = atan2(tmp_m(1,0), tmp_m(0,0));
    q_e(1) = atan2(-tmp_m(2,0), sqrt(tmp_m(2,1)*tmp_m(2,1) + tmp_m(2,2)*tmp_m(2,2)));
    q_e(2) = atan2(tmp_m(2,1),tmp_m(2,2));
    
    return q_e;
}
MatrixXd jointToPosJac(VectorXd q){
    
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4), T_IE(4,4); 
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4); 
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6; 
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6; 
    Vector3d r_I_IE;
    
    T_I0 = getTransformI0();
    T_01 = JointToTransform01(q);
    T_12 = JointToTransform12(q);
    T_23 = JointToTransform23(q);  
    T_34 = JointToTransform34(q);
    T_45 = JointToTransform45(q);
    T_56 = JointToTransform56(q);
    T_6E = getTransform6E();
    
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;                              //=T_I1*T_12
    T_I3 = T_I0*T_01*T_12*T_23;                       //=T_I2*T_23
    T_I4 = T_I0*T_01*T_12*T_23*T_34;                //=T_I3*T_34
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;         //=T_I4*T_45
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;  //=T_I5*T_56
    
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);
    
    r_I_I1 = T_I1.block(0,3,3,1);
    r_I_I2 = T_12.block(0,3,3,1);
    r_I_I3 = T_23.block(0,3,3,1);
    r_I_I4 = T_34.block(0,3,3,1);
    r_I_I5 = T_45.block(0,3,3,1);
    r_I_I6 = T_56.block(0,3,3,1);
    
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;
    
    n_I_1 = R_I1*n_1;
    n_I_2 = R_I2*n_2;
    n_I_3 = R_I3*n_3;
    n_I_4 = R_I4*n_4;
    n_I_5 = R_I5*n_5;
    n_I_6 = R_I6*n_6;
    
    T_IE = T_I6* T_6E;
    r_I_IE = T_IE.block(0,3,3,1);
    
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE-r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE-r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE-r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE-r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE-r_I_I6);
    
    return J_P;
}

MatrixXd jointToRotJac(VectorXd q){
    
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4), T_IE(4,4); 
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4); 
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6; 
    
    T_I0 = getTransformI0();
    T_01 = JointToTransform01(q);
    T_12 = JointToTransform12(q);
    T_23 = JointToTransform23(q);  
    T_34 = JointToTransform34(q);
    T_45 = JointToTransform45(q);
    T_56 = JointToTransform56(q);
    T_6E = getTransform6E();
    
    T_I1 = T_I0*T_01;
    T_I2 = T_I0*T_01*T_12;                              //=T_I1*T_12
    T_I3 = T_I0*T_01*T_12*T_23;                       //=T_I2*T_23
    T_I4 = T_I0*T_01*T_12*T_23*T_34;                //=T_I3*T_34
    T_I5 = T_I0*T_01*T_12*T_23*T_34*T_45;         //=T_I4*T_45
    T_I6 = T_I0*T_01*T_12*T_23*T_34*T_45*T_56;  //=T_I5*T_56
    
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = T_I2.block(0,0,3,3);
    R_I3 = T_I3.block(0,0,3,3);
    R_I4 = T_I4.block(0,0,3,3);
    R_I5 = T_I5.block(0,0,3,3);
    R_I6 = T_I6.block(0,0,3,3);
    
    n_1 << 0,0,1;
    n_2 << 1,0,0;
    n_3 << 0,1,0;
    n_4 << 0,1,0;
    n_5 << 0,1,0;
    n_6 << 1,0,0;
    
    J_R.col(0) << R_I1*n_1;
    J_R.col(1) << R_I2*n_2;
    J_R.col(2) << R_I3*n_3;
    J_R.col(3) << R_I4*n_4;
    J_R.col(4) << R_I5*n_5;
    J_R.col(5) << R_I6*n_6;
    
    
    return J_R;
}

MatrixXd pseudoInverseMat(MatrixXd A, double lambda){
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA;
    MatrixXd tmp_m;

    int m = A.rows();
    int n = A.cols();

    MatrixXd A_T = A.transpose();

    if(m >= n){
        tmp_m = A_T*A + lambda*lambda*(MatrixXd::Identity(n,n));
        pinvA = tmp_m.inverse()*A_T;
        std::cout << "pinvAl : " << pinvA << std::endl;
    }

    else{
        tmp_m = A*A_T + lambda*lambda*(MatrixXd::Identity(m,m));
        pinvA = A_T*tmp_m.inverse();
        std::cout << "pinvAr : " << pinvA << std::endl;
    }
     
    return pinvA;
}

VectorXd rotMatToRotVec(MatrixXd C)
{
    Vector3d phi,n;
    double th;

    th = acos((C(0,0)+C(1,1)+C(2,2)-1)/2);

    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{
        n << C(2,1)-C(1,2), \
             C(0,2)-C(2,0), \
             C(1,0)-C(0,1);
        n = n/(2*sin(th));
    }

    phi = th*n;

    return phi;
}

MatrixXd jointToGeoJac(VectorXd q)
{
    MatrixXd Jacobian = MatrixXd::Zero(6,6);
    Jacobian << jointToPosJac(q), jointToRotJac(q);

    return Jacobian;
}
VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    double num_it;
    MatrixXd J_P(3,6), J_R(3,6), J(6,6), pinvJ(6,6), C_err(3,3), C_IE(3,3);
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr, dph;
    double lambda;

    //* Set maximum number of iterations
    double max_it = 200;

    //* Initialize the solution with the initial guess
    q=q0;
    C_IE = jointToRotMat(q);
    C_err = C_des * C_IE.transpose();

    //* Damping factor
    lambda = 0.001;

    //* Initialize error
    dr = r_des - jointToPosition(q);
    dph =  rotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////

    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {

        //Compute Inverse Jacobian
        J_P = jointToPosJac(q);
        J_R = jointToRotJac(q);

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian

        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;

        // Update law
        q += 0.5*dq;

        // Update error
        C_IE = jointToRotMat(q);
        C_err = C_des * C_IE.transpose();

        dr = r_des - jointToPosition(q);
        dph = rotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);

        num_it++;
    }
    std::cout << "iteration: " << num_it << ", value: " << q << std::endl;

    return q;
}


void Practice(){
    //0411 실습
    MatrixXd TI0(4,4),T6E(4,4),T01(4,4),T12(4,4),T23(4,4),T34(4,4),T45(4,4),T56(4,4),TIE(4,4);
    MatrixXd pos(3,3), CIE(3,3);
    VectorXd q(6);
    q << 10*D2R, 20*D2R, 30*D2R, 40*D2R, 50*D2R, 60*D2R;
    Vector3d euler;
    MatrixXd test_pos_Jac(3,6), test_rot_Jac(3,6);
    
    //0502 실습 jointToPosJac
    test_pos_Jac << jointToPosJac(q);
    test_rot_Jac << jointToRotJac(q);
    std::cout << "Test, J_P : "<< test_pos_Jac << std::endl;
    std::cout << "Test, J_R : "<< test_rot_Jac << std::endl;
    /*TI0 = getTransformI0();
    T6E = getTransform6E();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);   
    T34 = JointToTransform34(q);   
    T45 = JointToTransform45(q);   
    T56 = JointToTransform56(q);   
    TIE = TI0*T01*T12*T23*T34*T45*T56*T6E;
    
    pos = jointToPosition(q);
    CIE = jointToRotMat(q);
    euler = rotToEuler(CIE);*/
    
    //0513 실습
    MatrixXd J(6,6);
    J << jointToPosJac(q),\
         jointToRotJac(q);

    MatrixXd pinvj;
    pinvj = pseudoInverseMat(J, 0.0);

    MatrixXd invj;
    invj = J.inverse();
    std::cout<<" Test, Inverse"<<invj<<std::endl;
    std::cout<<" Test, PseudoInverse"<<pinvj<<std::endl;
    
    VectorXd q_des(6),q_init(6);
    MatrixXd C_err(3,3), C_des(3,3), C_init(3,3);
    MatrixXd r_des(3,3);
    VectorXd q_cal(6);
    q_init = 0.5*q_des;
    C_des = jointToRotMat(q_des);
    C_init = jointToRotMat(q_init);
    C_err = C_des * C_init.transpose();

    VectorXd dph(3);

    dph = rotMatToRotVec(C_err);
    
    std::cout<<" Test, Rotational Vector"<<pinvj<<std::endl;
    r_des = jointToPosition(q);
    C_des = jointToRotMat(q);
    //r_des << 0, 0.105, -0.55;
    //C_des << 0, 0, 0, 0, \
            0, 0, 0, 0, \
            0, 0, 0, 0, \
            0, 0, 0, 0;

    q_cal = inverseKinematics(r_des, C_des, q*0.5, 0.001);
    std::cout << "IK result : "<< q_cal << std::endl;
    /*std::cout << "Hello world" << std::endl;
    std::cout << "TI0 : "<< TI0 << std::endl;
    std::cout << "T3E : "<< T3E << std::endl;
    std::cout << "T01 : "<< T01 << std::endl;
    std::cout << "T12 : "<< T12 << std::endl;
    std::cout << "T23 : "<< T23 << std::endl;
    std::cout << "TIE : "<< TIE << std::endl;
    
    std::cout << "Position : "<< pos << std::endl;
    std::cout << "CIE : "<< CIE << std::endl;
    std::cout << "Euler : "<< euler << std::endl;*/
    
    
}


void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */
    Practice();
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("/home/lsh356812/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();


    //* setting for getting dt
    last_update_time = model->GetWorld()->GetSimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));

}

void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*	
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->GetSimTime();
    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
    GetjointData();
    
    joint[LHY].targetRadian = 10*3.14/180;
    joint[LHR].targetRadian = 20*3.14/180;
    joint[LHP].targetRadian = 30*3.14/180;
    joint[LKN].targetRadian = 40*3.14/180;
    joint[LAP].targetRadian = 50*3.14/180;
    joint[LAR].targetRadian = 60*3.14/180;
    
    
    
    
    
    
    
    //* Joint Controller
    jointController();
}

void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian-joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity-joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);

    torso_joint->SetForce(0, joint[WST].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");

    //* FTsensor joint
    LS = this->model->GetJoint("LS");
    RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->GetAngle(0).Radian();
    joint[LHR].actualRadian = L_Hip_roll_joint->GetAngle(0).Radian();
    joint[LHP].actualRadian = L_Hip_pitch_joint->GetAngle(0).Radian();
    joint[LKN].actualRadian = L_Knee_joint->GetAngle(0).Radian();
    joint[LAP].actualRadian = L_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[LAR].actualRadian = L_Ankle_roll_joint->GetAngle(0).Radian();

    joint[RHY].actualRadian = R_Hip_yaw_joint->GetAngle(0).Radian();
    joint[RHR].actualRadian = R_Hip_roll_joint->GetAngle(0).Radian();
    joint[RHP].actualRadian = R_Hip_pitch_joint->GetAngle(0).Radian();
    joint[RKN].actualRadian = R_Knee_joint->GetAngle(0).Radian();
    joint[RAP].actualRadian = R_Ankle_pitch_joint->GetAngle(0).Radian();
    joint[RAR].actualRadian = R_Ankle_roll_joint->GetAngle(0).Radian();

    joint[WST].actualRadian = torso_joint->GetAngle(0).Radian();

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    joint[LHY].Kp = 2000;
    joint[LHR].Kp = 9000;
    joint[LHP].Kp = 2000;
    joint[LKN].Kp = 5000;
    joint[LAP].Kp = 3000;
    joint[LAR].Kp = 3000;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[WST].Kp = 2.;

    joint[LHY].Kd = 2.;
    joint[LHR].Kd = 2.;
    joint[LHP].Kd = 2.;
    joint[LKN].Kd = 4.;
    joint[LAP].Kd = 2.;
    joint[LAR].Kd = 2.;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;

    joint[WST].Kd = 2.;
}

