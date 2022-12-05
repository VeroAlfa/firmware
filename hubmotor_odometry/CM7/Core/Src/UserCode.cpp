/*
 * UserCode.cpp
 *
 *  Created on: Sep 29, 2022
 *      Author: natta
 */

#include <Eigen/Dense>
using namespace Eigen;

/* Eigen matrix declaration for Kalman filter */
Matrix<float,1,1> Q_L;		// process noise covariance matrix [left wheel]
Matrix<float,1,1> Q_R;		// process noise covariance matrix [right wheel]

//Matrix<float,1,1> R;		// measurement noise covariance matrix
Matrix<float,2,2> R_L;		// measurement noise covariance matrix [left wheel]
Matrix<float,2,2> R_R;		// measurement noise covariance matrix [right wheel]

Vector3f X_L;				// estimated state (first with X0) [left wheel]
Vector3f X_R;				// estimated state (first with X0) [right wheel]

Matrix3f P_L;				// estimated error covariance (first with P0) [left wheel]
Matrix3f P_R;				// estimated error covariance (first with P0) [right wheel]

Matrix3f A;					// state transition matrix [constant]
Vector3f B;					// control input matrix [constant]
Vector3f G;					// additional matrix [constant]
Matrix<float,1,1> U;		// kalman known input [constant]

//Matrix<float,1,1> Y;		// sensor value
//Matrix<float,1,3> C;		// observation matrix
Matrix<float,2,1> Y_L;		// sensor value (first with Y0) [left wheel]
Matrix<float,2,1> Y_R;		// sensor value (first with Y0) [right wheel]
Matrix<float,2,3> C;		// observation matrix [constant]

//Vector3f K;					// kalman gain
Matrix<float,3,2> K_L;		// kalman gain [left wheel]
Matrix<float,3,2> K_R;		// kalman gain [right wheel]

Matrix<float,2,2> speed_cov;
Matrix<float,2,2> forwardmatrix;
Matrix<float,2,2> twist_cov;

float Ts = 0.010; 			//Time step 500Hz

extern "C" void UserCodeSetup();
extern "C" void UserCodeLoop();
extern "C" float UserCodeUpdateRightWheel(float position, float velocity);
extern "C" float UserCodeUpdateLeftWheel(float position, float velocity);
extern "C" float UserGetRWSpeedVariance();
extern "C" float UserGetLWSpeedVariance();
extern "C" float UserGetLinVelVariance();
extern "C" float UserGetAngVelVariance();

void UserCodeSetup()
{
//	Q << 1000000;	// tunable past0.1
//	R << 0.00000001;	// tunable +0.01
//	C << 0,1,0;
	// Constant matrix declaration
	A << 1, Ts, Ts*Ts*0.5,
			0, 1, Ts,
			0, 0, 1;
	B << 0,
			0,
			0;
	G << (Ts*Ts*Ts)/6,
			(Ts*Ts)/2,
			Ts;
	U << 0;
	C << 1,0,0,
			0,1,0;

	// Left wheel matrix declaration
	Q_L << 100;						// tunable
	R_L << 100000000, 0,
			0, 0.13411045074;		// tunable
	// Kalman filter (theory [process model, Q] & read real [sensor model, R])
	X_L << 0,
			0,
			0;
	P_L << 0.0001,0,0,				// tunable
			0,0.0001,0,
			0,0,0.0001;

	// 0.00000001
	// Right wheel matrix declaration
	Q_R << 100;						// tunable
	R_R << 100000000, 0,
			0, 0.13411045074;		// tunable
	// 0.00770884
	// 0.53644180297
	// Kalman filter (theory [process model, Q] & read real [sensor model, R])
	X_R << 0,
			0,
			0;
	P_R << 0.0001,0,0,				// tunable
			0,0.0001,0,
			0,0,0.0001;

	// Twist covariance calculate
	speed_cov << 0,0,
				 0,0;
	forwardmatrix << 0.085*0.5, 0.085*0.5,						// r/2	r/2
					0.085/(2*0.39377), -0.085/(2*0.39377);		// r/2b -r/2b
}

void UserCodeLoop()
{
//	while(1);
}

// Kalman filter function
// X, P, Y, K
void Predict_RightWheel()
{
    X_R = A*X_R + B*U;
    P_R = A*P_R*(A.transpose()) + G*Q_R*(G.transpose());
}
void Predict_LeftWheel()
{
    X_L = A*X_L + B*U;
    P_L = A*P_L*(A.transpose()) + G*Q_L*(G.transpose());
}
float UserCodeUpdateRightWheel(float position, float velocity)
{
	Predict_RightWheel();
	Y_R(0,0) = position;	// assign value of encoder
	Y_R(1,0) = velocity;	// assign value of encoder
	K_R = P_R*(C.transpose()) * (C*P_R*(C.transpose()) + R_R).inverse();
	X_R = X_R + K_R*(Y_R - C*X_R);	// get new X(estimated state)
	P_R = (MatrixXf::Identity(3,3) - K_R*C) * P_R;
	return X_R(1,0);	// get estimated velocity
}
float UserCodeUpdateLeftWheel(float position, float velocity)
{
	Predict_LeftWheel();
	Y_L(0,0) = position;	// assign value of encoder
	Y_L(1,0) = velocity;	// assign value of encoder
	K_L = P_L*(C.transpose()) * (C*P_L*(C.transpose()) + R_L).inverse();
	X_L = X_L + K_L*(Y_L - C*X_L);	// get new X(estimated state)
	P_L = (MatrixXf::Identity(3,3) - K_L*C) * P_L;
	return X_L(1,0);	// get estimated velocity
}
float UserGetRWSpeedVariance()
{
	return P_R(1,1);
}
float UserGetLWSpeedVariance()
{
	return P_L(1,1);
}
float UserGetLinVelVariance()
{
	speed_cov(0,0) = P_R(1,1) * ((M_PI*M_PI)/(180*180)); 	// Var(DegSec) --> Var(radSec)
	speed_cov(1,1) = P_L(1,1) * ((M_PI*M_PI)/(180*180));	// Var(DegSec) --> Var(radSec)
	twist_cov = forwardmatrix*speed_cov*(forwardmatrix.transpose());
	return twist_cov(0,0);
}
float UserGetAngVelVariance()
{
	speed_cov(0,0) = P_R(1,1) * ((M_PI*M_PI)/(180*180));	// Var(DegSec) --> Var(radSec)
	speed_cov(1,1) = P_L(1,1) * ((M_PI*M_PI)/(180*180));	// Var(DegSec) --> Var(radSec)
	twist_cov = forwardmatrix*speed_cov*(forwardmatrix.transpose());
	return twist_cov(1,1);
}
