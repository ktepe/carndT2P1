#include "kalman_filter.h"
#include <iostream>
#include <math.h>
#include "ket.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

//kemal tepe
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
#if ket_debug  
  cout<< "in KF predict" << endl;
  cout<< "x_" << endl<< x_ << endl;
  cout<< "F_" << endl<< F_ << endl;
  cout<< "P_" << endl<< P_ << endl;
  cout<< "Q_" << endl<< Q_ << endl;
#endif

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
	
	
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations  
  */
  // convert state to polar coordinates
 	double rho=sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  double bearing=0;
  double x1_x0=x_(1)/x_(0);
  
  
	if (fabs(x_(0)) > 0.0001) { 
	  bearing=atan2(x_(1), x_(0)) ;
	  } else {
	 cout<< "error in atan2()" <<endl;
	 }
	  
	 
  double rho_dot=0.0;
  
  if (fabs(rho) < 0.0001) {
    rho_dot = 0.0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
	}
#if ket_debug  
  cout << "rho, bearing, rho_dot"<<endl;
  cout << rho << " " << bearing << " "<<rho_dot <<endl;
#endif

  VectorXd h=VectorXd(3);
  h<< rho, bearing, rho_dot;
  
  VectorXd y=z-h;
 	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	
	float y_1=y(1);
	//bearing normalization
	if (y(1) >= My_PI){
			y(1)= fmod(y(1)+My_PI, 2*My_PI)-My_PI;
		 }
	if (y(1) <= (-1.0*My_PI)){
			y(1)=fmod(y(1)-My_PI, 2*My_PI)+My_PI;	
	}
	
	//new estimate
	x_ = x_ + (K * y);
	int x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

#if ket_debug
  if (y_1 != y(1)){
  cout<< "bearing y in ekf before and after and K " << y_1<< " "<< y(1) <<" " << K << endl;
  }
#endif
  
}
