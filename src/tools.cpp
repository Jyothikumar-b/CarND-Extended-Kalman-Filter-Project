#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  	VectorXd rmse(4);
	rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if(estimations.size()==0){
	    cout<<"Estimation Size = 0";
	    return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size()!=ground_truth.size()){
	    cout<<"size mismatch";
	    return rmse;
	}
	// ... your code here

	//accumulate squared residuals
	VectorXd residual(4);
	for(int i=0; i < estimations.size(); ++i){
        residual<<(estimations[i]-ground_truth[i]).array().pow(2);
        rmse=rmse+residual;
	}

	//calculate the mean
    int	n=estimations.size();
	rmse<<rmse*(1.0/n);

	//calculate the squared root
	rmse<<rmse.array().pow(0.5);

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  Hj.fill(0.0);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
	if(fabs(px*px+py*py)<0.0001){
		cout<<"Calculate Jacobian Matrix - divison by zero"<<endl;
		return Hj;
	}
	//compute the Jacobian matrix
	Hj(0,0)=(px/pow(px*px+py*py,0.5));
	Hj(0,1)=(py/pow(px*px+py*py,0.5));
	Hj(0,2)=0;
	Hj(0,3)=0;
	
	Hj(1,0)=(-py/(px*px+py*py));
	Hj(1,1)=(px/(px*px+py*py));
	Hj(1,2)=0;
	Hj(1,3)=0;
	
	Hj(2,0)=py*(vx*py-vy*px)/pow(px*px+py*py,3/2.0);
	Hj(2,1)=px*(vy*px-vx*py)/pow(px*px+py*py,3/2.0);
	Hj(2,2)=px/pow(px*px+py*py,0.5);
	Hj(2,3)=py/pow(px*px+py*py,0.5);

	return Hj;
}
