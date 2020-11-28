#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

//// This code is more or less the same as explained on the conferences.
//MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
//  MatrixXd Hj(3,4);
//
//  if ( x_state.size() != 4 ) {
//      std::cout << "ERROR - CalculateJacobian () - The state vector must have size 4." << std::endl;
//    return Hj;
//  }
//    //recover state parameters
//    double px = x_state(0);
//    double py = x_state(1);
//    double vx = x_state(2);
//    double vy = x_state(3);
//
//    //pre-compute a set of terms to avoid repeated calculation
//    double c1 = px*px+py*py;
//    double c2 = sqrt(c1);
//    double c3 = (c1*c2);
//
//    //check division by zero
//    if(fabs(c1) < 0.0001){
//        std::cout << "ERROR - CalculateJacobian () - Division by Zero" << std::endl;
//        return Hj;
//    }
//
//    //compute the Jacobian matrix
//    Hj << (px/c2), (py/c2), 0, 0,
//          -(py/c1), (px/c1), 0, 0,
//          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
//
//    return Hj;
//}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    * TODO: Calculate the RMSE here.
    */

    //RMSE calculation copied from class exercise
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
    }

    // accumulate squared residuals
    for (int i=0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE
    // check division by zero
    if (fabs(px*px + py*py) < 0.0001) { // rather than exact zero check -0.0001 <x <0.0001
        cout << "error (calculating Jaccobian, division by zero!";
    } else
    {
        float px_div = px/sqrt(px*px + py*py);
        float px_div2 = px/(px*px + py*py);
        float py_div = py/sqrt(px*px + py*py);
        float py_div2 = py/(px*px + py*py);
        float px_mul_div =  px*(vy*px - vx*py) / ((px*px + py*py)*sqrt(px*px + py*py) );
        float py_mul_div = py*(vx*py - vy*px) / ((px*px + py*py)*sqrt(px*px + py*py) ) ;

        Hj <<   px_div, py_div, 0, 0,
                -py_div2, px_div2, 0, 0,
                py_mul_div, px_mul_div, px_div, py_div;
    }
    return Hj;
}
