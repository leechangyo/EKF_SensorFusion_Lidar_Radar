#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.(Radar)
   */
  MatrixXd Hj(3,4);
  // state parameter
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // prepare jacobian term(look at the document if you don't understand)
  double c1 = px*px + py* py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  if(fabs(c1)<0.0001)
  {
     std::cout <<"ERROR - division by zeo" <<std::endl;
     return Hj;
  }

  // compute jacobian
  Hj << px/c2, py/c2, 0, 0,
         -(py/c1), px/c1, 0,0,
         px*(vx*py-vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
   return Hj;
}
