#ifndef __ALPHA_FILTER_HPP__
#define __ALPHA_FILTER_HPP__

#include <iostream>
#include <inttypes.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Core>

namespace EstimateTools {

class AlphaFilter{
  public:
    AlphaFilter(double alpha_= 0.0);

    ~AlphaFilter(){
    }

    void processSample(Eigen::VectorXd& x, Eigen::VectorXd &x_filtered);

  private:
    double alpha_;

    bool init_;
    bool verbose_;

    Eigen::VectorXd x_filtered_prev_;
};

}

#endif
