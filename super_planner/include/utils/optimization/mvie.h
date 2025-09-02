//
// Created by yunfan on 2022/6/25.
//

#pragma once

#include <data_structure/base/ellipsoid.h>


namespace optimization_utils {

    using namespace geometry_utils;

    class MVIE {
    public:
        MVIE() = default;

        ~MVIE() = default;

        static void chol3d(const Eigen::Matrix3d &A,
                           Eigen::Matrix3d &L);

        static bool smoothedL1(const double &mu,
                               const double &x,
                               double &f,
                               double &df);

        static double costMVIE(void *data,
                               const Eigen::VectorXd &x,
                               Eigen::VectorXd &grad);

        // R is also assumed to be a rotation matrix
        static bool maxVolInsEllipsoid(const Eigen::MatrixX4d &hPoly,
                                       Ellipsoid &ellipsoid);

    };
}

