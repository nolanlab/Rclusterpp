#ifndef RCLUSTERPP_H
#define RCLUSTERPP_H

#include <RcppCommon.h>


#define EIGEN_MATRIXBASE_PLUGIN <RclusterppEigenMatrixPlugin.h>
#define EIGEN_ARRAYBASE_PLUGIN <RclusterppEigenArrayPlugin.h>
#include <RcppEigenForward.h>
#include <RclusterppForward.h>

#include <Rcpp.h>

#include <RcppEigenWrap.h>
#include <RclusterppEigenSugar.h>

#include <Rclusterpp/cluster.h>
#include <Rclusterpp/algorithm.h>
#include <Rclusterpp/method.h>
#include <Rclusterpp/hclust.h>

#endif
