#ifndef RCLUSTERPP_H
#define RCLUSTERPP_H

#include <RcppCommon.h>

namespace Rclusterpp {

	enum LinkageKinds {
		WARD,
		AVERAGE
	};

	enum DistanceKinds {
		EUCLIDEAN,
		MANHATTAN
	};

	class Hclust;

	template<class T>
	class ClusterVector;
}


namespace Rcpp {
	
	/*
	 * These functions map indices provided by R-bindings to linkage and distance
	 * methods. The relationship between the index and the method needs to be kept'
	 * in sync with the R-bindings.
	 */

	template <> Rclusterpp::LinkageKinds as(SEXP x) throw(not_compatible);	
	template <> Rclusterpp::DistanceKinds as(SEXP x) throw(not_compatible);

	template <> SEXP wrap( const Rclusterpp::Hclust& );

	template <typename T> SEXP wrap( const Rclusterpp::ClusterVector<T>& ) ;
}

#define EIGEN_MATRIXBASE_PLUGIN <RclusterppEigenMatrixPlugin.h>
#define EIGEN_ARRAYBASE_PLUGIN <RclusterppEigenArrayPlugin.h>
#include <RcppEigenForward.h>
#include <Rcpp.h>
#include <RcppEigenWrap.h>
#include <RclusterppEigenSugar.h>

#include <Rclusterpp/cluster.h>
#include <Rclusterpp/algorithm.h>
#include <Rclusterpp/method.h>
#include <Rclusterpp/hclust.h>

#endif
