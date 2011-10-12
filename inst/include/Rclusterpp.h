#ifndef RCLUSTERPP_H
#define RCLUSTERPP_H

#include <RcppCommon.h>

namespace Rclusterpp {

	enum LinkageKinds {
		WARD,
		AVERAGE
	};

	enum DistanceKinds {
		EUCLIDEAN
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

#include <RcppEigenForward.h>
#include <Rcpp.h>
#include <RcppEigenWrap.h>
#include <EigenSugar.h>

#include <Rclusterpp/cluster.h>
#include <Rclusterpp/algorithm.h>
#include <Rclusterpp/method.h>
#include <Rclusterpp/hclust.h>

#endif
