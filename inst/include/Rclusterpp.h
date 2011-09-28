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

	template <> Rclusterpp::LinkageKinds as(SEXP x) throw(not_compatible) {
		switch (as<int>(x)) {
			default: throw not_compatible("Linkage method invalid or not yet supported"); 
			case 1: return Rclusterpp::WARD;
			case 2: return Rclusterpp::AVERAGE;
		}
	}
	
	template <> Rclusterpp::DistanceKinds as(SEXP x) throw(not_compatible) {
		switch (as<int>(x)) {
			default: throw not_compatible("Distance method invalid or not yet supported"); 
			case 1: return Rclusterpp::EUCLIDEAN;
		}
	}

	template <> SEXP wrap( const Rclusterpp::Hclust& );

	template <typename T> SEXP wrap( const Rclusterpp::ClusterVector<T>& ) ;
}

#include <Rcpp.h>

#include <Rclusterpp/cluster.h>
#include <Rclusterpp/algorithm.h>
#include <Rclusterpp/method.h>
#include <Rclusterpp/hclust.h>

#endif
