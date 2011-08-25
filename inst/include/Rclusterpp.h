#ifndef RCLUSTERPP_H
#define RCLUSTERPP_H

#include <RcppCommon.h>

namespace Rclusterpp {

	typedef enum {
		WARD
	} LinkageKinds;

	typedef enum {
		EUCLIDEAN
	} DistanceKinds;

}

namespace Rcpp {
	template <> Rclusterpp::LinkageKinds as(SEXP x) throw(not_compatible) {
		switch (as<int>(x)) {
			default: throw not_compatible("Expecting integer index into linkages"); 
			case 1: return Rclusterpp::WARD;
		}
	}
	template <> Rclusterpp::DistanceKinds as(SEXP x) throw(not_compatible) {
		switch (as<int>(x)) {
			default: throw not_compatible("Expecting integer index into distances"); 
			case 1: return Rclusterpp::EUCLIDEAN;
		}
	}
}

#include <Rcpp.h>

#endif
