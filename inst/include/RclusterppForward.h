#ifndef RCLUSTERPPFORWARD_H
#define RCLUSTERPPFORWARD_H

namespace Rclusterpp {

	enum LinkageKinds {
		WARD,
		AVERAGE,
		SINGLE,
		COMPLETE
	};

	enum DistanceKinds {
		EUCLIDEAN,
		MANHATTAN,
		MAXIMUM,
		MINKOWSKI
	};


	// Forward declarations of internal data structures that can
	// be converted to R objects via Rcpp wrap functions

	class Hclust;

	template<class T>
	class ClusterVector;
}

namespace Eigen {

	// Convenience types for working with Eigen matrices

	typedef Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> >      NumericMatrix;
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>  RowMajorNumericMatrix;

}

namespace Rcpp {
	
	template <> Eigen::RowMajorNumericMatrix as(SEXP x); 

	// These functions map indices provided by R-bindings to linkage and distance
	// methods. The relationship between the index and the method needs to be kept'
	// in sync with the R-bindings.
	

	template <> Rclusterpp::LinkageKinds as(SEXP x) throw(not_compatible);	
	template <> Rclusterpp::DistanceKinds as(SEXP x) throw(not_compatible);

	template <> SEXP wrap( const Rclusterpp::Hclust& );
	template <typename T> SEXP wrap( const Rclusterpp::ClusterVector<T>& ) ;
}


#endif