#include <Rclusterpp.h>
#include <iostream>

RcppExport SEXP hclust_from_data(SEXP data, SEXP nr, SEXP nc, SEXP link, SEXP dist) {
BEGIN_RCPP
	using namespace Rcpp;
	using namespace Rclusterpp;

	// Reconstitute matrix (forced to 'double' vector in wrapping function)
	NumericMatrix data_m(as<int>(nr), as<int>(nc), as<NumericVector>(data).begin());
		
	std::cerr << "Matrix size: " << data_m.nrow() << "x" << data_m.ncol() << std::endl;

	LinkageKinds  lk = as<LinkageKinds>(link);
	DistanceKinds dk = as<DistanceKinds>(dist);


  return NumericVector::create(0.0);
END_RCPP
}
