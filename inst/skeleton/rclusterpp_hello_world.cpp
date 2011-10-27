#include "rclusterpp_hello_world.h"

RcppExport SEXP rclusterpp_hello_world(){
BEGIN_RCPP

	using namespace Rcpp;
	using namespace Rclusterpp;

	// Performing Ward's method clustering on stored data representation. We 
	// use clusters that maintain a "center"...
	typedef NumericCluster::center cluster_type;

	Eigen::NumericMatrix data = Eigen::NumericMatrix::Random(50,4);
	
	ClusterVector<cluster_type> clusters(data.rows());	
	init_clusters_from_rows(data, clusters);
	
	cluster_via_rnn(wards_linkage<cluster_type>(), clusters);
			
	return wrap(clusters);

END_RCPP
}
