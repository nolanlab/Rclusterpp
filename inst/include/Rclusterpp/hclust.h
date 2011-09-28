#ifndef RCLUSTERPP_HCLUST_H
#define RCLUSTERPP_HCLUST_H

namespace Rclusterpp {

	typedef Traits::Cluster<Rcpp::NumericMatrix::r_type::value> NumericCluster;	

	// Initialization and destruction
	template<class Matrix, class Clusters>
	void init_clusters_from_rows(const Matrix* matrix, Clusters& clusters);

	// Translate clustering results to format expected by R...

	template<class Clusters>
	void populate_Rhclust(const Clusters& clusters, Rcpp::IntegerMatrix* merge, Rcpp::NumericVector* height, Rcpp::IntegerVector* order);

} // end of Rclusterpp namespace

#endif
