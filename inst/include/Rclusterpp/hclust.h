#ifndef RCLUSTERPP_HCLUST_H
#define RCLUSTERPP_HCLUST_H

namespace Rclusterpp {

	typedef Traits::Cluster<Rcpp::NumericMatrix::r_type::value> NumericCluster;	

	// Initialization and destruction

	void init_clusters_from_rows(Rcpp::NumericMatrix& matrix, NumericCluster::center_vector& clusters);

	template<class Clusters>
	void destroy_clusters(Clusters& clusters);

	// Translate clustering results to format expected by R...

	template<class ClusterIterator>
	void populate_Rhclust(
		ClusterIterator first, 
		ClusterIterator last, 
		Rcpp::IntegerMatrix& merge, 
		Rcpp::NumericVector& height,
		Rcpp::IntegerVector& order
	);

} // end of Rclusterpp namespace

#endif
