#ifndef RCLUSTERPP_HCLUST_H
#define RCLUSTERPP_HCLUST_H

namespace Rclusterpp {

	typedef ClusterTypes<Rcpp::NumericMatrix::stored_type> NumericCluster;	

	// Initialization and destruction

	template<class Matrix, class Clusters>
	void init_clusters_from_rows(const Matrix& matrix, Clusters& clusters);

	// Translate clustering results to format expected by R...
	
	class Hclust {
		public:
		
			Rcpp::IntegerMatrix merge;
			Rcpp::NumericVector height;
			Rcpp::IntegerVector order;
	
			Hclust(size_t num_obs) : merge(num_obs-1, 2), height(num_obs-1), order(num_obs) {}

			size_t agglomerations() const { return merge.nrow(); }

		private:
			Hclust();
			explicit Hclust(const Hclust&);
	};
		
	template<class Clusters>
	void populate_Rhclust(const Clusters& clusters, Hclust& hclust);

} // end of Rclusterpp namespace

#endif
