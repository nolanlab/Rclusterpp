#ifndef RCLUSTERPP_HCLUST_H
#define RCLUSTERPP_HCLUST_H

namespace Rclusterpp {

	typedef ClusterTypes<Rcpp::NumericMatrix::r_type::value> NumericCluster;	

	// Initialization and destruction
	template<class Matrix, class Clusters>
	void init_clusters_from_rows(const Matrix* matrix, Clusters& clusters);

	template<class Matrix, class Clusters>
	void init_clusters_from_columns(const Matrix* matrix, Clusters& clusters);

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

namespace Rcpp {

	template <> SEXP wrap( const Rclusterpp::Hclust& hclust ) {
		return List::create( _["merge"] = hclust.merge, _["height"] = hclust.height, _["order"] = hclust.order ); 
	}

	template <typename T> SEXP wrap( const Rclusterpp::ClusterVector<T>& clusters ) {
		Rclusterpp::Hclust hclust(clusters.initial_clusters());
		Rclusterpp::populate_Rhclust(clusters, hclust);
		return Rcpp::wrap(hclust);
	}
}

#endif
