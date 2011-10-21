#ifndef RCLUSTERPP_HCLUST_H
#define RCLUSTERPP_HCLUST_H

namespace Rclusterpp {

	typedef ClusterTypes<Rcpp::NumericMatrix::stored_type> NumericCluster;	

	// Initialization and destruction

	template<class Matrix, class Clusters>
	Clusters& init_clusters(const Matrix& matrix, Clusters& clusters) {
		for (ssize_t i=0; i<matrix.rows(); i++) {
			clusters[i] = Clusters::make_cluster(-(i+1), i);
		}
		return clusters;
	}

	template<class Matrix, class Clusters>
	Clusters& init_clusters_from_rows(const Matrix& matrix, Clusters& clusters) {
		for (ssize_t i=0; i<matrix.rows(); i++) {
			clusters[i] = Clusters::make_cluster(-(i+1), i, matrix.row(i));
		}
		return clusters;
	}

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
	void populate_Rhclust(const Clusters& clusters, Hclust& hclust) {
	
		if (clusters.size() != (2 * hclust.agglomerations() + 1)) {
			std::invalid_argument("Rclusterpp clusters and hclust object inconsistently sized");
		}

		typedef typename Clusters::cluster_type cluster_type;
		typename Clusters::const_iterator last  = clusters.end();
		typename Clusters::const_iterator first = last - hclust.agglomerations();

		std::transform(first, last, hclust.merge.column(0).begin(), std::mem_fun(&cluster_type::parent1Id));
		std::transform(first, last, hclust.merge.column(1).begin(), std::mem_fun(&cluster_type::parent2Id));
		std::transform(first, last, hclust.height.begin(), std::mem_fun(&cluster_type::disimilarity));

		// Compute order that minimizes dendrogram crossings by performing DFS traversal of tree
		if (first != last) {
			size_t idx = 0;
			
			std::stack<cluster_type const*> stack;
			stack.push(*(last-1)); // Start with last cluster (that has no children)...

			while (!stack.empty()) {
				cluster_type const* top = stack.top();
				stack.pop();
				if (top->initial()) {
					(hclust.order)[idx++] = -(top->id());  // Recall we use negative IDs for inital clusters 
				} else {
					stack.push(top->parent2());
					stack.push(top->parent1());
				}
			}
		}
	}

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
} // Rcpp namespace

#endif
