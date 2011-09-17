#include <algorithm>
#include <functional>
#include <stack>
#include <stdexcept>

#include <Rclusterpp.h>

namespace Rclusterpp {

	void init_clusters_from_rows(Rcpp::NumericMatrix& matrix, NumericCluster::center_vector& clusters) {
		clusters.resize(matrix.nrow());
		for (ssize_t i=0; i<matrix.nrow(); i++) {
			clusters[i] = new NumericCluster::center(-(i+1), matrix.row(i));
		}
	}

	template<class Clusters>
	void destroy_clusters(Clusters& clusters) {
		for (typename Clusters::iterator i=clusters.begin(), e=clusters.end(); i!=e; ++i) {
			delete *i;
		}
		clusters.clear();
	}


	template<class ClusterIterator>
	void populate_Rhclust(
		ClusterIterator first, 
		ClusterIterator last, 
		Rcpp::IntegerMatrix& merge, 
		Rcpp::NumericVector& height,
		Rcpp::IntegerVector& order
	) {
		typedef typename Traits::Collection<ClusterIterator>::cluster_type cluster_type;
		
		std::transform(first, last, merge.column(0).begin(), std::mem_fun(&cluster_type::parent1Id));
		std::transform(first, last, merge.column(1).begin(), std::mem_fun(&cluster_type::parent2Id));
		std::transform(first, last, height.begin(), std::mem_fun(&cluster_type::disimilarity));

		// Compute order that minimizes dendrogram crossings by performing DFS traversal of tree
		if (first != last) {
			size_t idx = 0;
			
			std::stack<cluster_type const*> stack;
			stack.push(*(last-1)); // Start with last cluster (that has no children)...

			while (!stack.empty()) {
				cluster_type const* top = stack.top();
				stack.pop();
				if (top->initial()) {
					order[idx++] = -(top->id());  // Recall we use negative IDs for inital clusters 
				} else {
					stack.push(top->parent2());
					stack.push(top->parent1());
				}
			}
		}
	}

} // end of Rclusterpp namespace

RcppExport SEXP hclust_from_data(SEXP data, SEXP link, SEXP dist) {
BEGIN_RCPP
	using namespace Rcpp;
	using namespace Rclusterpp;

	NumericMatrix data_m(data);
		
	IntegerMatrix merge (data_m.nrow()-1, 2);
	NumericVector height(data_m.nrow()-1);
	IntegerVector order (data_m.nrow());

	LinkageKinds  lk = as<LinkageKinds>(link);
	DistanceKinds dk = as<DistanceKinds>(dist);

	switch (lk) {
		default:
			throw std::invalid_argument("Linkage or distance method not yet supported");
		case Rclusterpp::WARD: {
		
			NumericCluster::center_vector clusters;	
			init_clusters_from_rows(data_m, clusters);
			
			cluster_via_rnn(make_wards_method<NumericCluster::center>(), clusters);
			populate_Rhclust(clusters.begin()+data_m.nrow(), clusters.end(), merge, height, order);  // Recall "merged" clusters are in latter half
			
			destroy_clusters(clusters);
		}
	}
	
  return List::create( _["merge"] = merge, _["height"] = height, _["order"] = order ); 
END_RCPP
}
