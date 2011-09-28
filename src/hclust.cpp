#include <algorithm>
#include <functional>
#include <stack>
#include <stdexcept>
#include <cassert>

#include <Rclusterpp.h>

namespace Rclusterpp {
	

	template<class Matrix, class Clusters>
	void init_clusters_from_rows(const Matrix* matrix, Clusters& clusters) {
		typedef typename Clusters::cluster_type cluster_type;
		//clusters.resize(matrix->nrow());
		for (ssize_t i=0; i<matrix->nrow(); i++) {
			clusters.push_back( new cluster_type(-(i+1), i, ((Matrix*)matrix)->row(i)) );
		}
	}

	
	
	template<class Clusters>
	void populate_Rhclust(
		const Clusters& clusters,
		Rcpp::IntegerMatrix* merge, 
		Rcpp::NumericVector* height,
		Rcpp::IntegerVector* order
	) {
	
		if (merge->nrow()   != height->size() ||
				order->size()   != merge->nrow() + 1 ||
				clusters.size() != (2*(size_t)merge->nrow() + 1)) {
			std::invalid_argument("Invalid data structures for populate R hclust class");
		}

		typedef typename Clusters::cluster_type cluster_type;
		typename Clusters::const_iterator last  = clusters.end();
		typename Clusters::const_iterator first = last - merge->nrow();

		std::transform(first, last, merge->column(0).begin(), std::mem_fun(&cluster_type::parent1Id));
		std::transform(first, last, merge->column(1).begin(), std::mem_fun(&cluster_type::parent2Id));
		std::transform(first, last, height->begin(), std::mem_fun(&cluster_type::disimilarity));

		// Compute order that minimizes dendrogram crossings by performing DFS traversal of tree
		if (first != last) {
			size_t idx = 0;
			
			std::stack<cluster_type const*> stack;
			stack.push(*(last-1)); // Start with last cluster (that has no children)...

			while (!stack.empty()) {
				cluster_type const* top = stack.top();
				stack.pop();
				if (top->initial()) {
					(*order)[idx++] = -(top->id());  // Recall we use negative IDs for inital clusters 
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
			typedef NumericCluster::center       cluster_type;
			typedef ClusterVector<cluster_type>  clusters_type;

			clusters_type clusters;	
			init_clusters_from_rows(&data_m, clusters);
	
			cluster_via_rnn(wards_linkage<cluster_type>(), clusters);
			populate_Rhclust(clusters, &merge, &height, &order);
			
			break;	
		}
		case Rclusterpp::AVERAGE: {
			typedef NumericCluster::obs         cluster_type;
			typedef ClusterVector<cluster_type> clusters_type;

			clusters_type clusters;
			init_clusters_from_rows(&data_m, clusters);

			cluster_via_rnn(average_linkage_from_rows<cluster_type>(&data_m), clusters);
			populate_Rhclust(clusters, &merge, &height, &order);

			break;
		}
	}
	
  return List::create( _["merge"] = merge, _["height"] = height, _["order"] = order ); 
END_RCPP
}
