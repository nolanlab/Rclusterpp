#include <cassert>
#include <iostream>

#include <Rclusterpp.h>

namespace Rclusterpp {

	template<class ForwardIterator>
	void clusters2hclust(ForwardIterator first, ForwardIterator last, Rcpp::IntegerMatrix& merge, Rcpp::NumericVector& height) {
		std::transform(first, last, merge.column(0).begin(), std::mem_fun(&Traits::Iterator<ForwardIterator>::cluster_type::parent1Id));
		std::transform(first, last, merge.column(1).begin(), std::mem_fun(&Traits::Iterator<ForwardIterator>::cluster_type::parent2Id));
		std::transform(first, last, height.begin(), std::mem_fun(&Traits::Iterator<ForwardIterator>::cluster_type::disimilarity));
	}

} // end of Rclusterpp namespace


RcppExport SEXP hclust_from_data(SEXP data, SEXP link, SEXP dist) {
BEGIN_RCPP
	using namespace Rcpp;
	using namespace Rclusterpp;

	NumericMatrix data_m(data);
		
	IntegerMatrix merge(data_m.nrow()-1, 2);
	NumericVector height(data_m.nrow()-1);

	LinkageKinds  lk = as<LinkageKinds>(link);
	DistanceKinds dk = as<DistanceKinds>(dist);

	switch (lk) {
		default:
			assert(false && "Linkage kind not yet supported");
		case Rclusterpp::WARD: {
		
			NumericCluster::center_vector clusters(data_m.nrow());
			
			rows2clusters(clusters, data_m);
			hclust_rnn_chain(make_wards_method<NumericCluster::center>(), clusters);
			clusters2hclust(clusters.begin()+data_m.nrow(), clusters.end(), merge, height);
			destroy_clusters(clusters.begin(), clusters.end());
		}
	}
	
  return List::create( _["merge"] = merge, _["height"] = height ); 
END_RCPP
}
