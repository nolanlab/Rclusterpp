#ifndef RCLUSTERPP_HCLUST_H
#define RCLUSTERPP_HCLUST_H

namespace Rclusterpp {

	template<class ForwardIterator>
	void clusterpp2hclust(ForwardIterator first, ForwardIterator last, Rcpp::IntegerMatrix& merge, Rcpp::NumericVector& height);

} // end of Rclusterpp namespace

#endif
