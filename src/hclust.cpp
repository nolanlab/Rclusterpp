#include <algorithm>
#include <functional>
#include <stack>
#include <stdexcept>
#include <cassert>

#include <Rclusterpp.h>

namespace Rcpp {

	template <> Eigen::RowMajorNumericMatrix as(SEXP x) {
		return Eigen::RowMajorNumericMatrix(as<Eigen::MapNumericMatrix>(x));
	}

	template <> Rclusterpp::LinkageKinds as(SEXP x) throw(not_compatible) {
		switch (as<int>(x)) {
			default: throw not_compatible("Linkage method invalid or not yet supported"); 
			case 1: return Rclusterpp::WARD;
			case 2: return Rclusterpp::AVERAGE;
			case 3: return Rclusterpp::SINGLE;
			case 4: return Rclusterpp::COMPLETE;
		}
	}
	
	template <> Rclusterpp::DistanceKinds as(SEXP x) throw(not_compatible) {
		switch (as<int>(x)) {
			default: throw not_compatible("Distance method invalid or not yet supported"); 
			case 1: return Rclusterpp::EUCLIDEAN;
			case 2: return Rclusterpp::MANHATTAN;
			case 3: return Rclusterpp::MAXIMUM;
			case 4: return Rclusterpp::MINKOWSKI;
		}
	}


	template <> SEXP wrap( const Rclusterpp::Hclust& hclust ) {
		return List::create( _["merge"] = hclust.merge, _["height"] = hclust.height, _["order"] = hclust.order ); 
	}

	template <typename T> SEXP wrap( const Rclusterpp::ClusterVector<T>& clusters ) {
		Rclusterpp::Hclust hclust(clusters.initial_clusters());
		Rclusterpp::populate_Rhclust(clusters, hclust);
		return Rcpp::wrap(hclust);
	}
	
} // Rcpp

namespace Rclusterpp {

	template<class Matrix, class Clusters>
	void init_clusters(const Matrix& matrix, Clusters& clusters) {
		for (ssize_t i=0; i<matrix.rows(); i++) {
			clusters[i] = Clusters::make_cluster(-(i+1), i);
		}
	}

	template<class Matrix, class Clusters>
	void init_clusters_from_rows(const Matrix& matrix, Clusters& clusters) {
		for (ssize_t i=0; i<matrix.rows(); i++) {
			clusters[i] = Clusters::make_cluster(-(i+1), i, matrix.row(i));
		}
	}
	
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

RcppExport SEXP hclust_from_data(SEXP data, SEXP link, SEXP dist, SEXP minkowski) {
BEGIN_RCPP
	using namespace Rcpp;
	using namespace Rclusterpp;

	Eigen::RowMajorNumericMatrix data_e(as<Eigen::RowMajorNumericMatrix>(data));

	LinkageKinds  lk = as<LinkageKinds>(link);
	DistanceKinds dk = as<DistanceKinds>(dist);

	switch (lk) {
		default:
			throw std::invalid_argument("Linkage or distance method not yet supported");
		case Rclusterpp::WARD: {
			typedef NumericCluster::center cluster_type;

			ClusterVector<cluster_type> clusters(data_e.rows());	
			init_clusters_from_rows(data_e, clusters);
	
			cluster_via_rnn(wards_linkage<cluster_type>(), clusters);
			
			return wrap(clusters);	
		}
		case Rclusterpp::AVERAGE: {
			typedef NumericCluster::obs cluster_type;

			ClusterVector<cluster_type> clusters(data_e.rows());
			init_clusters_from_rows(data_e, clusters);

			cluster_via_rnn( average_linkage<cluster_type>( stored_data_rows(data_e, dk, as<double>(minkowski)) ), clusters );

			return wrap(clusters);
		}
		case Rclusterpp::SINGLE: {
			typedef NumericCluster::plain cluster_type;

			ClusterVector<cluster_type> clusters(data_e.rows());
			init_clusters_from_rows(data_e, clusters);

			cluster_via_slink( stored_data_rows(data_e, dk, as<double>(minkowski)), clusters );

			return wrap(clusters);
		}
		case Rclusterpp::COMPLETE: {
			typedef NumericCluster::obs cluster_type;

			ClusterVector<cluster_type> clusters(data_e.rows());
			init_clusters_from_rows(data_e, clusters);

			cluster_via_rnn( complete_linkage<cluster_type>( stored_data_rows(data_e, dk, as<double>(minkowski)) ), clusters );

			return wrap(clusters);
		}

	}
	 
END_RCPP
}

namespace {

	template<class Matrix>
	void populate_strictly_lower(Matrix& m, SEXP data) {
		ssize_t N = m.rows();

		const int RTYPE = ::Rcpp::traits::r_sexptype_traits<Eigen::NumericMatrix::Scalar>::rtype; 
		if (TYPEOF(data) != RTYPE)
			throw std::invalid_argument("Wrong R type for mapped vector");
		
		typedef ::Rcpp::traits::storage_type<RTYPE>::type STORAGE;
		double *d_start = ::Rcpp::internal::r_vector_start<RTYPE,STORAGE>(data);

		for (ssize_t c=0; c<N-1; c++) {
			m.block(c+1 /* starting row */, c, N-(c+1) /* numer of rows*/, 1) = Eigen::Map<Eigen::NumericMatrix>(d_start,N-(c+1),1);
			d_start += N-(c+1);
		}
	}
	
}

RcppExport SEXP hclust_from_distance(SEXP data, SEXP size, SEXP link) {
BEGIN_RCPP
	using namespace Rcpp;
	using namespace Rclusterpp;

	ssize_t N = as<ssize_t>(size);	
	Eigen::NumericMatrix data_e(N, N);
	
	populate_strictly_lower(data_e, data);  // Populate strictly lower distance matrix from packed vector
	Eigen::StrictlyLowerNumericMatrix data_t = data_e.triangularView<Eigen::StrictlyLower>(); 
			
	typedef NumericCluster::plain cluster_type;

	ClusterVector<cluster_type> clusters(data_t.rows());
	init_clusters(data_t, clusters);

	LinkageKinds  lk = as<LinkageKinds>(link);
	switch (lk) {
	default:
		throw std::invalid_argument("Linkage or distance method not yet supported");
	case Rclusterpp::AVERAGE:
		cluster_via_rnn( average_link<cluster_type>(data_t, FromDistance), clusters );
		break;
	case Rclusterpp::SINGLE:
		cluster_via_rnn( single_link<cluster_type>(data_t, FromDistance),  clusters );
		break;
	case Rclusterpp::COMPLETE:
		cluster_via_rnn( complete_link<cluster_type>(data_t, FromDistance), clusters );
		break;
	}

	return wrap(clusters);
END_RCPP
}
