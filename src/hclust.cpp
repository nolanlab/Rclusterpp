#include <algorithm>
#include <functional>
#include <stack>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <Rclusterpp.h>

namespace Rcpp {

	template <> Eigen::RowMajorNumericMatrix as(SEXP x) {
		return Eigen::RowMajorNumericMatrix(as<Eigen::MapNumericMatrix>(x));
	}

	template <> Rclusterpp::LinkageKinds as(SEXP x){
		switch (as<int>(x)) {
			default: throw not_compatible("Linkage method invalid or not yet supported"); 
			case 1: return Rclusterpp::WARD;
			case 2: return Rclusterpp::AVERAGE;
			case 3: return Rclusterpp::SINGLE;
			case 4: return Rclusterpp::COMPLETE;
		}
	}
	
	template <> Rclusterpp::DistanceKinds as(SEXP x){
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

} // Rcpp

RcppExport SEXP linkage_kinds() {
BEGIN_RCPP
	// This ordering matches the 'case' statement above in the 'as' function 
	Rcpp::CharacterVector lk(4);
	lk[0] = "ward";
	lk[1] = "average";
	lk[2] = "single";
	lk[3] = "complete";
	return Rcpp::wrap(lk);
END_RCPP
}

RcppExport SEXP distance_kinds() {
BEGIN_RCPP
	// This ordering matches the 'case' statement above in the 'as' function 
	Rcpp::CharacterVector lk(4);
	lk[0] = "euclidean";
	lk[1] = "manhattan";
	lk[2] = "maximum";
	lk[3] = "minkowski";
	return Rcpp::wrap(lk);
END_RCPP
}

RcppExport SEXP rclusterpp_get_num_procs() {
BEGIN_RCPP
#ifdef _OPENMP
  return Rcpp::wrap(omp_get_num_procs());
#else
  return Rcpp::wrap(1L);
#endif
END_RCPP
}

RcppExport SEXP rclusterpp_set_num_threads(SEXP threads) {
BEGIN_RCPP
#ifdef _OPENMP
	omp_set_num_threads(Rcpp::as<int>(threads));
  return Rcpp::wrap(omp_get_max_threads());
#else
  return Rcpp::wrap(1L);
#endif
END_RCPP
}


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
	
			cluster_via_rnn( wards_link<cluster_type>(), clusters );
			
			return wrap(clusters);	
		}
		case Rclusterpp::AVERAGE: {
			typedef NumericCluster::obs cluster_type;

			ClusterVector<cluster_type> clusters(data_e.rows());
			init_clusters_from_rows(data_e, clusters);

			cluster_via_rnn( average_link<cluster_type>( stored_data_rows(data_e, dk, as<double>(minkowski)) ), clusters );

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

			cluster_via_rnn( complete_link<cluster_type>( stored_data_rows(data_e, dk, as<double>(minkowski)) ), clusters );

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
		
		//typedef ::Rcpp::traits::storage_type<RTYPE>::type STORAGE;
		//double *d_start = ::Rcpp::internal::r_vector_start<RTYPE,STORAGE>(data);
		double *d_start = REAL(data);

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

	int N = as<int>(size);	
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
