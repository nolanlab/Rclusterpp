#ifndef RCLUSTERP_METHOD_H
#define RCLUSTERP_METHOD_H

#include <iostream>

namespace Rclusterpp {

	
	template<class Cluster>
	struct DistanceFunctor {
		typedef Cluster                         first_argument_type;
		typedef Cluster                         second_argument_type;
		typedef typename Cluster::distance_type result_type;
	};

	template<class Cluster>
	struct MergeFunctor {
		typedef Cluster                         first_argument_type;
		typedef Cluster                         second_argument_type;
		typedef Cluster                         third_argument_type;
		typedef void														result_type;
	};


	namespace Methods {

		// Distance
		// ----------------------------------------

		double euclidean_distance(const Rcpp::NumericVector& a, const Rcpp::NumericVector& b) {
			using namespace Rcpp;
			return (double)sqrt( sum( square( a - b ) ) );
		}

		// Distance Adaptors
		
		template<class Matrix>
		class DistanceFromStoredDataRows : public std::binary_function<size_t, size_t, double> {
			public:
				typedef typename DistanceFromStoredDataRows::result_type result_type;

				DistanceFromStoredDataRows(Matrix* data) : data_(data) {}	

				result_type operator()(size_t i1, size_t i2) {
					return euclidean_distance(((Matrix*)data_)->row(i1), ((Matrix*)data_)->row(i2));
				}

			private:	
				DistanceFromStoredDataRows() {}

				Matrix* data_;
		};

		// Link Adaptors
		
		template<class Cluster, class Distance>
		class AverageLink : public DistanceFunctor<Cluster> {
			public:
				typedef typename AverageLink::result_type result_type;
			
				AverageLink(Distance d) : d_(d) {}
			
				result_type operator()(const Cluster& c1, const Cluster& c2) {
					result_type result = 0.;
					typedef typename Cluster::idx_const_iterator iter;
					for (iter i=c1.idxs_begin(), ie=c1.idxs_end(); i!=ie; ++i) {
						for (iter j=c2.idxs_begin(), je=c2.idxs_end(); j!=je; ++j) {
							result += d_(*i, *j);
						}
					}
					return result / (c1.size() * c2.size());
				}

			private:
				Distance d_;
		};

		template<class Cluster>
		class WardsLink : public DistanceFunctor<Cluster> {
			public:
				typedef typename WardsLink::result_type result_type;
			
				result_type operator()(const Cluster& c1, const Cluster& c2) const {
					using namespace Rcpp;
					result_type dist = sum( square( c1.center() - c2.center() ) );	
					return dist * (c1.size() * c2.size()) / (c1.size() + c2.size());
				}
		};

		// Merge 
		// ----------------------------------------

		template<class Cluster>
		struct NoOpMerge : public MergeFunctor<Cluster> {
			void operator()(Cluster& co, const Cluster& c1, const Cluster& c2) const {
				// Noop
				return;
			}
		};

		template<class Cluster>
		struct WardsMerge : public MergeFunctor<Cluster> {
			void operator()(Cluster& co, const Cluster& c1, const Cluster& c2) const {
				using namespace Rcpp;
				NumericVector merged_center = ((c1.size() * c1.center()) + (c2.size() * c2.center())) / co.size();
				co.set_center(merged_center);
			}
		};

	} // end of Methods namespace


	template<class Cluster, class Distancer, class Merger> 
	struct LinkageMethod {
		typedef Cluster   cluster_type;
		
		typedef Distancer distancer_type;
		typedef Merger    merger_type;

		typedef typename Distancer::result_type distance_type;

		Distancer distancer;
		Merger    merger;

		LinkageMethod(Distancer distancer=Distancer(), Merger merger=Merger()) : 
			distancer(distancer), merger(merger) {}
	};


	template<class Cluster>
	LinkageMethod<Cluster, Methods::WardsLink<Cluster>, Methods::WardsMerge<Cluster> > wards_linkage() {
		return LinkageMethod<Cluster, Methods::WardsLink<Cluster>, Methods::WardsMerge<Cluster> >();
	}

	template<class Cluster, class Matrix>
	LinkageMethod<
		Cluster, 
		Methods::AverageLink<Cluster, Methods::DistanceFromStoredDataRows<Matrix> >, 
		Methods::NoOpMerge<Cluster> 
	> average_linkage_from_rows(Matrix* matrix) {
		return LinkageMethod<
			Cluster, 
			Methods::AverageLink<Cluster, Methods::DistanceFromStoredDataRows<Matrix> >, 
			Methods::NoOpMerge<Cluster> 
		>(Methods::AverageLink<Cluster, Methods::DistanceFromStoredDataRows<Matrix> >(Methods::DistanceFromStoredDataRows<Matrix>(matrix))); 
	}
	
	
} // end of Rclusterpp namespace

#endif
