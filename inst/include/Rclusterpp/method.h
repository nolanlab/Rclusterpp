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

		inline
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
		struct WardsLink : DistanceFunctor<Cluster> {
			typedef typename Cluster::distance_type result_type;
			typedef typename Cluster::value_type    value_type;

			static result_type sqofdiff(const value_type& l, const value_type& r) { 
				return (l - r) * (l -r); 
			}

			result_type operator()(const Cluster& c1, const Cluster& c2) const {
					result_type dist = std::inner_product(
							c1.center_begin(), c1.center_end(), 
							c2.center_begin(), 
							(result_type)0, // Inital value
							std::plus<result_type>(), sqofdiff); // '+' and '*' operator in inner product					
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
		struct WardsMerge : MergeFunctor<Cluster> {						
		
			struct OP : std::binary_function<typename Cluster::value_type, typename Cluster::value_type, typename Cluster::value_type> {				
				typedef typename Cluster::value_type value_type;
				
				size_t s1, s2, so;
				OP(size_t s1_, size_t s2_, size_t so_) : s1(s1_), s2(s2_), so(so_) {}
				
				value_type operator()(const value_type& c1, const value_type& c2) const { return ((c1 * s1) + (c2 * s2)) / so; }
			};
			
			void operator()(Cluster& co, const Cluster& c1, const Cluster& c2) const {
				std::transform(c1.center_begin(), c1.center_end(), c2.center_begin(), co.center_begin(), OP(c1.size(), c2.size(), co.size()));
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
