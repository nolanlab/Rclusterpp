#ifndef RCLUSTERP_METHOD_H
#define RCLUSTERP_METHOD_H

#include <iostream>

namespace Rclusterpp {

	
	template<class Cluster, class Distance=typename Cluster::distance_type>
	struct DistanceFunctor {
		typedef Cluster  first_argument_type;
		typedef Cluster  second_argument_type;
		typedef Distance third_argument_type;
		typedef Distance result_type;
	};

	template<class Cluster>
	struct MergeFunctor {
		typedef Cluster first_argument_type;
		typedef Cluster second_argument_type;
		typedef Cluster third_argument_type;
		typedef void		result_type;
	};


	namespace Methods {

		// Distance
		// ----------------------------------------

		template<class V>
		typename V::RealScalar euclidean_distance(const V& a, const V& b) {
			using namespace Eigen;
			return norm( a - b ); 
		}

		template<class V>
		typename V::RealScalar manhattan_distance(const V& a, const V& b) {
			using namespace Eigen;
			return sum( Eigen::abs( a - b ) );  // Note abs is ambiguous to use explicit namespace
		}

		template<class V>
		typename V::RealScalar maximum_distance(const V& a, const V& b) {
			using namespace Eigen;
			return maxCoeff( Eigen::abs( a - b ) );  // Note abs is ambiguous to use explicit namespace
		}

		namespace {
			// TODO: Encapsulate this with function binding instead of using a global variable. Will
			// require more sophisticated typing for distance functions.
			double minkowski_power_g = 1.0;
		}

		template<class V>
		typename V::RealScalar minkowski_distance(const V& a, const V& b) {
			using namespace Eigen;
			return lpNorm( a - b, minkowski_power_g );
		}


		// Distance Adaptors
		
		template<class Matrix, class Distance>
		class DistanceFromStoredDataRows : public std::binary_function<size_t, size_t, typename Distance::result_type> {
			public:
				typedef typename DistanceFromStoredDataRows::result_type result_type;

				DistanceFromStoredDataRows(const Matrix& data, Distance distance) : data_(data), distance_(distance) {}	

				result_type operator()(size_t i1, size_t i2) const {
					return distance_(data_.row(i1), data_.row(i2));
				}

			private:	
				DistanceFromStoredDataRows() {}

				const Matrix& data_;
				Distance distance_;
		};

		// Link Adaptors
		
		template<class Cluster, class Distance>
		class AverageLink : public DistanceFunctor<Cluster, typename Distance::result_type> {
			public:
				typedef typename AverageLink::result_type result_type;
			
				AverageLink(Distance d) : d_(d) {}
			
				result_type operator()(const Cluster& c1, const Cluster& c2, result_type m=std::numeric_limits<result_type>::max()) const {
					if (m < std::numeric_limits<result_type>::max()) {
						m *= (c1.size() * c2.size());  // Adjust threshold to account for averaging denominator
					}
					
					result_type result = 0.;
					typedef typename Cluster::idx_const_iterator iter;
					for (iter i=c1.idxs_begin(), ie=c1.idxs_end(); i!=ie; ++i) {
						for (iter j=c2.idxs_begin(), je=c2.idxs_end(); j!=je; ++j) {
							result += d_(*i, *j);
							if (result > m) {
								return std::numeric_limits<result_type>::max();  // Return early if exceed threshold
							}
						}
					}
					return result / (c1.size() * c2.size());
				}

			private:
				Distance d_;
		};

		template<class Cluster, class Distance>
		class CompleteLink : public DistanceFunctor<Cluster, typename Distance::result_type> {
			public:
				typedef typename CompleteLink::result_type result_type;
			
				CompleteLink(Distance d) : d_(d) {}
			
				result_type operator()(const Cluster& c1, const Cluster& c2, result_type m=std::numeric_limits<result_type>::max()) const {
					result_type result = std::numeric_limits<result_type>::min();
					typedef typename Cluster::idx_const_iterator iter;
					for (iter i=c1.idxs_begin(), ie=c1.idxs_end(); i!=ie; ++i) {
						for (iter j=c2.idxs_begin(), je=c2.idxs_end(); j!=je; ++j) {
							result = std::max(result, d_(*i, *j));
							if (result > m) {
								return std::numeric_limits<result_type>::max();  // Return early if exceed threshold
							}
						}
					}
					return result;
				}

			private:
				Distance d_;
		};


		template<class Cluster>
		struct WardsLink : DistanceFunctor<Cluster> {
			typedef typename Cluster::distance_type result_type;		
			result_type operator()(const Cluster& c1, const Cluster& c2, result_type d=0.) const {
				using namespace Eigen;
				return squaredNorm( c1.center() - c2.center() ) * (c1.size() * c2.size()) / (c1.size() + c2.size()); 
			}
		};

		// Distance matrix
		
		template<class Cluster, class Matrix, class Distance=typename Matrix::Scalar>
		class StoredDistance : public DistanceFunctor<Cluster, Distance> {
			public:
			
				typedef typename StoredDistance::result_type result_type;
			
				StoredDistance(const Matrix& m) : distance(m) {}
			
				// TODO: Note current assuming strictly lower matrix, attempt to use template
				// specialization to automatically select right approach
				result_type operator()(const Cluster& c1, const Cluster& c2, result_type d=0.) const {	
					return distance.coeff(std::max(c1.idx(), c2.idx()), std::min(c1.idx(), c2.idx()));
				}

			private:
				const Matrix& distance;
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
			void operator()(Cluster& co, const Cluster& c1, const Cluster& c2) const {
				co.set_center( ((c1.center() * c1.size()) + (c2.center() * c2.size())) / co.size() );
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

		LinkageMethod(Distancer distancer=Distancer(), Merger merger=Merger()) : distancer(distancer), merger(merger) {}
	};

#define CONST_ROW Matrix::ConstRowXpr

	template<class Matrix>
	Methods::DistanceFromStoredDataRows<
		Matrix, std::pointer_to_binary_function<typename CONST_ROW&, typename CONST_ROW&, typename Matrix::RealScalar> 
	> 
	stored_data_rows(const Matrix& m, DistanceKinds dk, double minkowski=1.0) {
		typedef Methods::DistanceFromStoredDataRows<
			Matrix, std::pointer_to_binary_function<typename CONST_ROW&, typename CONST_ROW&, typename Matrix::RealScalar> 
		> distancer_type;

		switch (dk) {
			default:
				throw std::invalid_argument("Linkage or distance method not yet supported");
			case Rclusterpp::EUCLIDEAN:
				return distancer_type(m, std::ptr_fun(&Methods::euclidean_distance<typename CONST_ROW>));
			case Rclusterpp::MANHATTAN:
				return distancer_type(m, std::ptr_fun(&Methods::manhattan_distance<typename CONST_ROW>));
			case Rclusterpp::MAXIMUM:
				return distancer_type(m, std::ptr_fun(&Methods::maximum_distance<typename CONST_ROW>));
			case Rclusterpp::MINKOWSKI:
				Methods::minkowski_power_g = minkowski;
				return distancer_type(m, std::ptr_fun(&Methods::minkowski_distance<typename CONST_ROW>));

		}
	}

#undef CONST_ROW
	
	template<class Cluster>
	LinkageMethod<Cluster, Methods::WardsLink<Cluster>, Methods::WardsMerge<Cluster> > wards_linkage() {
		return LinkageMethod<Cluster, Methods::WardsLink<Cluster>, Methods::WardsMerge<Cluster> >();
	}

	template<class Cluster, class Distance>
	LinkageMethod<Cluster, Methods::AverageLink<Cluster, Distance>, Methods::NoOpMerge<Cluster> > average_linkage(Distance d) {
		return LinkageMethod<Cluster, Methods::AverageLink<Cluster, Distance>, Methods::NoOpMerge<Cluster> >(
			Methods::AverageLink<Cluster, Distance>(d)
		); 
	}

	template<class Cluster, class Distance>
	LinkageMethod<Cluster, Methods::CompleteLink<Cluster, Distance>, Methods::NoOpMerge<Cluster> > complete_linkage(Distance d) {
		return LinkageMethod<Cluster, Methods::CompleteLink<Cluster, Distance>, Methods::NoOpMerge<Cluster> >(
			Methods::CompleteLink<Cluster, Distance>(d)
		); 
	}

	template<class Cluster, class Matrix>
	LinkageMethod<Cluster, Methods::StoredDistance<Cluster, Matrix>, Methods::NoOpMerge<Cluster> > lancewilliams(Matrix& m, LinkageKinds lk) {
		switch (lk) {
			default:
				throw std::invalid_argument("Linkage method not yet supported");
			case Rclusterpp::AVERAGE: {
				throw std::invalid_argument("Linkage method not yet supported");
				return LinkageMethod<Cluster, Methods::StoredDistance<Cluster, Matrix>, Methods::NoOpMerge<Cluster> >(
					Methods::StoredDistance<Cluster, Matrix>(m)
				);
			}
		}
		
	}


} // end of Rclusterpp namespace

#endif
