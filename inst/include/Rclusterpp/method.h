#ifndef RCLUSTERP_METHOD_H
#define RCLUSTERP_METHOD_H

#include <Rclusterpp/cluster.h>

namespace Rclusterpp {

	template<class Cluster, class Distancer, class Merger> 
	struct Method {
		typedef Cluster   cluster_type;
		
		typedef Distancer distancer_type;
		typedef Merger    merger_type;

		typedef typename Distancer::result_type distance_type;

		Distancer distancer;
		Merger    merger;

		Method(Distancer distancer=Distancer(), Merger merger=Merger()) : 
			distancer(distancer), merger(merger) {}
	};

	template<class Cluster>
	struct DistanceFunction {
		typedef Cluster                         first_argument_type;
		typedef Cluster                         second_argument_type;
		typedef typename Cluster::distance_type result_type;
	};


	namespace Methods {
	
		template<class Cluster>
		struct WardsDistance : DistanceFunction<Cluster> {
			typedef typename WardsDistance::result_type result_type;
			result_type operator()(const Cluster& c1, const Cluster& c2) const {
				using namespace Rcpp;
				result_type dist = sum( pow(c1.center() - c2.center(), 2) );	
				return dist * (c1.size() * c2.size()) / (c1.size() + c2.size());
			}
		};

		template<class Cluster>
		struct WardsMerge {
			void operator()(Cluster& co, const Cluster& c1, const Cluster& c2) {
				Rcpp::NumericVector merged_center = 
					((c1.size() * c1.center()) + (c2.size() * c2.center())) / co.size();
				co.set_center(merged_center);
			}
		};

	} // end of Methods namespace

	template<class Cluster>
	Method<Cluster, Methods::WardsDistance<Cluster>, Methods::WardsMerge<Cluster> > make_wards_method() {
		return Method<Cluster, Methods::WardsDistance<Cluster>, Methods::WardsMerge<Cluster> >();
	}

} // end of Rclusterpp namespace

#endif
