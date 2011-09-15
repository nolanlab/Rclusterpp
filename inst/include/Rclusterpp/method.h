#ifndef RCLUSTERP_METHOD_H
#define RCLUSTERP_METHOD_H

#include <cassert>

#include <Rclusterpp/cluster.h>

namespace Rclusterpp {

	template<class Cluster, class Distancer, class Merger> 
	struct Method {
		typedef Cluster   cluster_type;
		typedef Distancer distancer_type;
		typedef Merger    merger_type;

		Distancer distancer;
		Merger    merger;

		Method(Distancer distancer=Distancer(), Merger merger=Merger()) : 
			distancer(distancer), merger(merger) {}
	};

	namespace Methods {
	
		template<class Cluster>
		struct WardsDistance {
			typedef typename Cluster::distance_type result_type;
			
			result_type operator()(const Cluster& c1, const Cluster& c2) {
				result_type dist = 0.;
				
				typename Cluster::const_center_iterator cen1 = c1.center_begin(), cen2 = c2.center_begin();
				for (size_t i=0; i<c1.dim(); i++) {
					dist += (cen1[i] - cen2[i]) * (cen1[i] - cen2[i]);
				}
				return dist * (c1.size() * c2.size()) / (c1.size() + c2.size());
			}
		};

		template<class Cluster>
		struct WardsMerge {
			void operator()(Cluster& co, const Cluster& c1, const Cluster& c2) {
				typename Cluster::const_center_iterator cen1 = c1.center_begin(), cen2 = c2.center_begin();
				typename Cluster::center_iterator       cenO = co.center_begin();
				for (size_t i=0; i<co.dim(); i++) {
					cenO[i] = (c1.size() * cen1[i] + c2.size() * cen2[i]) /  (co.size());
				}
			}
		};

	} // end of Methods namespace

	template<class Cluster>
	Method<Cluster, Methods::WardsDistance<Cluster>, Methods::WardsMerge<Cluster> > make_wards_method() {
		return Method<Cluster, Methods::WardsDistance<Cluster>, Methods::WardsMerge<Cluster> >();
	}

} // end of Rclusterpp namespace

#endif
