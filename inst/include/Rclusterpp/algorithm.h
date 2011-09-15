#ifndef RCLUSTERP_ALGORITHM_H
#define RCLUSTERP_ALGORITHM_H

#include <limits>
#include <utility>
#include <cassert>

#include <Rclusterpp/cluster.h>

namespace Rclusterpp {

	namespace {

		template<typename ClusterP, typename Distance>
		struct ChainEntry {
				ClusterP cluster;
				Distance to_dist;  // Distance to me from predeccesor in chain

				ChainEntry(ClusterP cluster, Distance dist = std::numeric_limits<Distance>::max()) :
					cluster(cluster), to_dist(dist) {} 
		};
		
		template<class ClusterP, class Distancer>
		struct ChainDistancer {
			typedef typename Distancer::result_type result_type;

			Distancer distancer;
			const ClusterP tip;  // Tip of the chain	

			ChainDistancer(Distancer& distancer, const ClusterP tip) : distancer(distancer), tip(tip) {}

			result_type operator()(const ClusterP c) {
				return distancer(*tip, *c);
			}
		};

	} // end of anonymous namespace

	
	template<typename Distancer, class ForwardIterator>
	std::pair<ForwardIterator, typename Distancer::result_type> nearest_neighbor(
		Distancer       distancer, 
		ForwardIterator first, 
		ForwardIterator last, 
		typename Distancer::result_type max_dist=std::numeric_limits<typename Distancer::result_type>::max()
	) {
	
		typedef typename Distancer::result_type Dist_t;

		ForwardIterator min_i = last;
		Dist_t          min_d = max_dist;
		for (ForwardIterator i=first; i!=last; ++i) {
			Dist_t dist = distancer(*i);
			if (dist < min_d) {
				min_i = i;
				min_d = dist;
			}
		}

		return std::make_pair(min_i, min_d);	
	}
	

	template<class ClusteringMethod, class ClusterVector>
	void hclust_rnn_chain(ClusteringMethod method, ClusterVector& clusters) {

		typedef typename ClusteringMethod::cluster_type   cluster_type;
		typedef ClusterVector                             clusters_type;

		typedef typename ClusteringMethod::distancer_type distancer_type;
		typedef typename ClusteringMethod::merger_type    merger_type;

		typedef ChainEntry<typename clusters_type::value_type, typename distancer_type::result_type> entry_type;
		typedef std::pair<typename clusters_type::iterator, typename distancer_type::result_type>    nn_type;

		// Expand the size of clusters vector to the contain exactly the newly created clusters	
		size_t initial_clusters = clusters.size(), result_clusters = (initial_clusters * 2) - 1;
		clusters.reserve(result_clusters);
		
		std::vector<entry_type> chain;

		typename clusters_type::iterator next_unchained = clusters.begin();
		while (clusters.size() != result_clusters) {
			if (chain.empty()) {
				// Pick next "unchained" cluster as default
				chain.push_back( entry_type(*next_unchained) );
				++next_unchained;
			} else {

				// Find next nearest neighbor from remaining "unchained" clusters			
				ChainDistancer<typename clusters_type::value_type, distancer_type> dc(method.distancer, chain.back().cluster);	
				nn_type nn = nearest_neighbor(dc, next_unchained, clusters.end(), chain.back().to_dist);
				
				if (nn.first != clusters.end()) {
					std::iter_swap(next_unchained, nn.first);
					chain.push_back( entry_type(*next_unchained, nn.second) );
					++next_unchained;
				} else {
					// Tip of chain is recursive nearest neighbor
					entry_type c1(chain.back()); chain.pop_back();
					entry_type c2(chain.back()); chain.pop_back();

					// Merge clusters 1 & 2 into new cluster, appended to "unchained" clusters
					cluster_type* cn = new cluster_type(c2.cluster, c1.cluster, c1.to_dist);
					method.merger(*cn, *(cn->parent1()), *(cn->parent2()));
					clusters.push_back(cn);
				}
			}
		}

		

		// Cleanup cluster listing
		
		// Re-order the clusters, with initial clusters in the beginning, ordered by id
		// from 0 .. -initial_clusters, followed by the agglomerated clusters sorted by
		// increasing disimilarity.
		
		std::partition(clusters.begin(), clusters.end(), std::mem_fun(&cluster_type::initial));
		std::sort(clusters.begin(), clusters.begin()+initial_clusters, std::not2(cluster_type::id_predicate()));
		std::sort(clusters.begin() + initial_clusters, clusters.end(), cluster_type::disimilarity_predicate());

		for (size_t i=initial_clusters; i<result_clusters; i++) {
			clusters[i]->set_id(i - initial_clusters + 1);  // Use R hclust 1-indexed convention for Id's
			//std::cerr << clusters[i]->parent1()->id() << " " << clusters[i]->parent2()->id() << " " << clusters[i]->id() << " " << clusters[i]->disimilarity() << std::endl;
		}

	}

} // end of Rclustercpp namespace

#endif
