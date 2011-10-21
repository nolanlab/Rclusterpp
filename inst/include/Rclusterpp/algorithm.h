#ifndef RCLUSTERP_ALGORITHM_H
#define RCLUSTERP_ALGORITHM_H

#include <limits>
#include <utility>
#include <algorithm>
#include <functional>
#include <stack>

#include <Rclusterpp/cluster.h>
#include <Rclusterpp/util.h>

namespace Rclusterpp {

	template<class RandomIterator, class Distancer>
	std::pair<RandomIterator, typename Distancer::result_type> nearest_neighbor(
		const RandomIterator& first, 
		const RandomIterator& last, 
		Distancer       distancer, 
		typename Distancer::result_type max_dist=std::numeric_limits<typename Distancer::result_type>::max()
	) {
	
		typedef typename Distancer::result_type Dist_t;

		RandomIterator min_i = last;
		Dist_t         min_d = max_dist;

#ifdef _OPENMP
		#pragma omp parallel shared(min_i, min_d, distancer)	
#endif
		{
			RandomIterator min_i_l;
			Dist_t         min_d_l = min_d;

#ifdef _OPENMP
			#pragma omp for nowait	
#endif
			for (ssize_t i=0; i<(last-first); i++) {
				Dist_t dist = distancer(*(first+i), min_d_l);				
				if (dist < min_d_l) {
					min_i_l = first + i; 
					min_d_l = dist;
				}
			}

#ifdef _OPENMP
			#pragma omp critical	
#endif
			{
				if (min_d_l < min_d) {
					min_i = min_i_l; 
					min_d = min_d_l; 
				}
			}
		}
		return std::make_pair(min_i, min_d);	
	}


	template<class ClusteringMethod, class ClusterVector>
	void cluster_via_rnn(ClusteringMethod method, ClusterVector& clusters) {

		typedef ClusterVector                            clusters_type;
		typedef typename clusters_type::cluster_type     cluster_type;
		typedef typename ClusteringMethod::distance_type distance_type;
	
		// Result from nearest neighbor scan
		typedef std::pair<typename clusters_type::iterator, distance_type>  nearn_type;

#define nn_cluster(x) (x).first
#define distance_to_nn(x) (x).second

		// Nearest neighbor chain
		typedef std::pair<typename clusters_type::value_type, distance_type> entry_type;

		std::stack<entry_type> chain;

#define cluster_at_tip(x) (x).top().first
#define distance_to_tip(x) (x).top().second
								
		// Expand the size of clusters vector to the contain exactly the newly created clusters	
		size_t initial_clusters = clusters.size(), result_clusters = (initial_clusters * 2) - 1;
		clusters.reserve(result_clusters);
		
		// List of valid clusters (used in mer
		Util::IndexList valid(initial_clusters);

		typename clusters_type::iterator next_unchained = clusters.begin();
		while (clusters.size() != result_clusters) {
			if (chain.empty()) {
				// Pick next "unchained" cluster as default
				chain.push( entry_type(*next_unchained, std::numeric_limits<distance_type>::max()) );
				++next_unchained;
			} else {

				// Find next nearest neighbor from remaining "unchained" clusters			
				nearn_type nn = nearest_neighbor(
					next_unchained, 
					clusters.end(), 
					Util::cluster_bind(method.distancer, cluster_at_tip(chain)), // Bind tip into distance function for computing nearest neighbor 
					distance_to_tip(chain)
				);
				
				if (nn.first != clusters.end()) {
					std::iter_swap(next_unchained, nn_cluster(nn));
					chain.push( entry_type(*next_unchained, distance_to_nn(nn)) );
					++next_unchained;
				} else {
					// Tip of chain is recursive nearest neighbor
					cluster_type* r = cluster_at_tip(chain);
					distance_type d = distance_to_tip(chain);
					chain.pop();

					cluster_type* l  = cluster_at_tip(chain);
					chain.pop();

					// Remove "tip" and "next tip"  from chain and merge into new cluster appended to "unchained" clusters
					cluster_type* cn = ClusterVector::make_cluster(std::min(l->idx(), r->idx()), l, r, d);
					
					valid.remove(std::max(r->idx(), l->idx()));
					method.merger(*cn, *(cn->parent1()), *(cn->parent2()), valid);
					
					clusters.push_back(cn);
				}
			}
		}
	
		// Cleanup cluster listing
		
		// Re-order the clusters, with initial clusters in the beginning, ordered by id
		// from -1 .. -initial_clusters, followed by the agglomerated clusters sorted by
		// increasing disimilarity.
		
		std::partition(clusters.begin(), clusters.end(), std::mem_fun(&cluster_type::initial));
		std::sort(clusters.begin(), clusters.begin()+initial_clusters, std::not2(std::ptr_fun(&compare_id<cluster_type>))); 
		std::sort(clusters.begin() + initial_clusters, clusters.end(), std::ptr_fun(&compare_disimilarity<cluster_type>)); 

		for (size_t i=initial_clusters; i<result_clusters; i++) {
			clusters[i]->set_id(i - initial_clusters + 1);  // Use R hclust 1-indexed convention for Id's
		}

	}

	namespace {

		typedef std::pair<size_t, size_t> Merge_t;

		inline Merge_t make_merge(size_t from, size_t into) { return std::make_pair(from, into); }
		inline size_t from(const Merge_t& m) { return m.first; }
		inline size_t into(const Merge_t& m) { return m.second; }

		template<class Distance>
		struct MergeCMP {
			const Distance& height;
		
			MergeCMP(const Distance& height_) : height(height_) {}	
			bool operator()(const Merge_t& a, const Merge_t& b) const {	
				return height[from(a)] < height[from(b)];	
			}
    };

	} // end of anonymous namespace		

	template<class Distancer, class ClusterVector>
	void cluster_via_slink(const Distancer& distancer, ClusterVector& clusters) {

		typedef typename Distancer::result_type distance_type;

		size_t initial_clusters = clusters.size(), result_clusters = (initial_clusters * 2) - 1;
		clusters.reserve(result_clusters);

		std::vector<size_t> P = std::vector<size_t>(initial_clusters);
		std::vector<distance_type> L = std::vector<distance_type>(initial_clusters);
		std::vector<distance_type> M = std::vector<distance_type>(initial_clusters);

		for (size_t i=0; i<initial_clusters; i++) {
			// Step 1: Initialize
			P[i] = i;
			L[i] = std::numeric_limits<distance_type>::max();

			// Step 2: Build out pairwise distances from objects in pointer
			// represenation to the new object
#ifdef _OPENMP	
			#pragma omp parallel for shared(i, M)
#endif
			for (ssize_t j=0; j<(ssize_t)i; j++) {
				M[j] = distancer(i, j); 
			}

			// Step 3: Update M, P, L
			for (size_t j=0; j<i; j++) {
				distance_type l = L[j], m = M[j];
				if (l >= m) {
					M[P[j]] = std::min(M[P[j]], l);
					L[j]    = m;
					P[j]    = i;
				} else {
					M[P[j]] = std::min(M[P[j]], m);
				}
			}

			// Step 4: Actualize the clusters
			for (size_t j=0; j<i; j++) {
				if (L[j] >= L[P[j]])
					P[j] = i;
			}
		}

		// Convert the pointer representation to dendogram 
		std::vector<Merge_t> merges = std::vector<Merge_t>(initial_clusters-1);
		for (size_t i=0; i<(initial_clusters-1); i++) {
			merges[i] = make_merge(i, P[i]); // from, into
		}
		std::sort(merges.begin(), merges.end(), MergeCMP<std::vector<distance_type> >(L));
		for (size_t i=0; i<initial_clusters; i++) {
			P[i] = i;
		}
		for (size_t i=0; i<(initial_clusters-1); i++) {
			size_t f = from(merges[i]), t = into(merges[i]);
			clusters.push_back(ClusterVector::make_cluster( 0, clusters[P[f]], clusters[P[t]], L[f] ));
			P[t] = i + initial_clusters;
		}
		
		for (size_t i=initial_clusters; i<result_clusters; i++) {
			clusters[i]->set_id(i - initial_clusters + 1);  // Use R hclust 1-indexed convention for Id's
		}

		
	}

} // end of Rclustercpp namespace

#endif
