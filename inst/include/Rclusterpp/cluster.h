#ifndef RCLUSTERPP_CLUSTER_H
#define RCLUSTERPP_CLUSTER_H

#include <limits>
#include <vector>

namespace Rclusterpp {
	
	template<class Derived, class Distance=double>
	class Cluster {
		public:

			typedef Distance distance_type;

			static ssize_t NULLID() { return std::numeric_limits<ssize_t>::min(); } 
	
		public:

			Cluster(ssize_t id, size_t idx) : 
				id_(id), idx_(idx), size_(1), parent1_(NULL), parent2_(NULL), disimilarity_(0) {}
			
			Cluster(size_t idx, Derived const * parent1, Derived const * parent2, distance_type disimilarity) :
				id_(NULLID()), idx_(idx), size_(parent1->size()+parent2->size()), parent1_(parent1), parent2_(parent2), disimilarity_(disimilarity) {}
							
			ssize_t id() const { return id_; }
			void set_id(ssize_t id) { id_ = id; } 
		
			size_t idx() const { return idx_; }
			size_t set_idx(size_t idx) { idx_ = idx; return idx; }

			size_t size() const { return size_; }

			bool initial() const { return parent1_ == NULL || parent2_ == NULL; }
			
			Derived const * parent1() const { return parent1_; }
			Derived const * parent2() const { return parent2_; }

			ssize_t parent1Id() const { return (parent1_) ? parent1_->id() : NULLID(); }
			ssize_t parent2Id() const { return (parent2_) ? parent2_->id() : NULLID(); }

			void swap_parents() { std::swap(parent1, parent2); }

			distance_type disimilarity() const { return disimilarity_; }
	
		private:
			
			ssize_t id_;
			size_t  idx_;
			size_t  size_;
	
			Derived const * parent1_;
			Derived const * parent2_;

			distance_type disimilarity_;

	};

	template<class Cluster>
	inline bool compare_id(Cluster const* l, Cluster const* r) {
		return l->id() < r->id();
	}

	template<class Cluster>
	inline bool compare_disimilarity(Cluster const* l, Cluster const* r) {
		return l->disimilarity() < r->disimilarity();
	}


	class ClusterWithID : public Cluster<ClusterWithID> {
		private:
			typedef Cluster<ClusterWithID> base_class;
		
		public:

			typedef base_class::distance_type distance_type;
			
		public:	
			
			ClusterWithID(size_t idx, ClusterWithID const * parent1, ClusterWithID const * parent2, distance_type disimilarity) : 
				base_class(idx, parent1, parent2, disimilarity) {} 

			ClusterWithID(ssize_t id, size_t obs_id) : base_class(id, obs_id) {}

			template<class V>
			ClusterWithID(ssize_t id, size_t obs_id, const V& v) : base_class(id, obs_id) {}
			
	};


	template<class Value>
	class ClusterWithCenter : public Cluster<ClusterWithCenter<Value> > {
		private:
			typedef Cluster<ClusterWithCenter<Value> > base_class;
		
		public:

			typedef Value                                value_type;	
			typedef typename base_class::distance_type   distance_type;
			typedef Eigen::Array<Value,1,Eigen::Dynamic> center_type;
			
		public:	
			
			ClusterWithCenter(size_t idx, ClusterWithCenter const * parent1, ClusterWithCenter const * parent2, distance_type disimilarity) : 
				base_class(idx, parent1, parent2, disimilarity), center_() {} 

			template<class V>
			ClusterWithCenter(ssize_t id, size_t obs_id, const V& v) : base_class(id, obs_id), center_(v) {}
			
			size_t dim() const { return center_.size(); }
	
			void set_center(const center_type& v) { center_ = v; }
			const center_type& center() const { return center_; }
	
		private:
			
			center_type center_;
	};

	class ClusterWithObs : public Cluster<ClusterWithObs> {
		private:
			typedef Cluster<ClusterWithObs> base_class;

		public:
			
			typedef base_class::distance_type distance_type;
			typedef std::vector<size_t>       idx_type;

			typedef idx_type::iterator       idx_iterator;
			typedef idx_type::const_iterator idx_const_iterator;

		public:

			ClusterWithObs(size_t idx, ClusterWithObs const * parent1, ClusterWithObs const * parent2, distance_type disimilarity) : 
				base_class(idx, parent1, parent2, disimilarity), idxs_(parent1->idxs()) {
				
				// Append "merged" observation idxs	
				idxs_.insert(idxs_.end(), parent2->idxs_begin(), parent2->idxs_end());
			} 

			template<class V>
			ClusterWithObs(ssize_t id, size_t obs_id, const V& vector) : 
				base_class(id, obs_id), idxs_(1, obs_id) {}
	
			const idx_type& idxs() const { return idxs_; }
			idx_const_iterator idxs_begin() const { return idxs_.begin(); }
			idx_const_iterator idxs_end()   const { return idxs_.end(); }

		private:

			idx_type        idxs_;
	};

	template<class T>
	class ClusterVector {
		private:

			typedef std::vector<T*> underlying_type;
		
		public:

			typedef T cluster_type;
			typedef typename cluster_type::distance_type      distance_type;
			typedef typename underlying_type::value_type      value_type;
			typedef typename underlying_type::reference       reference;
			typedef typename underlying_type::const_reference const_reference;
			typedef typename underlying_type::iterator        iterator;
			typedef typename underlying_type::const_iterator  const_iterator;
			typedef typename underlying_type::size_type       size_type;
		
			ClusterVector(size_t n) : initial_(n), clusters_(n) {}
			
			~ClusterVector() {
				for (iterator i=begin(), e=end(); i!=e; ++i)
					delete *i;
			}

			iterator begin() { return clusters_.begin(); }
			iterator end() { return clusters_.end(); }

			const_iterator begin() const { return clusters_.begin(); }
			const_iterator end() const { return clusters_.end(); }

			size_type size() const { return clusters_.size(); }
			void reserve(size_type n) { clusters_.reserve(n); }

			reference operator[](size_type n) { return clusters_[n]; }
			const_reference operator[](size_type n) const { return clusters_[n]; }
			void push_back(const value_type& v) { clusters_.push_back(v); }

			size_t initial_clusters() const { return initial_; }

			static cluster_type* make_cluster(ssize_t id, size_t obs_id) {
				return new cluster_type(id, obs_id);
			}

			template<class Vector>
			static cluster_type* make_cluster(ssize_t id, size_t obs_id, const Vector& vector) {
				return new cluster_type(id, obs_id, vector);
			}

			static cluster_type* make_cluster(size_t idx, cluster_type const * parent1, cluster_type const * parent2, distance_type disimilarity) {
				return new cluster_type(idx, parent1, parent2, disimilarity);
			}


		private:

			ClusterVector() : clusters_() {}
			explicit ClusterVector(const ClusterVector& v);

			size_t          initial_;  // Number of initial clusters
			underlying_type clusters_;

	};

	
	// NOTE: We can expand out this trait with template specialization
	// to support initialization from other sources than NumericMatrix, etc.

	template<class Value>
		struct ClusterTypes {
			typedef ClusterWithID            plain;
			typedef ClusterWithCenter<Value> center;
			typedef ClusterWithObs           obs;
		};
	

} // end of Rcluserpp namespace


#endif
