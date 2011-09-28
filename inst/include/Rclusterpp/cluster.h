#ifndef RCLUSTERPP_CLUSTER_H
#define RCLUSTERPP_CLUSTER_H

#include <limits>
#include <vector>

namespace Rclusterpp {
	
	template<typename Derived>
	class Cluster {
		public:

			typedef double distance_type;

			static ssize_t NULLID() { return std::numeric_limits<ssize_t>::min(); } 
	
		public:

			explicit Cluster(ssize_t id) : 
				id_(id), size_(1), parent1_(NULL), parent2_(NULL), disimilarity_(0) {}
			
			Cluster(Derived const * parent1, Derived const * parent2, distance_type disimilarity) :
				id_(NULLID()), size_(parent1->size()+parent2->size()), parent1_(parent1), parent2_(parent2), disimilarity_(disimilarity) {}
			
			ssize_t id() const { return id_; }
			void set_id(ssize_t id) { id_ = id; } 
			
			size_t  size() const { return size_; }

			bool initial() const { return parent1_ == NULL || parent2_ == NULL; }
			
			Derived const * parent1() const { return parent1_; }
			Derived const * parent2() const { return parent2_; }

			ssize_t parent1Id() const { return (parent1_) ? parent1_->id() : NULLID(); }
			ssize_t parent2Id() const { return (parent2_) ? parent2_->id() : NULLID(); }

			distance_type disimilarity() const { return disimilarity_; }
	
		private:
			
			ssize_t id_;
			size_t  size_;
	
			Derived const * parent1_;
			Derived const * parent2_;

			double disimilarity_;

		public:
	
			struct DisimilarityPredicate : std::binary_function<Derived const *, Derived const *, bool> {
				bool operator()(Derived const * lhs, Derived const * rhs) const {
					return lhs->disimilarity() < rhs->disimilarity();
				}
			};
			static DisimilarityPredicate disimilarity_predicate() { return DisimilarityPredicate(); }
	};

	template<class Cluster>
	inline bool compare_id(Cluster const* l, Cluster const* r) {
		return l->id() < r->id();
	}

	template<class Cluster>
	inline bool compare_disimilarity(Cluster const* l, Cluster const* r) {
		return l->disimilarity() < r->disimilarity();
	}

	template<int RTYPE>
	class ClusterWithCenter : public Cluster<ClusterWithCenter<RTYPE> > {
		private:
			typedef Cluster<ClusterWithCenter<RTYPE> > base_class;
		
		public:
			
			typedef typename base_class::distance_type distance_type;
			typedef Rcpp::Vector<RTYPE>               center_type;
			
		public:	
			
			ClusterWithCenter(ClusterWithCenter const * parent1, ClusterWithCenter const * parent2, distance_type disimilarity) : 
				base_class(parent1, parent2, disimilarity), center_(parent1->dim()) {} 

			ClusterWithCenter(ssize_t id, size_t obs_id, const center_type& vector) : base_class(id), center_(vector) {}
			
			size_t dim() const { return center_.size(); }
			typename center_type::stored_type size() const { return (typename center_type::stored_type)this->base_class::size(); }

			const center_type& center() const { return center_; }
			
			template<class T>
			void set_center(const T& center) { center_ = center; }

		private:
			
			center_type center_;
	};

	template<int RTYPE>
	class ClusterWithObs : public Cluster<ClusterWithObs<RTYPE> > {
		private:
			typedef Cluster<ClusterWithObs<RTYPE> > base_class;

		public:
			
			typedef typename base_class::distance_type distance_type;
			typedef typename std::vector<size_t>       idx_type;

			typedef typename idx_type::iterator       idx_iterator;
			typedef typename idx_type::const_iterator idx_const_iterator;

		public:

			ClusterWithObs(ClusterWithObs const * parent1, ClusterWithObs const * parent2, distance_type disimilarity) : 
				base_class(parent1, parent2, disimilarity), idxs_(parent1->idxs()) {
				
				// Append "merged" observation idxs	
				idxs_.insert(idxs_.end(), parent2->idxs_begin(), parent2->idxs_end());
			} 

			ClusterWithObs(ssize_t id, size_t obs_id, const Rcpp::Vector<RTYPE>& vector) : 
				base_class(id), idxs_(1, obs_id) {}
	
			const idx_type& idxs() const { return idxs_; }
			idx_const_iterator idxs_begin() const { return idxs_.begin(); }
			idx_const_iterator idxs_end() const { return idxs_.end(); }

		private:

			idx_type        idxs_;
	};

	template<class T>
	class ClusterVector {
		private:

			typedef std::vector<T*> underlying_type;
		
		public:

			typedef T cluster_type;
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

			template<class Vector>
			static cluster_type* make_cluster(ssize_t id, size_t obs_id, const Vector& vector) {
				return new cluster_type(id, obs_id, vector);
			}

		private:

			ClusterVector() : clusters_() {}
			explicit ClusterVector(const ClusterVector& v);

			size_t          initial_;  // Number of initial clusters
			underlying_type clusters_;

	};

	
	// NOTE: We can expand out this trait with template specialization
	// to support initialization from other sources than NumericMatrix, etc.

	template<int RTYPE>
		struct ClusterTypes {
			typedef ClusterWithCenter<RTYPE> center;
			typedef ClusterWithObs<RTYPE>    obs;
		};
	

} // end of Rcluserpp namespace


#endif
