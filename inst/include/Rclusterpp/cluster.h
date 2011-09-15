#ifndef RCLUSTERPP_CLUSTER_H
#define RCLUSTERPP_CLUSTER_H

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

			struct IdPredicate : std::binary_function<Derived const *, Derived const *, bool> {
				bool operator()(Derived const * lhs, Derived const * rhs) const {
					return lhs->id() < rhs->id();
				}
			};
			static IdPredicate id_predicate() { return IdPredicate(); }

			struct DisimilarityPredicate : std::binary_function<Derived const *, Derived const *, bool> {
				bool operator()(Derived const * lhs, Derived const * rhs) const {
					return lhs->disimilarity() < rhs->disimilarity();
				}
			};
			static DisimilarityPredicate disimilarity_predicate() { return DisimilarityPredicate(); }
	};

	template<typename T>
	class ClusterWithCenter : public Cluster<ClusterWithCenter<T> > {
		private:
			typedef Cluster<ClusterWithCenter<T> > base_type;
		
		public:
			
			typedef T data_type;
			typedef typename base_type::distance_type distance_type;
			typedef std::vector<data_type> center_type;
			
			typedef typename center_type::const_iterator const_center_iterator;
			typedef typename center_type::iterator center_iterator;

		public:	
			ClusterWithCenter(ClusterWithCenter const * parent1, ClusterWithCenter const * parent2, distance_type disimilarity) : 
				base_type(parent1, parent2, disimilarity), center(parent1->dim()) {} 

			template<int RTYPE>
			ClusterWithCenter(ssize_t id, Rcpp::MatrixRow<RTYPE> row) : base_type(id), center(row.begin(), row.end()) {}
			
			size_t dim() const { return center.size(); }
					
			const_center_iterator center_begin() const { return center.begin(); }
			center_iterator center_begin() { return center.begin(); }

			
		private:
			
			center_type center;
	};

	namespace Util {
	
		template<typename T>
		struct Dereference {
			typedef T value_type;
		};

		template<typename T>
		struct Dereference<T*> {
			typedef T value_type;
		};

	} // end of Util namespace 

	namespace Traits {
		
		template<typename Data>
		struct Cluster {
			typedef ClusterWithCenter<Data> center;
			typedef std::vector<center*>    center_vector;
		};

		template<typename Iter>
		struct Iterator {
			typedef typename Util::Dereference<typename Iter::value_type>::value_type cluster_type;
		};
	
	} // end of Traits namespace


	typedef Traits::Cluster<Rcpp::NumericMatrix::stored_type> NumericCluster;	

	void rows2clusters(NumericCluster::center_vector& clusters, Rcpp::NumericMatrix& matrix) {
		clusters.resize(matrix.nrow());
		for (ssize_t i=0; i<matrix.nrow(); i++) {
			clusters[i] = new NumericCluster::center(-(i+1), matrix.row(i));
		}
	}

}

#endif
