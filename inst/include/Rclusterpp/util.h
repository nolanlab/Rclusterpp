#ifndef RCLUSTERP_UTIL_H
#define RCLUSTERP_UTIL_H

namespace Rclusterpp {

	namespace Util {
			
		template<class OP>
		class ClusterBinder : public std::binary_function<typename OP::second_argument_type*, typename OP::third_argument_type, typename OP::result_type> {
		public:
			ClusterBinder(OP& o, typename OP::first_argument_type const* a1) : op(o), arg(a1) {}
			typename OP::result_type operator()(const typename OP::second_argument_type* x, const typename OP::third_argument_type& d) {
				return op(*arg, *x, d);
			}
			
		private:
			OP& op;
			const typename OP::first_argument_type* arg;
		};

		template<class OP>
		inline ClusterBinder<OP> cluster_bind(OP& o, const typename OP::first_argument_type* a) { 
			return ClusterBinder<OP>(o,a); 
		} 


		class IndexList {
			private:
				typedef std::vector<std::pair<size_t, size_t> > indexes_type;
		
			public:
				IndexList(size_t n) : idxs(n+1), begin_(0), end_(n) {
					idxs[0] = std::make_pair(0, 1);
					for (size_t i=1; i<end_; i++)
						idxs[i] = std::make_pair(i-1, i+1);
				}

				size_t begin() const { return begin_; }
				size_t end() const { return end_; }

				size_t succ(size_t i) const {
					return idxs[i].second;
				}

				void remove(size_t i) {
					if (i == begin_) {
						begin_ = idxs[i].second;
					} else {
						idxs[idxs[i].first].second = idxs[i].second;
						idxs[idxs[i].second].first = idxs[i].first;
					}
				}
 
			private:
				indexes_type idxs;
				size_t       begin_, end_;
		};
	
	} // end of Util namespace
	
} // end of Rclusterpp namespace

#endif
