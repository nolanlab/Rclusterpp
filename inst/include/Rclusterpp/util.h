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
	
	} // end of Util namespace
	
} // end of Rclusterpp namespace

#endif
