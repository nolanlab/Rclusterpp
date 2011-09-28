#ifndef RCLUSTERP_UTIL_H
#define RCLUSTERP_UTIL_H

namespace Rclusterpp {

	namespace Util {
			
		template<class OP>
		class binder1st_P : public std::unary_function<typename OP::second_argument_type*, typename OP::result_type> {
		public:
			binder1st_P(OP& o, typename OP::first_argument_type const* a1) : op(o), arg(a1) {}
			typename OP::result_type operator()(const typename OP::second_argument_type* x) {
				return op(*arg, *x);
			}
			
		private:
			OP& op;
			typename OP::first_argument_type const* arg;
		};

		template<class OP>
		inline binder1st_P<OP> bind1st_P(OP& o, typename OP::first_argument_type const* a) { return binder1st_P<OP>(o,a); } 
		
	} // end of Util namespace
	
} // end of Rclusterpp namespace

#endif
