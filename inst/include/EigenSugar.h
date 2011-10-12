#ifndef EIGENSUGAR_H
#define EIGENSUGAR_H

namespace Eigen {
	
	template<class T>
	inline const CwiseUnaryOp<internal::scalar_square_op<typename T::Scalar>, const T>
	square(const T& t) {
		return t.square();
	}

	template<class T>
	inline typename internal::traits< T >::Scalar 
	sum(const T& t) {
		return t.sum();
	}
}

#endif
