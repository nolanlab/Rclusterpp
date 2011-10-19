#ifndef RCLUSTERPPEIGENSUGAR_H
#define RCLUSTERPPEIGENSUGAR_H

namespace Eigen {
	
	// These return types are copied from the Eigen classes

	template<class T>
	inline const CwiseUnaryOp<internal::scalar_abs_op<typename T::Scalar>, const T> abs(const T& t) {
		return t.abs();
	}

	template<class T>
	inline typename T::Scalar maxCoeff(const T& t) {
		return t.maxCoeff();
	}

	template<class T>
	inline typename T::RealScalar lpNorm(const T& t, double p) {
		return t.lpNorm(p);
	}

	template<class T>
	inline typename T::RealScalar norm(const T& t) {
		return t.norm();
	}

	template<class T>
	inline typename T::RealScalar squaredNorm(const T& t) {
		return t.squaredNorm();
	}


	template<class T>
	inline const CwiseUnaryOp<internal::scalar_square_op<typename T::Scalar>, const T> square(const T& t) {
		return t.square();
	}

	template<class T>
	inline typename T::Scalar sum(const T& t) {
		return t.sum();
	}

} // namespace Eigen

#endif
