#ifndef RCLUSTERPPEIGENMATRIXPLUGIN_H
#define RCLUSTERPPEIGENMATRIXPLUGIN_H

const CwiseUnaryOp<internal::scalar_abs_op<Scalar>, const Derived> abs() const {
	return cwiseAbs();
}

RealScalar lpNorm(double p) const {
	return std::pow(array().abs().pow(p).sum(), 1/p);
}

#endif
