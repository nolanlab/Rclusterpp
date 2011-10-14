#ifndef RCLUSTERPPEIGENMATRIXPLUGIN_H
#define RCLUSTERPPEIGENMATRIXPLUGIN_H

const CwiseUnaryOp<internal::scalar_abs_op<Scalar>, const Derived> abs() const {
	return cwiseAbs();
}

#endif
