#ifndef RCLUSTERPPEIGENARRAYPLUGIN_H
#define RCLUSTERPPEIGENARRAYPLUGIN_H

RealScalar squaredNorm() const {
	return square().sum();
}

RealScalar norm() const {
	return sqrt(square().sum());
}

#endif
