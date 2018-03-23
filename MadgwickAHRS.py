import numpy as np
class madgwick:

	beta = 0.2			# 2 * proportional gain (Kp)
	q0 = 1.0
	q1 = 0.0
	q2 = 0.0
	q3 = 0.0	# quaternion of sensor frame relative to auxiliary frame

	def invSqrt(self,x):
		return 1.0/(x)**(1/2.0)

	def MadgwickAHRSupdate(self,gx, gy, gz, ax, ay, az, mx, my, mz,del_t):
		beta = madgwick.beta
		q0 = madgwick.q0
		q1 = madgwick.q1
		q2 = madgwick.q2
		q3 = madgwick.q3
		recipNorm = 0.0
		s0 = 0.0
		s1 = 0.0
		s2 = 0.0
		s3 = 0.0
		qDot1 = 0.0
		qDot2 = 0.0
		qDot3 = 0.0
		qDot4 = 0.0
		hx = 0.0
		hy = 0.0


		# Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)

		# Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

		# Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):

			# Normalise accelerometer measurement
			recipNorm = self.invSqrt(ax * ax + ay * ay + az * az)
			ax *= recipNorm
			ay *= recipNorm
			az *= recipNorm

			# Normalise magnetometer measurement
			recipNorm = self.invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm
			my *= recipNorm
			mz *= recipNorm

			# Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0 * q0 * mx
			_2q0my = 2.0 * q0 * my
			_2q0mz = 2.0 * q0 * mz
			_2q1mx = 2.0 * q1 * mx
			_2q0 = 2.0 * q0
			_2q1 = 2.0 * q1
			_2q2 = 2.0 * q2
			_2q3 = 2.0 * q3
			_2q0q2 = 2.0 * q0 * q2
			_2q2q3 = 2.0 * q2 * q3
			q0q0 = q0 * q0
			q0q1 = q0 * q1
			q0q2 = q0 * q2
			q0q3 = q0 * q3
			q1q1 = q1 * q1
			q1q2 = q1 * q2
			q1q3 = q1 * q3
			q2q2 = q2 * q2
			q2q3 = q2 * q3
			q3q3 = q3 * q3

			# Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = (hx * hx + hy * hy)**(1/2.0)
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0 * _2bx
			_4bz = 2.0 * _2bz

			# Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
			recipNorm = self.invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); # normalise step magnitude
			s0 *= recipNorm
			s1 *= recipNorm
			s2 *= recipNorm
			s3 *= recipNorm

			# Apply feedback step
			qDot1 -= beta * s0
			qDot2 -= beta * s1
			qDot3 -= beta * s2
			qDot4 -= beta * s3


		# Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * del_t
		q1 += qDot2 * del_t
		q2 += qDot3 * del_t
		q3 += qDot4 * del_t

		# Normalise quaternion
		recipNorm = self.invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
		q0 *= recipNorm
		q1 *= recipNorm
		q2 *= recipNorm
		q3 *= recipNorm

		madgwick.beta = beta
		madgwick.q0 = q0
		madgwick.q1 = q1
		madgwick.q2 = q2
		madgwick.q3 = q3
		return np.array([q0,q1,q2,q3])

	#---------------------------------------------------------------------------------------------------
	# IMU algorithm update
