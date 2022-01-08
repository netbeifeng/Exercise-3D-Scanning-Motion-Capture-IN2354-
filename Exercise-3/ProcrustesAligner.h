#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;

		std::cout << estimatedPose << std::endl;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		// Hint: You can use the .size() method to get the length of a vector.
		Vector3f mean = Vector3f::Zero();

		for each (Vector3f p in points) {
			mean += p;
		}

		return mean / points.size();
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Hint: You can initialize an Eigen matrix with "MatrixXf m(num_rows,num_cols);" and access/modify parts of it using the .block() method (see above).
		
		auto size = sourcePoints.size();

		MatrixXf sourceMatrix_t(size, 3);
		for (size_t i = 0; i < size; i++) {
			ArrayXXf sourcePoint_a(1,3);
			sourcePoint_a << sourcePoints[i].x(), sourcePoints[i].y(), sourcePoints[i].z();
			sourceMatrix_t.block(i, 0, 1, 3) = sourcePoint_a;
		}

		MatrixXf targetMatrix_t(size, 3);
		for (size_t i = 0; i < size; i++) {
			ArrayXXf targetPoint_a(1, 3);
			targetPoint_a << targetPoints[i].x(), targetPoints[i].y(), targetPoints[i].z();
			targetMatrix_t.block(i, 0, 1, 3) = targetPoint_a;
		}

		MatrixXf tempMatrix = (targetMatrix_t.transpose() * sourceMatrix_t);
		JacobiSVD<MatrixXf> svd(tempMatrix, ComputeThinU | ComputeThinV);

		// std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
		// std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl << svd.matrixU() << std::endl;
		// std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl << svd.matrixV() << std::endl;

		MatrixXf U = svd.matrixU();
		MatrixXf V = svd.matrixV();

		MatrixXf R = U * V.transpose();

		float determinant_Of_UVT = R.determinant();
		// std::cout << (svd.matrixU() * svd.matrixV().transpose()).transpose() << std::endl;
		// std::cout << determinant_Of_UVT << std::endl;

		if (determinant_Of_UVT < 0) {
			MatrixXf tmp;
			tmp <<	1, 0, 0,
					0, 1, 0,
					0, 0, -1;
			R = U * tmp * V.transpose();
		}

		Matrix3f rotation = R;
        return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
		Vector3f translation = Vector3f::Zero();
		translation = - (rotation * sourceMean) + targetMean;
        return translation;
	}
};
