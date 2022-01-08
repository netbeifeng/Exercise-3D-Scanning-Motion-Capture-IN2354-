#pragma once

#ifndef IMPLICIT_SURFACE_H
#define IMPLICIT_SURFACE_H

#include "Eigen.h"
#include "SimpleMesh.h"

class ImplicitSurface
{
public:
	virtual double Eval(const Eigen::Vector3d& x) = 0;
};


class Sphere : public ImplicitSurface
{
public:
	Sphere(const Eigen::Vector3d& center, double radius) : m_center(center), m_radius(radius)
	{
	}

	double Eval(const Eigen::Vector3d& _x)
	{
		// TODO: implement the implicit sphere formula using the member variables m_center and m_radius
		
		return (_x - m_center).norm() - m_radius;
	}

private:
	Eigen::Vector3d m_center;
	double m_radius;
};


class Torus : public ImplicitSurface
{
public:
	Torus(const Eigen::Vector3d& center, double radius, double a) : m_center(center), m_radius(radius), m_a(a)
	{
	}

	double Eval(const Eigen::Vector3d& _x)
	{
		// TODO: implement the implicit torus formula using the  variables m_center, m_radius (radius of the ring) and the radius m_a (small radius)
		
		Vector3d point =  m_center - _x;
		// surface formula
		float val = pow(pow(point.x(), 2) + pow(point.y(), 2) + pow(point.z(), 2) + pow(m_radius, 2) - pow(m_a, 2), 2) - 4 * pow(m_radius, 2) * (pow(point.x(), 2) + pow(point.y(), 2));
		return val;
	}

private:
	Eigen::Vector3d m_center;
	double m_radius;
	double m_a;
};


class Hoppe : public ImplicitSurface
{
public:
	Hoppe(const std::string& filenamePC)
	{
		m_pointcloud.ReadFromFile(filenamePC);
	}

	double Eval(const Eigen::Vector3d& _x)
	{
		Eigen::Vector3f x = Eigen::Vector3f((float)_x.x(), (float)_x.y(), (float)_x.z());
		unsigned int idx = m_pointcloud.GetClosestPoint(x);
		if (idx == m_pointcloud.GetPoints().size()) return 0.0;

		Eigen::Vector3f p = m_pointcloud.GetPoints()[idx];
		Eigen::Vector3f n = m_pointcloud.GetNormals()[idx];

		// TODO: implement the evaluation using Hoppe's method (see lecture slides)
		
		// std::cout << (x - p).dot(n) << std::endl;
		return (x - p).dot(n);
	}

private:
	PointCloud m_pointcloud;
};

///////////////////////////////////////////

class FunctionSamples
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	FunctionSamples() {}

	void insertSample(const Vector3d& pos, const double targetFunctionValue)
	{
		m_pos.push_back(pos);
		m_val.push_back(targetFunctionValue);
	}

	std::vector<Vector3d> m_pos;
	std::vector<double> m_val;
};

#ifdef ORIGINAL
class RBF : public ImplicitSurface
{
public:
	RBF(const std::string& filenamePC)
	{
		// load point cloud
		m_pointcloud.ReadFromFile(filenamePC);

		// Create function samples
		double eps = 0.01f;
		for (unsigned int i = 0; i < m_pointcloud.GetPoints().size(); i++)
		{
			Eigen::Vector3f ptF = m_pointcloud.GetPoints()[i];
			Eigen::Vector3f nF = m_pointcloud.GetNormals()[i];
			const Vector3d pt(ptF[0], ptF[1], ptF[2]);
			const Vector3d n(nF[0], nF[1], nF[2]);

			m_funcSamp.insertSample(pt, 0);
			m_funcSamp.insertSample(pt + n*eps, eps);
			eps *= -1;
		}

		m_numCenters = m_funcSamp.m_pos.size();
		const unsigned int dim = m_numCenters + 4;

		// build and solve the linear system of equations
		m_systemMatrix = MatrixXd(dim, dim);
		m_rhs = VectorXd(dim);
		m_coefficents = VectorXd(dim); // result of the linear system
		BuildSystem();
		SolveSystem();
	}

	double Eval(const Eigen::Vector3d& _x)
	{
		// TODO: eval the RBF function based on the coefficents stored in m_coefficents (see lecture slides)
		// the first m_numCenters entries contain the coefficients of the kernels (that can be evaluated using EvalBasis())
		// the following parameters are the coeffients for the linear and the constant part
		double result = 0.0;


		return result;
	}

private:

	double EvalBasis(double x)
	{
		return x*x*x;
	}

	void CompResidual()
	{
		double residual = 0;
		for (int x = 0; x < m_numCenters; x++)
		{
			Vector3d pt = m_funcSamp.m_pos[x];
			residual += fabs(Eval(pt) - m_funcSamp.m_val[x]);
		}
		residual /= (double)m_numCenters;

		std::cerr << "Residual: " << residual << std::endl;
	}

#define phi(i,j) EvalBasis((m_funcSamp.m_pos[i]-m_funcSamp.m_pos[j]).norm())
	//! Computes the system matrix.
	void BuildSystem()
	{
		m_systemMatrix.setZero();
		m_rhs.setZero();

		for (int i = 0; i<m_numCenters; i++) {
			for (int j = 0; j<m_numCenters; j++) m_systemMatrix(i, j) = phi(i, j);

			m_systemMatrix.row(i).segment(m_numCenters, 3) = m_funcSamp.m_pos[i];
			m_systemMatrix.row(i)(m_numCenters + 3) = 1;

			m_systemMatrix.col(i).segment(m_numCenters, 4) = m_systemMatrix.row(i).segment(m_numCenters, 4);

			m_rhs(i) = m_funcSamp.m_val[i];
		}

		// regularizer -> smoother surface
		m_systemMatrix.diagonal() += 0.001 * VectorXd::Ones(m_numCenters + 4);
	}

	void SolveSystem()
	{
		std::cerr << "Solving RBF System" << std::endl;
		std::cerr << "Computing LU..." << std::endl;

		FullPivLU<Matrix<double, Dynamic, Dynamic>> LU(m_systemMatrix);
		m_coefficents = LU.solve(m_rhs);

		std::cerr << "Done." << std::endl;
	}

	// point cloud
	PointCloud m_pointcloud;

	//! The given function samples (at each function sample, we place a basis function).
	FunctionSamples m_funcSamp;

	//! The number of center = number of function samples.
	unsigned int m_numCenters;

	//! the right hand side of our system of linear equation. Unfortunately, float-precision is not enough, so we have to use double here.
	VectorXd m_rhs;

	//! the system matrix. Unfortunately, float-precision is not enough, so we have to use double here
	MatrixXd m_systemMatrix;

	//! store the result of the linear system here. Unfortunately, float-precision is not enough, so we have to use double here
	VectorXd m_coefficents;
};

#else

class RBF : public ImplicitSurface
{
public:
	RBF(const std::string& filenamePC)
	{
		// load point cloud
		m_pointcloud.ReadFromFile(filenamePC);

		// Create function samples
		double eps = 0.01f;

		// on surface points (-> center points of the RBFs)
		for (unsigned int i = 0; i < m_pointcloud.GetPoints().size(); i++)
		{
			Eigen::Vector3f ptF = m_pointcloud.GetPoints()[i];
			Eigen::Vector3f nF = m_pointcloud.GetNormals()[i];
			const Vector3d pt(ptF[0], ptF[1], ptF[2]);
			const Vector3d n(nF[0], nF[1], nF[2]);

			m_funcSamp.insertSample(pt, 0); // on surface point => distance = 0
		}

		// off surface points
		for (unsigned int i = 0; i < m_pointcloud.GetPoints().size(); i++)
		{
			Eigen::Vector3f ptF = m_pointcloud.GetPoints()[i];
			Eigen::Vector3f nF = m_pointcloud.GetNormals()[i];
			const Vector3d pt(ptF[0], ptF[1], ptF[2]);
			const Vector3d n(nF[0], nF[1], nF[2]);

			m_funcSamp.insertSample(pt + n*eps, eps);// off surface point => distance = eps
			eps *= -1;
		}

		m_numCenters = (unsigned int) m_pointcloud.GetPoints().size();
		const unsigned int dim = m_numCenters + 4;

		// build and solve the linear system of equations
		m_systemMatrix = MatrixXd(dim, dim);
		m_rhs = VectorXd(dim);
		m_coefficents = VectorXd(dim); // result of the linear system
		BuildSystem();
		SolveSystem();
	}

	double Eval(const Eigen::Vector3d& _x)
	{
		// TODO: eval the RBF function based on the coefficents stored in m_coefficents
		// the first m_numCenters entries contain the coefficients of the kernels (that can be evaluated using EvalBasis())
		// the following parameters are the coeffients for the linear and the constant part
		// the centers of the RBFs are the first m_numCenters sample points (use m_funcSamp.m_pos[i] to access them)
		// hint: Eigen provides a norm() function to compute the l2-norm of a vector (e.g. see macro phi(i,j))
		double result = 0.0;
		
		// calc first part with for loop sum
		for (int idx = 0; idx < m_numCenters; idx++) {
			result += m_coefficents[idx] * pow((m_funcSamp.m_pos[idx] - _x).norm(), 3);
		}

		// build up b vector from m_coefficents
		Vector3d b = Vector3d(m_coefficents[m_numCenters], m_coefficents[m_numCenters + 1], m_coefficents[m_numCenters + 2]);
		// calc second part and add it to result b dot x
		result += (b.dot(_x));

		// add last part d
		result += m_coefficents[m_numCenters + 3];

		return result;
	}

private:

	double EvalBasis(double x)
	{
		return x*x*x;
	}

#define phi(i,j) EvalBasis((m_funcSamp.m_pos[i]-m_funcSamp.m_pos[j]).norm())

	//! Computes the system matrix.
	void BuildSystem()
	{
		MatrixXd A(2 * m_numCenters, m_numCenters + 4);
		VectorXd b(2 * m_numCenters);
		A.setZero();
		b.setZero();

		// TODO fill the matrix A and the vector b as described in the exercise sheet
		// note that all sample points (both on and off surface points) are stored in m_funcSamp
		// you can access matrix elements using for example A(i,j) for the i-th row and j-th column
		// similar you access the elements of the vector b, e.g. b(i) for the i-th element
		
		for (int u = 0; u < m_numCenters * 2; u++) {
			// fill b with 0 ... 0 ±𝜀 ±𝜀 ±𝜀
			b(u) = m_funcSamp.m_val[u];

			for (int v = 0; v < m_numCenters; v++) {
				A(u, v) = phi(u, v);
			}

			// fill last col with px py pz 1
			A(u, m_numCenters)     = m_funcSamp.m_pos[u].x();
			A(u, m_numCenters + 1) = m_funcSamp.m_pos[u].y();
			A(u, m_numCenters + 2) = m_funcSamp.m_pos[u].z();
			A(u, m_numCenters + 3) = 1;
		}

		// build the system matrix and the right hand side of the normal equation
		m_systemMatrix = A.transpose() * A;
		m_rhs = A.transpose() * b;

		// regularizer -> smoother surface
		// pushes the coefficients to zero
		double lambda = 0.0001;
		m_systemMatrix.diagonal() += lambda * lambda * VectorXd::Ones(m_numCenters + 4);
	}

	void SolveSystem()
	{
		std::cerr << "Solving RBF System" << std::endl;
		std::cerr << "Computing LU..." << std::endl;

		FullPivLU<Matrix<double, Dynamic, Dynamic>> LU(m_systemMatrix);
		m_coefficents = LU.solve(m_rhs);

		std::cerr << "Done." << std::endl;
	}

	// point cloud
	PointCloud m_pointcloud;

	//! The given function samples (at each function sample, we place a basis function).
	FunctionSamples m_funcSamp;

	//! The number of center = number of function samples.
	unsigned int m_numCenters;

	//! the right hand side of our system of linear equation. Unfortunately, float-precision is not enough, so we have to use double here.
	VectorXd m_rhs;

	//! the system matrix. Unfortunately, float-precision is not enough, so we have to use double here
	MatrixXd m_systemMatrix;

	//! store the result of the linear system here. Unfortunately, float-precision is not enough, so we have to use double here
	VectorXd m_coefficents;
};

#endif

#endif //IMPLICIT_SURFACE_H
