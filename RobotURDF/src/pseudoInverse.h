#pragma once

//#include "stdafx.h"
#include<iostream> 
#include <Eigen/Dense>
#include<Eigen/Core> 
#include<Eigen/SVD>  

using namespace Eigen; 

/*
MatrixXd pinv(MatrixXd M) { 
	M.completeOrthogonalDecomposition().pseudoInverse();
	return M; 
}  */


	template <class MatT>
	inline Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
		pseudoInverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{ 1e-9 }) // choose appropriately
	{
		typedef typename MatT::Scalar Scalar;
		auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
		const auto &singularValues = svd.singularValues();
		Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
		singularValuesInv.setZero();
		for (unsigned int i = 0; i < singularValues.size(); ++i) {
			if (singularValues(i) > tolerance)
			{
				singularValuesInv(i, i) = Scalar{ 1 } / singularValues(i);
			}
			else
			{
				singularValuesInv(i, i) = Scalar{ 0 };
			}
		}
		return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
	}


/*
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
	std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}*/
