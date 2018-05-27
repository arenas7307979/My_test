#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/opencv.hpp>
//Eigen
#include <Eigen/Core>
//稠密矩陣代數運算
#include<Eigen/Dense>
using namespace std;
#define MATRIX_SIZE 50

int main ( int argc, char**argv )
{
//  Eigen::Matrix
    Eigen::Matrix<float,2,3> matrix_23;
    matrix_23 << 1,2,3,4,5,6;
    Eigen::Matrix<float,2,2> matrix_22;
    matrix_22 << 1,2,3,4;
//  Eigen::Vector3d = Eigen::Matrix<double, 3, 3>
    Eigen::Vector3d v_3d;

// Eigen::Matrix3d
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();

    Eigen::Matrix<double, Eigen::Dynamic,  Eigen::Dynamic> matrix_dynamic;

//用()訪問矩陣中的元素
    for ( int i=0; i<2 ; i++ ) {
        for ( int j=0; j<2 ; j++ ) {
            //cout << matrix_23 ( i,j ) <<endl;
        }
    }

        for ( int i=0; i<2 ; i++ ) {
        for ( int j=0; j<2 ; j++ ) {
            cout << matrix_22 ( 0,j ) <<endl;
        }
    }
    
    v_3d<<3,2,1;

//要乘法要轉為相同矩陣
    Eigen::Matrix<double, 2, 1> results ;
    results = matrix_23.cast<double>()  * v_3d;



//Create Matrix 3d
    matrix_33 = Eigen::Matrix3d::Random();

//Solve eigen value
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver ( matrix_33.transpose() * matrix_33 );
    cout << "eigen value = " << eigen_solver.eigenvalues() <<endl;
    cout << "eigen vector = " << eigen_solver.eigenvectors() <<endl;
//Solve Equation
//QR and Inverse
    Eigen::Matrix<double, MATRIX_SIZE,MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random ( MATRIX_SIZE , MATRIX_SIZE );
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random ( MATRIX_SIZE, 1 );

//INVERSE
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd ;
   
//QU is quick to compute
    //time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);

}
