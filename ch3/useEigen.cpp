#include <iostream>
#include "Eigen/Core"
#include "Eigen/Dense"

using namespace std;
#define MAT_SIZE 10
 

int main(int argc, char** argv){
    
    std::cout<<"This is the use Eigen script!"<<std::endl;

    
// Test the eigen mat with different datatype, size, read and write, access values
    Eigen::Vector3d vec_3d;
    Eigen::Vector4d vec_4d;
    Eigen::Matrix3d mat_3d = Eigen::Matrix3d::Ones();
    Eigen::Matrix4f mat_4f;
    //if didn't manual init, the mat will be inited with trash values
    Eigen::Matrix<double,3,4> mat_34d = Eigen::Matrix<double,3,4>::Zero();
    Eigen::MatrixXd mat_un_d;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> mat_dy_d;
    
    mat_4f<<1,2,3,4,
            1,2,3,4,
            1,2,3,4,
            4,4,4,4;
    vec_4d<<1,0,0,1;
/*    
    for(int i = 0; i < 4;i++)
        for(int j = 0; j < 4; j++)
            cout<<"value: "<<mat_4f(i,j)<<endl;*/
        
    
// Matrix operation
    mat_dy_d = mat_4f.cast<double>()*vec_4d;
    mat_un_d = mat_4f.cast<double>()*vec_4d;
    mat_4f = mat_4f*mat_4f.transpose();
    std::cout<<mat_dy_d.transpose()<<endl;
    std::cout<<mat_un_d.transpose()<<endl;    
    std::cout<<mat_4f<<endl;
    std::cout<<mat_4f.trace()<<" "<<mat_4f.sum()<<" "<<mat_4f.determinant()<<endl; 
    
        
        
    
// SVD

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> eigen_solver(mat_4f);
    cout<<eigen_solver.eigenvalues()<<endl;
    cout<<eigen_solver.eigenvectors()<<endl;
    
// solve functions   Ax = B in inverse manner and QR decomposition  

    Eigen::Matrix<double,MAT_SIZE,MAT_SIZE> mat_10d;
    Eigen::Matrix<double,MAT_SIZE,1> vec_10d;
    mat_10d = Eigen::Matrix<double,MAT_SIZE,MAT_SIZE>::Random();
    vec_10d = Eigen::MatrixXd::Random(MAT_SIZE,1);
    std::cout<<"mat_10d: \n"<<mat_10d.inverse()<<endl;
    std::cout<<"vec_10d: \n"<<vec_10d.transpose()<<endl;
    //Ax = B   
    Eigen::Matrix<double,MAT_SIZE,1> sol = mat_10d.inverse()*vec_10d;
    cout<<"sol : \n"<<sol.transpose()<<endl;
    cout<<"verify solution: \n"<<(mat_10d*sol).transpose()<<endl;
    cout<<"the diffence to true Mat B: \n"<<(mat_10d*sol-vec_10d).transpose()<<endl;
    
    
// solve via  QR
    sol = mat_10d.colPivHouseholderQr().solve(vec_10d);
    cout<<"sol via QR: \n"<<sol.transpose()<<endl;
    
    
    
    return 0;
    



    
}
