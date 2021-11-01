#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

using namespace std;
int main(int argc, char** argv){

std::cout<<"This is for testing Sophus!"<<endl;
//create so3 via angle axis, rot_r, q
Eigen::AngleAxisd rot_aa(M_PI/4,Eigen::Vector3d(0,0,1));
Eigen::Matrix3d rot_r = rot_aa.toRotationMatrix();
Eigen::Quaterniond rot_q(rot_aa);


Sophus::SO3  SO3_q(rot_q);
Sophus::SO3  SO3_r(rot_r);
//construct from rot_aa is not straigt forward.
Sophus::SO3  SO3_aa(0,0,M_PI/4);
std::cout<<"rot mat: \n"<<rot_r<<endl;
//cout:  SO3_r.matrix() is the lie group; SO3_r is the lie algbra
std::cout<<"sophus: \n"<<SO3_q.matrix()<<" \n"<<SO3_r.matrix()<<endl;  
//the corrosponding lie algbra
Eigen::Vector3d so3_q = SO3_q.log();
Eigen::Vector3d so3_r = SO3_r.log();
std::cout<<"lie algbra so3: \n"<<so3_q.transpose()<<"\n"<<so3_r.transpose()<<endl;




Eigen::Vector3d translation(3,4,5);
Sophus::SE3 SE3_q(rot_q,translation);
Sophus::SE3 SE3_r(rot_r,translation);
std::cout<<"sophus: \n"<<SE3_q.matrix()<<" \n "<<SE3_r.matrix()<<endl;
std::cout<<"lie algbra se3: \n"<<SE3_q.log().transpose()<<endl;// rou theta
std::cout<<"lie algbra so3 + translation : \n"<<SE3_r<<endl;


// test log and exp

std::cout<<"test exp of so3: \n"<<Sophus::SO3::exp(so3_q).matrix()<<endl;
std::cout<<"test exp of se3: \n"<<Sophus::SE3::exp(SE3_q.log()).matrix()<<endl;
 
// vee and hat operation
// the output is not SO3, hat and vee is just for 反对称矩阵！
std::cout<<"test hat of so3: \n"<<Sophus::SO3::hat(so3_q)<<endl; 
std::cout<<"test hat of se3: \n"<<Sophus::SE3::hat(SE3_q.log())<<endl; 
// note the param of vee is not lie group var Sophus::SO3, it should be a Eigen mat!
// the out put of vee is not really lie algbra, it is actual translaiton!
std::cout<<"test vee of SO3: \n"<<Sophus::SO3::vee(SO3_q.matrix()).transpose()<<endl;
std::cout<<"test vee of SE3: \n"<<Sophus::SE3::vee(SE3_q.matrix()).transpose()<<endl;
 


// operation of two so3  se3
 
Eigen::Matrix<double, 6, 1> delta_vec = Eigen::Matrix<double, 6, 1>::Zero();
delta_vec(0,0) = 1e-4d; // A minor translaiton on x
Sophus::SE3 new_SE3 = Sophus::SE3::exp(delta_vec) * SE3_q;
Eigen::MatrixXd new_SE3_mat = Sophus::SE3::exp(delta_vec).matrix() * SE3_q.matrix();
std::cout<<"new_SE3: \n"<<new_SE3<<"\n"<<new_SE3.matrix()<<"\n"<<new_SE3.log().transpose()<<endl;
std::cout<<"new_SE3_mat: \n"<<new_SE3_mat<<endl;






return 0;
}
