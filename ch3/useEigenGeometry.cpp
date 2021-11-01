#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;


int main(int argc, char** argv){
    
    
//Test the expression of rotation : anagle axis; rot mat; Euler(rpy, zyx); Quaterniond; Trans mat(Euler transformation)
    
// anagle axis; Eigen::AngleAxised
// mat;  Eigen::Matrix3d
// Euler(rpy, zyx);  Eigen::Vector3d
// Quaterniond; Eigen::Quaterniond 
// Trans mat(Euler transformation): Eigen::Isometry3d
    
    
    Eigen::AngleAxisd rot_aa(M_PI/4, Eigen::Vector3d(0,0,1));
    cout<<"angle: "<<rot_aa.angle()<<"   axis: "<<rot_aa.axis().transpose()<<endl;
    cout<<"rot aa: \n"<<rot_aa.matrix()<<endl;  //  rigorous formula is auto carried out here!
    Eigen::Matrix3d rot_mat(rot_aa);
    cout<<"rot mat: \n"<<rot_mat<<endl;
    rot_aa.fromRotationMatrix(rot_mat);
    cout<<"rot aa: \n"<<rot_aa.matrix()<<endl;
    
    
    Eigen::Vector3d rot_rpy = rot_mat.eulerAngles(2,1,0);   //rpy corrosponds 2 1 0  z y x
    cout<<"rpy: \n"<<rot_rpy.transpose()<<endl;
    Eigen::Quaterniond rot_q(rot_aa);
    cout<<"q: \n"<<rot_q.coeffs().transpose()<<endl;  //  x y z w
    rot_q = Eigen::Quaterniond(rot_mat);
    cout<<"q: \n"<<rot_q.coeffs().transpose()<<endl;  //  x y z w    
    
    
    Eigen::Isometry3d rot_T = Eigen::Isometry3d::Identity(); // this line of code is necessary, why?
//     rot_T.rotate(rot_aa);
    rot_T.rotate(rot_mat);
    rot_T.pretranslate(Eigen::Vector3d(1,2,30));
    cout<<"rot T mat: \n "<<rot_T.matrix()<<"\n rotation: \n"<<rot_T.rotation()<<"\n translation: \n"<<rot_T.translation().transpose()<<endl;
    
    
    Eigen::Vector3d P(10,10,10);
    Eigen::Vector3d P_new;
    P_new = rot_aa*P;
    cout<<"new P: \n"<<(rot_aa*P).transpose()<<" \n"<<(rot_mat*P).transpose()
    <<" \n"<<(rot_q*P).transpose()<<"\n"<<(rot_T*P).transpose()<<endl;
    
    
    
    
    
    
    
// how to convert one to the others formilarities    
//test transform 3D point via given transoformation in different formilarities
    
    return 0;
}
