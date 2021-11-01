#include <iostream>
#include <cmath>


#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

using namespace std;


// construct a cost fucntional for futural use in ceres solving
struct COST_FUNCTIONAL{
    const double _x,_y;
    COST_FUNCTIONAL(double x, double y):_x(x),_y(y){}
    //error computation
    template <typename T>
    bool operator()(
        const T* const abc,
        T* residual)const
    {
        
        residual[0] = T(_y) -ceres::exp(abc[0]*T(_x)*T(_x)+abc[1]*T(_x)+abc[2]);
//         std::cout<<T.template<<endl;
        return true;
    }
    
    
    
    
    
    
    
    
};







int main(int argc, char** argv){
    

    
    
    //y = exp(ax^2+bx+c)+w    data preparation  
    int num4pts = 100;
    double sigma = 0.01;
    double abc[3] = {1,2,3};
    vector<double> x_s,y_s;
    cv::RNG rng;
    
    for(int i = 0; i<num4pts;i++){
        double x = double(i)/num4pts;
        double y = ceres::exp(abc[0]*x*x+abc[1]*x+abc[2])+rng.gaussian(sigma);
//         std::cout<<x<<" "<<y<<endl;
        x_s.push_back(x);
        y_s.push_back(y);
    }
    
    
    double abc_esti[3] = {1,2,7};//{0,0,0};// init values
    
    
    //contruct ceres problem
    // cores: define each cost func; in our problem :only one single cost func
    
    
    ceres::Problem curve_fit_plm;
    for(int i = 0; i<num4pts;i++){
        curve_fit_plm.AddResidualBlock(
            
            // they are the costfunc error term ptr, kernel func ptr;paramster to be estimated;
            new ceres::AutoDiffCostFunction<COST_FUNCTIONAL,1,3>(new COST_FUNCTIONAL(int(x_s[i]),y_s[i])),// 1 3 depends on the dim of cost output and param_esti, should be aligh with the defination in COST_FUNCTIONAL.
            nullptr,
            abc_esti
        );
        
        
        
    }
    
    
    //setting ceres solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary sum;
    
    
    //sovle ...
    ceres::Solve(options,&curve_fit_plm,&sum);
    cout<<sum.BriefReport()<<endl;
    
    for(auto data:abc_esti) cout<<data<<" ";
 

	return 0;
}
