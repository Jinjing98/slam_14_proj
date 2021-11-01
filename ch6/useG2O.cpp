#include <iostream>
#include <opencv2/core/core.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <cmath>

#include <ceres/ceres.h>
 
using namespace std;
using namespace g2o;

//  define vertex and edge in the curve fitting problem


class cf_Vertex:public g2o::BaseVertex<3,Eigen::Vector3d>{// the dim of variable to be estimated and its datatype

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // rewrite virtual func
    //reset
    virtual void setToOriginImpl(){
        _estimate = Eigen::Vector3d(0,0,0);
    }
    //update
    virtual void oplusImpl ( const double* update ) {
        _estimate += Eigen::Vector3d(update);
//         cout<<"the update is :"<<Eigen::Vector3d(update)<<endl;
    }
    //read and write
    virtual bool read ( std::istream & is ){}
    virtual bool write ( std::ostream & os ) const {}  //  why the const is necessary here? or there will be error when compiling cf_Edge
    
    
    
};


 



class cf_Edge:public g2o::BaseUnaryEdge<1,double,cf_Vertex>{//dim of measurement, datatype of measurement, vertex type
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double _x;
    cf_Edge(double x):BaseUnaryEdge(),_x(x){}
    //compute error
    void computeError(){
        //access the _estimate by seeking for the connected vertex to the edge
        const cf_Vertex* v_ptr = static_cast<const cf_Vertex*>(_vertices[0]);
        const Eigen::Vector3d abc_esti = v_ptr->estimate();
        _error(0,0) = _measurement-std::exp(abc_esti[0]*_x*_x+abc_esti[1]*_x+abc_esti[2]);
    }
    virtual bool read ( std::istream & is ){}
    virtual bool write ( std::ostream & os )const {}
    
    
};
 
 




int main(int argc, char** argv){

        
    //y = exp(ax^2+bx+c)+w    data preparation  
    int num4pts = 100;
    double sigma = 1;
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
    
/*    
    //test
    Eigen::Vector3d a=Eigen::Vector3d(3,4,5);
    float update[3] = {1,2,3};
    cout<<Eigen::Vector3f(update)<<endl;
    */



//block
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> BlockSolverType;//  the dimension of varible to be optimized and the dimention of the error term.  cf: Jacob mat
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;    
 

    
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
// //optimizer -- graph
    g2o::SparseOptimizer optimizer_graph;      
    optimizer_graph.setAlgorithm(solver);
    optimizer_graph.setVerbose(true);




//put the values in vertex and edge
    
    cf_Vertex* v = new cf_Vertex();
    v->setEstimate(Eigen::Vector3d(1.1,0.1,0.1));  // init values; not start with 0,0,0.  or there will be numerical instability
    v->setId(0);
    optimizer_graph.addVertex(v);
    
    
    
    for(int i = 0; i<num4pts;i++){
        cf_Edge* e = new cf_Edge(x_s[i]);
        e->setMeasurement(y_s[i]);
        e->setId(i);
        e->setVertex(0,v);
        e->setInformation(Eigen::Matrix<double,1,1>::Identity()/(sigma*sigma));
        optimizer_graph.addEdge(e);
        
        
    }
    
 
    
    
    
    // start optimization_algorithm_dogleg
    optimizer_graph.initializeOptimization();
    optimizer_graph.optimize(1000);
    std::cout<<"the estimated abc are : "<<v->estimate()<<endl;
    
    
    
    
    
    
    
    
    
    
    
    
    
    return 0;
}
