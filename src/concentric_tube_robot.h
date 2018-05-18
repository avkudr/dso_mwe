#ifndef CONCENTRICTUBEROBOT_H
#define CONCENTRICTUBEROBOT_H

#include <iostream>

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "distances.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


using VectorOfTransforms = std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > ;
using VectorOfPoints2D = std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > ;

class ConcentricTubeRobot
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConcentricTubeRobot();
    void updateRobotKinematics();
    void updateTransformMatrices();

    void setJointPos(const std::vector<double> &);

    void estimateTransformMatrices(VectorOfTransforms & T, std::vector<double> * L = nullptr);
    void optimizeTubeLengthsForPoints(const VectorOfPoints2D &, std::vector<double> &);
    double distancePoints2DRobot(std::vector<double> & L, const Eigen::Vector2d & pts);
    std::vector<double> getResultingCurvature() const {return _K;}
    VectorOfTransforms _T; //transformation matrices

private:
    int _nbTubes;
    double _E; // Modulus of elasticity (Young's modulus)
    std::vector<double> _radiiInt; // internal radii of tubes
    std::vector<double> _radiiExt; // external radii of tubes
    std::vector<double> _I; // inertia moment
    std::vector<double> _tubeKappa; // 1/m - circular precurvatures
    std::vector<double> _tubeLength; // m - lengths of curved parts
    std::vector<double> _alpha; // joint rotations
    std::vector<double> _rho; // joint translations

    std::vector<double> _K; // Deformed resultant curvature
    std::vector<double> _P; // Phi, rotation angle around z
    std::vector<double> _L; // Section's final lengths due to the translation rho

};

//----------------------------------------------------------------------------------------
#include <unsupported/Eigen/NonLinearOptimization>


// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
};


struct MyFunctor : Functor<double>
{
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
    {
        std::vector<double> L(3);
        L[0] = x(0);
        L[1] = x(1);
        L[2] = x(2);
        VectorOfTransforms T;
        robot->estimateTransformMatrices(T,&L);
        //construct arcs
        //inline double distancePointArc2D( const Vector2d & p, const Vector2d & center, const Vector2d & arcStartPt, const Vector2d & arcEndPt)
        Eigen::Vector2d  origin(T[0](0,3),T[0](2,3)); // x and z from transform matrix
        Eigen::Vector2d arc1end(T[1](0,3),T[1](2,3)); // x and z from transform matrix
        Eigen::Vector2d arc2end(T[2](0,3),T[2](2,3)); // x and z from transform matrix
        Eigen::Vector2d arc3end(T[3](0,3),T[3](2,3)); // x and z from transform matrix

        double r1,r2,r3;
        std::vector<double> K = robot->getResultingCurvature();
        r1 = 1.0 / K[0];
        r2 = 1.0 / K[1];
        r3 = 1.0 / K[2];

        Eigen::Vector2d arc1center, arc2center,arc3center;
        Eigen::Vector3d temp;
        temp = T[1] * Eigen::Vector3d(r1,0,0);
        arc1center(0) = temp(0);
        arc1center(1) = temp(2);
        temp = T[2] * Eigen::Vector3d(r2,0,0);
        arc2center(0) = temp(0);
        arc2center(1) = temp(2);
        temp = T[3] * Eigen::Vector3d(r3,0,0);
        arc3center(0) = temp(0);
        arc3center(1) = temp(2);

        for(auto i = 0; i < this->points.size(); i++)
        {
            //fvec(i) = robot->distancePoints2DRobot(L,this->points[i]);
            double d1 = distancePointArc2D(points[i],arc1center, origin,arc1end);
            double d2 = distancePointArc2D(points[i],arc2center,arc1end,arc2end);
            double d3 = distancePointArc2D(points[i],arc3center,arc2end,arc3end);
            double d  = std::min(d1,std::min(d2,d3));
            // add some confidence region to d
            fvec(i) = d;
        }
        fvec(points.size()) = 100 * distancePointPoint(this->points.back(),arc3end);
        //the factor of X translate the fact that this constraint is more important than others

        //fvec(points.size()) = fvec(points.size()) * fvec(points.size());
        return 0;
    }

    VectorOfPoints2D points;
    ConcentricTubeRobot * robot;

    int inputs() const { return 3; } // There are three parameters of the model
    int values() const { return this->points.size() + 1; } // The number of observations
};

using MyFunctorNumericalDiff = Eigen::NumericalDiff<MyFunctor>;
//----------------------------------------------------------------------------------


#endif // CONCENTRICTUBEROBOT_H
