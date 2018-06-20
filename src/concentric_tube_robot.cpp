#include "concentric_tube_robot.h"

ConcentricTubeRobot::ConcentricTubeRobot()
{
    _nbTubes = 3;
    _E = 80e9;

    _radiiInt = {
        (1.100e-3)/2,
        (0.770e-3)/2,
        (0.414e-3)/2
    };
    _radiiExt = { // external
        (1.600e-3)/2,
        (1.010e-3)/2,
        (0.640e-3)/2
    };

    for(auto i = 0; i < _radiiInt.size(); i++){
        _I.push_back( M_PI * ( pow(_radiiExt[i],4) - pow(_radiiInt[i],4)) / 4.0 );
    }

    _tubeKappa  = {-6.503325,70.033717802220,0}; //9.76
    _tubeLength = {40e-3,155e-3,200e-3};

    _alpha = {0,0,0};
    _rho = {0,0,0};

    _distanceThreshold = 0.002; // 2mm -> for optimization

    updateRobotKinematics();
}

void ConcentricTubeRobot::updateRobotKinematics()
{
    auto Cte2 = _E*_I[1] + _E*_I[2];
    auto Cte1 = _E*_I[0] + Cte2;

    auto num2cos = (_E*_I[1]*_tubeKappa[1]*cos(_alpha[1])+
                    _E*_I[2]*_tubeKappa[2]*cos(_alpha[2]));
    auto num2sin = (_E*_I[1]*_tubeKappa[1]*sin(_alpha[1])+
                    _E*_I[2]*_tubeKappa[2]*sin(_alpha[2]));

    // Deformed resultant curvature components
    auto ksi1   = (_E*_I[0]*_tubeKappa[0]*cos(_alpha[0]) + num2cos) / Cte1;
    auto gamma1 = (_E*_I[0]*_tubeKappa[0]*sin(_alpha[0]) + num2sin) / Cte1;
    auto ksi2   = num2cos / Cte2;
    auto gamma2 = num2sin / Cte2;

    _K.resize(_nbTubes);
    _K[0] = sqrt(ksi1*ksi1 + gamma1*gamma1);
    _K[1] = sqrt(ksi2*ksi2 + gamma2*gamma2);
    _K[2] = _tubeKappa[2];

    _P.resize(_nbTubes);
    _P[0] = atan2(gamma1,ksi1);
    _P[1] = atan2(gamma2,ksi2) - _P[0];
    _P[2] = _alpha[2] - _P[1] - _P[0];

    _L.resize(_nbTubes);
    _L[0] = _tubeLength[0] + _rho[0];
    _L[1] = _tubeLength[1] + _rho[1] - _L[0];
    _L[2] = _tubeLength[2] + _rho[2] - _L[1] - _L[0];

    // Transformation matrices
    _T.resize( _nbTubes + 1);
    updateTransformMatrices();
}

void ConcentricTubeRobot::updateTransformMatrices()
{
    estimateTransformMatrices(_T);
}

void ConcentricTubeRobot::estimateTransformMatrices(VectorOfTransforms &T, std::vector<double> * Lpointer)
{
    if (Lpointer == nullptr) Lpointer = &_L;
    auto L = *Lpointer;
    assert(L.size() == _nbTubes);

    if (T.size() != _nbTubes + 1) T.resize(_nbTubes + 1);

    for(int i = 0; i < _nbTubes; i++){
        if (_K[i] < 0.001){
            T[i+1](0,0) = cos(_P[i]);
            T[i+1](0,1) = -sin(_P[i]);
            T[i+1](0,2) = 0;
            T[i+1](0,3) = 0;

            T[i+1](1,0) = sin(_P[i]);
            T[i+1](1,1) = cos(_P[i]);
            T[i+1](1,2) = 0;
            T[i+1](1,3) = 0;

            T[i+1](2,0) = 0;
            T[i+1](2,1) = 0;
            T[i+1](2,2) = 1;
            T[i+1](2,3) = L[i];
        }else{
            T[i+1](0,0) = cos(_P[i])*cos(_K[i]*L[i]);
            T[i+1](0,1) = -sin(_P[i]);
            T[i+1](0,2) = cos(_P[i])*sin(_K[i]*L[i]);
            T[i+1](0,3) = cos(_P[i])*(1-cos(_K[i]*L[i]))/_K[i];

            T[i+1](1,0) = sin(_P[i])*cos(_K[i]*L[i]);
            T[i+1](1,1) = cos(_P[i]);
            T[i+1](1,2) = sin(_P[i])*sin(_K[i]*L[i]);
            T[i+1](1,3) = sin(_P[i])*(1-cos(_K[i]*L[i]))/_K[i];

            T[i+1](2,0) = -sin(_K[i]*L[i]);
            T[i+1](2,1) = 0;
            T[i+1](2,2) = cos(_K[i]*L[i]);
            T[i+1](2,3) = sin(_K[i]*L[i])/_K[i];
        }
    }

    T[0].setIdentity();
    T[1] = T[0] * T[1]; // end of tube 1
    T[2] = T[1] * T[2]; // end of tube 2
    T[3] = T[2] * T[3]; // end of tube 3 - tool position

//    std::cout << " TO\n" << T[0].matrix() << std::endl;
//    std::cout << " T1\n" << T[1].matrix() << std::endl;
//    std::cout << " T2\n" << T[2].matrix() << std::endl;
    //    std::cout << " T3\n" << T[3].matrix() << std::endl;
}

void ConcentricTubeRobot::optimizeTubeLengthsForPoints(const VectorOfPoints2D &pts, std::vector<double> & newRho)
{
    std::cout << "Optimizing segment lengths..." << std::endl;

    Eigen::VectorXd x(_nbTubes);
    for (auto i = 0; i < _nbTubes; i++)
        x(i) = _L[i];

    MyFunctorNumericalDiff functor;
    functor.points = pts;
    functor.robot = this;
    Eigen::LevenbergMarquardt<MyFunctorNumericalDiff> lm(functor);
//    lm.parameters.maxfev = 1000;
//    lm.parameters.epsfcn = 1.0e-10;
//    lm.parameters.xtol = 1.0e-10;
//    lm.parameters.ftol = 1.0e-10;

    std::cout << "initial x: " << x.transpose() << std::endl;
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
    //std::cout << "status: " << status << std::endl;
    std::cout << "x that minimizes the function: " << x.transpose() << std::endl;

    for (auto i = 0; i < _nbTubes; i++)
        _L[i] = x(i);

    newRho.clear();
    newRho.push_back(0);
    newRho.push_back(0);
    newRho.push_back(0);
    newRho.push_back((_L[0] - _tubeLength[0])/1); //to meters
    newRho.push_back((_L[1] - _tubeLength[1] + _L[0])/1);
    newRho.push_back((_L[2] - _tubeLength[2] + _L[1] + _L[0])/1);
}


void ConcentricTubeRobot::setJointPos(const std::vector<double> & q){
    _alpha = {q[0],q[1],q[2]};
    _rho = {q[3],q[4],q[5]};
    updateRobotKinematics();
}

//double ConcentricTubeRobot::distancePoints2DRobot(std::vector<double> &L, const Eigen::Vector2d &pt)
//{
//    VectorOfTransforms T;
//    estimateTransformMatrices(T,&L);
//    //construct arcs
//    //inline double distancePointArc2D( const Vector2d & p, const Vector2d & center, const Vector2d & arcStartPt, const Vector2d & arcEndPt)
//    Eigen::Vector2d  origin(T[0](0,3),T[0](2,3)); // x and z from transform matrix
//    Eigen::Vector2d arc1end(T[1](0,3),T[1](2,3)); // x and z from transform matrix
//    Eigen::Vector2d arc2end(T[2](0,3),T[2](2,3)); // x and z from transform matrix
//    Eigen::Vector2d arc3end(T[3](0,3),T[3](2,3)); // x and z from transform matrix

//    double r1,r2,r3;
//    r1 = 1.0 / _K[0];
//    r2 = 1.0 / _K[1];
//    r3 = 1.0 / _K[2];

//    Eigen::Vector2d arc1center, arc2center,arc3center;
//    Eigen::Vector3d temp;
//    temp = T[1] * Eigen::Vector3d(r1,0,0);
//    arc1center(0) = temp(0);
//    arc1center(1) = temp(2);
//    temp = T[2] * Eigen::Vector3d(r2,0,0);
//    arc2center(0) = temp(0);
//    arc2center(1) = temp(2);
//    temp = T[3] * Eigen::Vector3d(r3,0,0);
//    arc3center(0) = temp(0);
//    arc3center(1) = temp(2);

//    double d1 = distancePointArc2D(pt,arc1center, origin,arc1end);
//    double d2 = distancePointArc2D(pt,arc2center,arc1end,arc2end);
//    double d3 = distancePointArc2D(pt,arc3center,arc2end,arc3end);
//    return std::min(d1,std::min(d2,d3));
//}
