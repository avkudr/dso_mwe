#ifndef DISTANCES_H
#define DISTANCES_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

using Eigen::Vector2d;
using Eigen::VectorXd;

inline double distancePointPoint( const VectorXd & p1, const VectorXd & p2){
    assert( p1.size() == p2.size() );
    VectorXd v = p1 - p2;
    return v.norm();
}

inline double distancePointLine( const VectorXd & pt, const VectorXd & ptLine1, const VectorXd & ptLine2){
    assert(      pt.size() == ptLine1.size() );
    assert( ptLine1.size() == ptLine2.size() );

    VectorXd u = ptLine2 - ptLine1;
    VectorXd v =      pt - ptLine1;

    VectorXd ptProjection = ptLine1 + u.dot(v) / u.squaredNorm() * u ;
    return (pt - ptProjection).norm();
}

inline double distancePointLineSegment( const VectorXd & pt, const VectorXd & ptLine1, const VectorXd & ptLine2){
    assert(      pt.size() == ptLine1.size() );
    assert( ptLine1.size() == ptLine2.size() );

    VectorXd u = ptLine2 - ptLine1;
    VectorXd v =      pt - ptLine1;

    double temp = u.dot(v) / u.squaredNorm() ;
    if (temp < 0){ // pt belongs to [-inf;ptLine1]
        return distancePointPoint(pt,ptLine1);
    }
    else if (temp > 1){ // pt belongs to [ptLine2; +inf]
        return distancePointPoint(pt,ptLine2);
    }
    else{ // pt belongs to [ptLine1;ptLine2]
        return distancePointLine(pt,ptLine1,ptLine2);
    }
}

inline double distancePointCircle2D( const Vector2d & p1, const Vector2d & center, double r){
    VectorXd v = center - p1;
    return abs(v.norm()-r);
}

inline bool areClockwise(const Vector2d & v1, const Vector2d & v2) {
  return -v1(0)*v2(1) + v1(1)*v2(0) > 0;
}

inline double distancePointArc2D( const Vector2d & p, const Vector2d & center, const Vector2d & arcStartPt, const Vector2d & arcEndPt){

    Vector2d v1 = arcStartPt - center;
    Vector2d v2 = arcEndPt - center;
    Vector2d vp = p - center;

    double r = (arcStartPt - center).norm();
    bool insideSector;
    insideSector = areClockwise(v2, v1) && (!areClockwise(v1, vp) && areClockwise(v2, vp));
    insideSector = insideSector || (areClockwise(v1, v2) && areClockwise(v1, vp) && !areClockwise(v2, vp));
    if (insideSector){ // inside sector
        return distancePointCircle2D(p,center,r);
    }
    else{
        double d1 = distancePointPoint(p,arcStartPt);
        double d2 = distancePointPoint(p,arcEndPt);
        double d3 = vp.norm() + r;
        double res = std::min(d1,d2);
        return std::min(res,d3);
    }
}

#endif // ARR_H
