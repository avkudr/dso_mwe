#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>

#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

using Vector3DPoints = std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>;
using Vector2DPoints = std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>>;

class PathPlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PathPlanner(){
        this->circleRadius          = 0.1;
        this->distanceCameraToPlane = 1.0; //drawPlaneInFront
        this->distanceCloseToPlane  = 0.15;
    }
    ~PathPlanner(){}

    void setCircleRadius(double r){
        this->circleRadius = r;
    }
    void init(Vector3DPoints pcl, const Eigen::Matrix4d & cameraPose){
        this->pcl = pcl;
        this->cameraPose = cameraPose;
    }
    void projectClosePointsOnPlane3D();
    void project3DPointsto2D();
    void estimateDesiredPoint2D();
    Eigen::Vector3d getDesiredPoint3D();

    double getCircleRadius() const{return circleRadius;};
    Eigen::MatrixXd getClosePointsProjectedOnPlane2D(){
        return closePointsProjectedOnPlane2D;
    }
    Eigen::Vector3d getDesiredPoint2D(){
        return this->desiredPoint2D;
    }
    Eigen::Vector3d getExplorePoint2D(){
        return this->explorePoint2D;
    }
    bool isCollisionExpected();
    double getRepulsiveForceZ();

private:
    int getNbPointsInsideCircle(
    	const Eigen::Vector3d & point,
    	const Eigen::MatrixXd & pcl,
    	double circleRadius);
    void vectorToSkewSymm(const Eigen::Vector3d v, Eigen::Matrix3d & m);
    void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);

    //parameters
    double circleRadius; //m
    double distanceCameraToPlane; //m
    double distanceCloseToPlane; //m

    //from SLAM
    Vector3DPoints pcl;
    Eigen::Matrix4d cameraPose;

    //variables
    Eigen::MatrixXd closePointsProjectedOnPlane3D;
    Eigen::MatrixXd closePointsProjectedOnPlane2D;
    Eigen::Vector3d explorePoint3D;
    Eigen::Vector3d explorePoint2D;
    Eigen::Vector3d desiredPoint2D;
    Eigen::Vector3d desiredPoint3D;

    Eigen::Vector3d plane_normal;
    double plane_d;
    Eigen::Matrix3d rotationMatrix;
};

#endif //PATHPLANNER_H
