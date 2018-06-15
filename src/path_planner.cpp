#include <path_planner.hpp>

void PathPlanner::projectClosePointsOnPlane3D(){
    //get plane normal -> camera z axis
    plane_normal = this->cameraPose.block<3,1>(0,2);
    plane_normal / plane_normal.norm();

    //get explore point -> projection of camera center on the plane
    Eigen::Vector3d camPosition = this->cameraPose.block<3,1>(0,3);
    explorePoint3D = camPosition + this->distanceCameraToPlane * plane_normal / plane_normal.norm();

    // get plane d from [a,b,c,d]^\top
    double plane_d = -1*(explorePoint3D.transpose()*plane_normal)(0);

    //Get distances for all points
    auto nbPts = this->pcl.size();
    Eigen::VectorXd dist(nbPts);

    //get 3D projections of point cloud on the plane
    // a point is added to $pp$ if it is close enough to the plane

    Vector3DPoints pp;
	pp.clear();
	for (auto i = 0; i < nbPts; i++){
		dist(i) = (plane_normal.transpose() * pcl[i] + plane_d) / plane_normal.norm();
		if (fabs(dist(i)) < this->distanceCloseToPlane){
			pp.push_back(pcl[i] - plane_normal / plane_normal.norm() * dist(i));
		}
	}

    Eigen::MatrixXd ppmat(3,pp.size());
    for (auto i = 0; i < pp.size(); i++){
        ppmat.block<3,1>(0,i) = pp[i];
    }

    this->closePointsProjectedOnPlane3D = ppmat;
}

void PathPlanner::project3DPointsto2D(){
    Eigen::Vector3d b(0,0,1);
    Eigen::Vector3d v = plane_normal.cross(b);
    double c = plane_normal.dot(b);
    Eigen::Matrix3d vSkew;

    vectorToSkewSymm(v,vSkew);

    this->rotationMatrix = Eigen::Matrix3d::Identity() + vSkew + vSkew * vSkew / (1+c);

    explorePoint2D = rotationMatrix * explorePoint3D;
    closePointsProjectedOnPlane2D = rotationMatrix * closePointsProjectedOnPlane3D;
}

int PathPlanner::getNbPointsInsideCircle(
	const Eigen::Vector3d & point,
	const Eigen::MatrixXd & pcl,
	double circleRadius)
{
	std::cout << "Getting the number of points close to end-effector..." << std::endl;
	int nb = 0;

	for (auto i = 0; i < pcl.cols(); i++){
		Eigen::Vector3d diff;
		Eigen::Vector3d pointFromCloud = pcl.block<3,1>(0,i);
		diff = point - pointFromCloud;
		if (sqrt(diff.x()*diff.x() + diff.y()*diff.y()) < circleRadius){
			nb++;
		}
	}
	return nb;
}

void PathPlanner::estimateDesiredPoint2D(){
    // ----------------- Path planning -----------------------------------------
    // adapt explorePoint2D to be as far as needed from all other points..
    // explorePoint2D -> starting explore point in 3D with z-coord equal to zeros

    this->desiredPoint2D = this->explorePoint2D;
}

bool PathPlanner::isCollisionExpected(){
    int nbPts = getNbPointsInsideCircle(this->explorePoint2D, this->closePointsProjectedOnPlane2D, this->circleRadius);
    if (nbPts > 200){
        return true;
    }else{
        return false;
    }
}

Eigen::Vector3d PathPlanner::getDesiredPoint3D(){
    this->desiredPoint3D = this->rotationMatrix.transpose() * this->desiredPoint2D;
    return this->desiredPoint3D;
}

//-------------------------- SUPPLEMENTATY FUNCTIONS ---------------------------

void PathPlanner::vectorToSkewSymm(const Eigen::Vector3d v, Eigen::Matrix3d & m){
	m <<      0, -v.z(),  v.y(),
	      v.z(),      0, -v.x(),
		 -v.y(),  v.x(),      0;
}

void PathPlanner::removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}
