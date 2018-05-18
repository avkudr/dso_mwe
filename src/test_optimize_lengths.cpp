#include <iostream>

#include "distances.h"
#include "concentric_tube_robot.h"

void loadPoints(VectorOfPoints2D & pts){
    pts.clear();

    // points for L = [0.1,0.1,0.1]
    pts.push_back(Eigen::Vector2d(     0,      0));
    pts.push_back(Eigen::Vector2d(0.0000, 0.0067));
    pts.push_back(Eigen::Vector2d(0.0002, 0.0133));
    pts.push_back(Eigen::Vector2d(0.0004, 0.0200));
    pts.push_back(Eigen::Vector2d(0.0008, 0.0267));
    pts.push_back(Eigen::Vector2d(0.0012, 0.0333));
    pts.push_back(Eigen::Vector2d(0.0018, 0.0399));
    pts.push_back(Eigen::Vector2d(0.0024, 0.0466));
    pts.push_back(Eigen::Vector2d(0.0032, 0.0532));
    pts.push_back(Eigen::Vector2d(0.0040, 0.0598));
    pts.push_back(Eigen::Vector2d(0.0049, 0.0664));
    pts.push_back(Eigen::Vector2d(0.0060, 0.0730));
    pts.push_back(Eigen::Vector2d(0.0071, 0.0796));
    pts.push_back(Eigen::Vector2d(0.0083, 0.0861));
    pts.push_back(Eigen::Vector2d(0.0096, 0.0927));
    pts.push_back(Eigen::Vector2d(0.0110, 0.0992));
    pts.push_back(Eigen::Vector2d(0.0110, 0.0992));
    pts.push_back(Eigen::Vector2d(0.0127, 0.1056));
    pts.push_back(Eigen::Vector2d(0.0146, 0.1120));
    pts.push_back(Eigen::Vector2d(0.0169, 0.1183));
    pts.push_back(Eigen::Vector2d(0.0195, 0.1244));
    pts.push_back(Eigen::Vector2d(0.0224, 0.1304));
    pts.push_back(Eigen::Vector2d(0.0256, 0.1363));
    pts.push_back(Eigen::Vector2d(0.0291, 0.1420));
    pts.push_back(Eigen::Vector2d(0.0328, 0.1475));
    pts.push_back(Eigen::Vector2d(0.0369, 0.1528));
    pts.push_back(Eigen::Vector2d(0.0412, 0.1578));
    pts.push_back(Eigen::Vector2d(0.0457, 0.1627));
    pts.push_back(Eigen::Vector2d(0.0505, 0.1673));
    pts.push_back(Eigen::Vector2d(0.0556, 0.1717));
    pts.push_back(Eigen::Vector2d(0.0608, 0.1758));
    pts.push_back(Eigen::Vector2d(0.0663, 0.1797));
    pts.push_back(Eigen::Vector2d(0.0663, 0.1797));
    pts.push_back(Eigen::Vector2d(0.0719, 0.1833));
    pts.push_back(Eigen::Vector2d(0.0777, 0.1865));
    pts.push_back(Eigen::Vector2d(0.0837, 0.1894));
    pts.push_back(Eigen::Vector2d(0.0899, 0.1918));
    pts.push_back(Eigen::Vector2d(0.0963, 0.1938));
    pts.push_back(Eigen::Vector2d(0.1027, 0.1954));
    pts.push_back(Eigen::Vector2d(0.1093, 0.1966));
    pts.push_back(Eigen::Vector2d(0.1159, 0.1974));
    pts.push_back(Eigen::Vector2d(0.1226, 0.1977));
    pts.push_back(Eigen::Vector2d(0.1292, 0.1976));
    pts.push_back(Eigen::Vector2d(0.1359, 0.1971));
    pts.push_back(Eigen::Vector2d(0.1425, 0.1961));
    pts.push_back(Eigen::Vector2d(0.1490, 0.1947));
    pts.push_back(Eigen::Vector2d(0.1554, 0.1929));
    pts.push_back(Eigen::Vector2d(0.1617, 0.1907));
}

int main(int argc, char **argv) {

    std::cout << "Test: optimize the lengths of segmetns of CTR for given trajectory points\n";
    ConcentricTubeRobot * robot = new ConcentricTubeRobot();

    std::vector<double> L = {0.1,0.1,0.1};

    VectorOfPoints2D pts;
    loadPoints(pts);

    robot->optimizeTubeLengthsForPoints(L,pts);

    return 0;
}
