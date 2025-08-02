#pragma once

class Perceptor{
public:
    Perceptor();
    ~Perceptor();

    // Using PSDK, get the distances to the nearest obstable along
    // different directions.
    double getForwardObstacleDistance();
    double getBackwardObstacleDistance();
    double getLeftObstacleDistance();
    double getRightObstacleDistance();
    double getDownObstacleDistance();
};
