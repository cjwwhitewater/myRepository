#pragma once

class RTHController{
public:
    RTHController();

    // initiate the RTH action.
    void startReturnToHome();

    // This function will be peridically called to check whether the drone has
    // accomplished the RTH action and finally grounded.
    bool isDroneGrounded();
};
