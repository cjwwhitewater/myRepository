#include <cmath>
#include <string>
#include <sstream>
#include <fmt/core.h>
#include "DronePosition.h"

using namespace std;

DronePosition::DronePosition()
{
    longitude = 0;
    latitude  = 0;
    height    = 0;
}

bool DronePosition::isInvalid()
{
    // For our project, every component of the position can not be zero.
    return ( fabs(longitude) < 1e-6 ||
             fabs(latitude)  < 1e-6 ||
             fabs(height)    < 1e-6 );
}

string DronePosition::toString()
{
    ostringstream os;
    os << "{"
       << fmt::format("{:.6f}, {:.6f}, {:.1f}",
          longitude / M_PI * 180,
          latitude  / M_PI * 180,
          height)
       << "}";

    return os.str();
}
