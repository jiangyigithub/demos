#include <cmath>

struct Geodetic_Type {
    double lat;
    double lon;
    double height;
};

struct Ecef_Type {
    double x;
    double y;
    double z;
};

struct Enu_Type {
    double xEast;
    double yNorth;
    double zUp;
};

const double a = 6378137.0000;  // earth radius in meters
const double b = 6356752.3142;  // earth semiminor in meters
const double f = (a - b) / a;
const double e_sq = f * (2 - f);
const double pi = 3.14159265359;

double radians(double angle);

Ecef_Type enu_to_ecef(Enu_Type& enu, Geodetic_Type& geo);

Geodetic_Type ecef_to_geodetic(Ecef_Type& xyz);

// -----------------------------------------------------------
// Geodetic_Type geo = enu_to_geodetic(enu, ref_point_);
// -----------------------------------------------------------
/**
 * param: enu - ENU type point
 * param: geo - relative point
 **/
Geodetic_Type enu_to_geodetic(Enu_Type& enu, Geodetic_Type& geo);