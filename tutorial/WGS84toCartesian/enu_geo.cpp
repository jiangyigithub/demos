#include "enu_geo.hpp"

Geodetic_Type enu_to_geodetic(Enu_Type& enu, Geodetic_Type& geo) {
    Ecef_Type xyz = enu_to_ecef(enu, geo);
    return ecef_to_geodetic(xyz);
}

double radians(double angle) { return angle * pi / 180.0; }

Ecef_Type enu_to_ecef(Enu_Type& enu, Geodetic_Type& geo) {
    Ecef_Type ret;
    double lamb = radians(geo.lat);
    double phi = radians(geo.lon);
    double s = sin(lamb);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lamb);
    double cos_lambda = cos(lamb);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    double x0 = (geo.height + N) * cos_lambda * cos_phi;
    double y0 = (geo.height + N) * cos_lambda * sin_phi;
    double z0 = (geo.height + (1 - e_sq) * N) * sin_lambda;

    double t = cos_lambda * enu.zUp - sin_lambda * enu.yNorth;

    double zd = sin_lambda * enu.zUp + cos_lambda * enu.yNorth;
    double xd = cos_phi * t - sin_phi * enu.xEast;
    double yd = sin_phi * t + cos_phi * enu.xEast;

    ret.x = xd + x0;
    ret.y = yd + y0;
    ret.z = zd + z0;
    return ret;
}

Geodetic_Type ecef_to_geodetic(Ecef_Type& xyz) {
    Geodetic_Type ret;
    // Convert from ECEF cartesian coordinates to
    // latitude, longitude and height.  WGS-84
    double x2 = pow(xyz.x, 2);
    double y2 = pow(xyz.y, 2);
    double z2 = pow(xyz.z, 2);
    double e = sqrt(1 - pow((b / a), 2));
    double b2 = b * b;
    double e2 = pow(e, 2);
    double ep = e * (a / b);
    double r = sqrt(x2 + y2);
    double r2 = r * r;
    double E2 = pow(a, 2) - pow(b, 2);
    double F = 54 * b2 * z2;
    double G = r2 + (1 - e2) * z2 - e2 * E2;
    double c = (e2 * e2 * F * r2) / (G * G * G);
    double s = pow((1 + c + sqrt(c * c + 2 * c)), (1 / 3));
    double P = F / (3 * pow((s + 1 / s + 1), 2) * G * G);
    double Q = sqrt(1 + 2 * e2 * e2 * P);
    double ro =
        -(P * e2 * r) / (1 + Q) + sqrt((a * a / 2) * (1 + 1 / Q) - (P * (1 - e2) * z2) / (Q * (1 + Q)) - P * r2 / 2);
    double tmp = pow((r - e2 * ro), 2);
    double U = sqrt(tmp + z2);
    double V = sqrt(tmp + (1 - e2) * z2);
    double zo = (b2 * xyz.z) / (a * V);

    double height = U * (1 - b2 / (a * V));

    double lat = atan((xyz.z + ep * ep * zo) / r);

    double temp = atan(xyz.y / xyz.x);
    double long0 = temp;
    if (xyz.x >= 0) {
        long0 = temp;
    } else if ((xyz.x < 0) && (xyz.y >= 0)) {
        long0 = pi + temp;
    } else {
        long0 = temp - pi;
    }

    ret.lat = lat / (pi / 180);
    ret.lon = long0 / (pi / 180);
    ret.height = height;

    return ret;
}