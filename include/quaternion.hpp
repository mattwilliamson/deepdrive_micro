#ifndef QUATERNION_HPP
#define QUATERNION_HPP

class Quaternion {
public:
    double w;
    double x;
    double y;
    double z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double roll, double pitch, double yaw);
    Quaternion(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    void normalize();

};

#endif // QUATERNION_HPP