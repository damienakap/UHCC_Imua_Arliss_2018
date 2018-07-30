
/*
 *  My Library of commonly used classes and functions
 */

#ifndef MYLIB_H
#define MYLIB_H


/***********
 * #########
 * Functions
 * #########
 ***********
 */
extern bool updateTimer( long *timer, double *dt, int delayTime );
extern bool inRange( double a, double lower, double uppper );
extern int sign( long n );
extern int sign( double n );
extern double magnitude( double x, double y, double z );

/********************
 * ##################
 * ##### Vector #####
 * ##################
 ********************
 */
class Vector{
  public:
    double x,y,z;
    Vector(double x, double y, double z);
    Vector();
    const Vector operator+(const Vector& v) const;
    const Vector operator-(const Vector& v) const;
    const Vector operator*(const double& s) const;
    const Vector operator/(const double& s) const;
    
    Vector& operator=(const Vector& v);
    Vector& operator+=(const Vector& v);
    Vector& operator-=(const Vector& v);
    Vector& operator*=(const double& s);
    Vector& operator/=(const double& s);

    void print();
    void printCSV();
    void normalize();
    void set( double x, double y, double z );
    void copy( Vector v );
    double length();
    double dot( Vector v );
    Vector clone();
    Vector cross( Vector v );
    double angleTo( Vector v );
};






/************************
 * ######################
 * ##### Quaternion #####
 * ######################
 ************************
 */
class Quaternion{
  public:
    double w,x,y,z;
    Quaternion();
    Quaternion(double theta, Vector v);
    Quaternion(Vector v);
    Quaternion(double w, double x, double y, double z );

    const Quaternion operator*(const Quaternion& q) const;
    const Quaternion operator+(const Quaternion& q) const;
    const Quaternion operator-(const Quaternion& q) const;
    const Quaternion operator*(const double& s) const;
    const Quaternion operator/(const double& s) const;

    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator-=(const Quaternion& q);
    Quaternion& operator*=(const Quaternion& q);
    
    Quaternion& operator*=(const double& s);
    Quaternion& operator/=(const double& s);
    
    void set(double w, double x, double y, double z );
    Quaternion clone();
    void rotatePoint( Vector* p );
    void copy( Quaternion q );
    void normalize();
    double length();
};

#endif
