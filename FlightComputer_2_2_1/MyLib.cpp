
/*
 *  My Library of commonly used classes and functions
 */

#include "MyLib.h"
#include "arduino.h"



/***********
 * #########
 * Functions
 * #########
 ***********
 */
 
/**************
 * Update Timer
 **************
 */
bool updateTimer( long *timer, double *dt, int delayTime ){
  if( *timer > millis() ){
    *timer = millis();
    return false;
  }
  
  *dt = (double)(millis()-(*timer));
  if( *dt >= delayTime ){
    *dt /= 1000.0d;
    return true;
  }
  
  return false;
}
/**********
 * In Range
 **********
 */
bool inRange( double val, double lower, double upper ){
  if( val >= lower && val < upper ){ return true; }
  return false;
}
/***********
 * Magnitude
 ***********
 */
double magnitude( double x, double y, double z ){ return pow( x*x + y*y + z*z , 0.5 ); }

/******
 * Sign
 ******
 */
int sign( long n ){
  if(n<0){return -1;}
  return 1;
}
int sign( double n ){
  if(n<0){return -1;}
  return 1;
}





/********************
 * ##################
 * ##### Vector #####
 * ##################
 ********************
 */
Vector::Vector(double x, double y, double z) {
  this->set( x, y, z );
}
Vector::Vector() {
  this->set( 0,0,0 );
}
const Vector Vector::operator+(const Vector& v) const{
  Vector r(this->x, this->y, this->z);
  r.x += v.x;
  r.y += v.y;
  r.z += v.z;
  return r;
}
const Vector Vector::operator-(const Vector& v) const{
  Vector r(this->x, this->y, this->z);
  r.x -= v.x;
  r.y -= v.y;
  r.z -= v.z;
  return r;
}
const Vector Vector::operator*(const double& s) const{
  Vector r(this->x, this->y, this->z);
  r.x *= s;
  r.y *= s;
  r.z *= s;
  return r;
}
const Vector Vector::operator/(const double& s) const{
  Vector r(this->x, this->y, this->z);
  r.x /= s;
  r.y /= s;
  r.z /= s;
  return r;
}
Vector& Vector::operator=(const Vector& v){
  this->x = v.x;
  this->y = v.y;
  this->z = v.z;
  return *this;
}
Vector& Vector::operator+=(const Vector& v){
  this->x += v.x;
  this->y += v.y;
  this->z += v.z;
  return *this;
}

Vector& Vector::operator-=(const Vector& v){
  this->x -= v.x;
  this->y -= v.y;
  this->z -= v.z;
  return *this;
}
Vector& Vector::operator*=(const double& s){
  this->x *= s;
  this->y *= s;
  this->z *= s;
  return *this;
}
Vector& Vector::operator/=(const double& s){
  this->x /= s;
  this->y /= s;
  this->z /= s;
  return *this;
}
void Vector::print(){
  Serial.print("<");
  Serial.print(this->x);
  Serial.print(",");
  Serial.print(this->y);
  Serial.print(",");
  Serial.print(this->z);
  Serial.print(">");
}
void Vector::printCSV(){
  Serial.print(this->x);
  Serial.print(",");
  Serial.print(this->y);
  Serial.print(",");
  Serial.print(this->z);
}
void Vector::normalize() {
  *this / this->length();
}
double Vector::length() {
  return pow( (this->x)*(this->x) + (this->y)*(this->y) + (this->z)*(this->z) , 0.5 );
}
void Vector::set( double x, double y, double z ) {
  this->x = x;
  this->y = y;
  this->z = z;
}
void Vector::copy( Vector v ) {
  this->set( v.x, v.y, v.x );
}
Vector Vector::clone() {
  Vector c( this->x, this->y, this->z );
  return c;
}
Vector Vector::cross( Vector v ) {
  Vector r;
  r.x = this->y * v.z - this->z * v.y;
  r.y = this->z * v.x - this->x * v.z;
  r.z = this->x * v.y - this->y * v.x;
  return r;
}
double Vector::dot( Vector v ) {
  return (this->x)*v.x + (this->y)*v.y + (this->z*v.z);
}
double Vector::angleTo( Vector v ){
  return acos( this->dot(v) / ( this->length() * v.length()  ) );
}






/************************
 * ######################
 * ##### Quaternion #####
 * ######################
 ************************
 */
Quaternion::Quaternion(double theta, Vector v) {
  this->w = cos(theta / 2);
  double s = sin(theta / 2);
  this->x = v.x * s;
  this->y = v.y * s;
  this->z = v.z * s;
}
Quaternion::Quaternion( Vector v) {
  this->w = 0;
  this->x = v.x;
  this->y = v.y;
  this->z = v.z;
}
Quaternion::Quaternion( double w, double x, double y, double z ){
  this->set( w, x, y, z );
}
Quaternion::Quaternion(){
  this->set( 1, 0, 0, 0 );
}
void Quaternion::set( double w, double x, double y, double z  ){
  this->w = w;
  this->x = x;
  this->y = y;
  this->z = z;
}
const Quaternion Quaternion::operator*(const Quaternion& q) const{
  Quaternion r;

  r.w = q.w*(this->w) - q.x*(this->x) - q.y*(this->y) - q.z*(this->z);
  r.x = q.w*(this->x) + q.x*(this->w) - q.y*(this->z) + q.z*(this->y);
  r.y = q.w*(this->y) + q.x*(this->z) + q.y*(this->w) - q.z*(this->x);
  r.z = q.w*(this->z) - q.x*(this->y) + q.y*(this->x) + q.z*(this->w);
  
  return r;
}
const Quaternion Quaternion::operator+(const Quaternion& q) const{
  Quaternion r;

  r.w += q.w;
  r.x += q.x;
  r.y += q.y;
  r.z += q.z;

  return r;
}
const Quaternion Quaternion::operator-(const Quaternion& q) const{
  Quaternion r;

  r.w -= q.w;
  r.x -= q.x;
  r.y -= q.y;
  r.z -= q.z;

  return r;
}

const Quaternion Quaternion::operator*(const double& s) const{
  Quaternion r( (this->w)*s, (this->x)*s, (this->y)*s, (this->z)*s );
  return r;
}
const Quaternion Quaternion::operator/(const double& s) const{
  Quaternion r( (this->w)/s, (this->x)/s, (this->y)/s, (this->z)/s );
  return r;
}

Quaternion& Quaternion::operator+=(const Quaternion& q){
  this->w += q.w;
  this->x += q.x;
  this->y += q.y;
  this->z += q.z;
  return *this;
}
Quaternion& Quaternion::operator-=(const Quaternion& q){
  this->w -= q.w;
  this->x -= q.x;
  this->y -= q.y;
  this->z -= q.z;
  return *this;
}
Quaternion& Quaternion::operator*=(const Quaternion& q){
  double w = q.w*(this->w) - q.x*(this->x) - q.y*(this->y) - q.z*(this->z);
  double x = q.w*(this->x) + q.x*(this->w) - q.y*(this->z) + q.z*(this->y);
  double y = q.w*(this->y) + q.x*(this->z) + q.y*(this->w) - q.z*(this->x);
  double z = q.w*(this->z) - q.x*(this->y) + q.y*(this->x) + q.z*(this->w);
  this->w = w;
  this->x = x;
  this->y = y;
  this->z = z;
  return *this;
}

Quaternion& Quaternion::operator*=(const double& s){
  this->w *= s;
  this->x *= s;
  this->y *= s;
  this->z *= s;
  return *this;
}
Quaternion& Quaternion::operator/=(const double& s){
  this->w /= s;
  this->x /= s;
  this->y /= s;
  this->z /= s;
  return *this;
}
Quaternion Quaternion::clone() {
  Quaternion r( this->w, this->x, this->y, this->z );
  return r;
}
void Quaternion::copy( Quaternion q ) {
  this->set( q.w, q.x, q.y, q.z );
}
void Quaternion::normalize() {
  *this /= this->length();
}
double Quaternion::length(){
  return pow( (this->w)*(this->w) + (this->x)*(this->x) + (this->y)*(this->y) + (this->z)*(this->z) , 0.5 );
}
void Quaternion::rotatePoint( Vector* point ){
  Vector axis( this->x, this->y, this->z );
  Vector p;
  p.set(point->x, point->y, point->z);
  Vector c = axis.cross(p);

  //Serial.println(p.z);

  p += c*2*(this->w);
  p += axis.cross(c)*2;

  *point = p;
  
}


