#ifndef mathVec_H
#define mathVec_H

#include <memory.h>

#define SQUARE(number) (number*number)
#define PI_2 3.141593 
#define DEG_TO_RAD(deg) ((float)(deg)*PI_2/180)

class vector3f
{
public:

	double vertex[3];

	vector3f(){	memset(vertex, 0, sizeof(double[3]));	}
	vector3f(double x, double y, double z);
	~vector3f();

	void LoadZero(void);
	void Set(double x,double y,double z);
	double dist(vector3f v1, vector3f v2);

	vector3f& operator= (const vector3f &v1);
	friend bool operator== (const vector3f &v1, const vector3f &v2);
	friend vector3f operator+ (const vector3f &v1, const vector3f &v2);
	friend vector3f operator+ (const vector3f &v1, const double scalar);
	friend vector3f operator- (const vector3f &v1, const vector3f &v2);
	friend vector3f operator- (const vector3f &v1, const double scalar);
	friend vector3f operator* (const vector3f &v1, const vector3f &v2);
	friend vector3f operator* (const vector3f &v1, const double scalar);  

	friend double dot(const vector3f &v1, const vector3f &v2);
	friend vector3f Cross(const vector3f &v1, const vector3f &v2);

	
};


class matrix9f
{

	// matrix = [ 0  1  2
	//			  3  4  5
	//			  6  7  8]

public:
	double matrix[9];

	matrix9f()
	{	memset(matrix, 0, sizeof(double[9]));	}
	matrix9f(double x0, double x1, double x2, double x3, double x4, double x5, double x6, double x7, double x8);

	void LoadZero(void);
	void LoadIdentity(void);
	matrix9f Transpose(void);

	void RotateX(float deg);
	void RotateY(float deg);
	void RotateZ(float deg);


	matrix9f& operator= (const matrix9f &m1);
	friend matrix9f operator+ (const matrix9f &m1, const matrix9f &m2);
	friend matrix9f operator- (const matrix9f &m1, const matrix9f &m2);
	friend matrix9f operator* (const matrix9f &m1, const double scalar);
	friend matrix9f operator* (const matrix9f &m1, const matrix9f &m2);
	friend vector3f operator* (const matrix9f &m1, const vector3f &v2);
	friend bool operator== (const matrix9f &m1, const matrix9f &m2);
};




#endif

