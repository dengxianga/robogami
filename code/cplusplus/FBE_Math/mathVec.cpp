#include <iostream>
#include <math.h>
#include "mathVec.h"

vector3f::vector3f(double x, double y, double z)
{
	vertex[0]=x;
	vertex[1]=y;
	vertex[2]=z;
}

vector3f::~vector3f()
{
}

void vector3f::Set(double x,double y, double z)
{
	vertex[0]=x;
	vertex[1]=y;
	vertex[2]=z;
}

void vector3f::LoadZero(void)
{
	vertex[0] = 0;
	vertex[1] = 0;
	vertex[2] = 0;

}

double vector3f::dist(vector3f v1, vector3f v2){
	double dx = vertex[0] - v1.vertex[0];
	double dy = vertex[1] - v1.vertex[1];
	double dz = vertex[2] - v1.vertex[2];

	double result = v2.vertex[0]*pow(dx,2) + v2.vertex[1]*pow(dy,2) +v2.vertex[2]*pow(dz,2) ; 
	return result;
}

vector3f& vector3f::operator=(const vector3f &v1)
{
	vertex[0] =v1.vertex[0];
	vertex[1] =v1.vertex[1];
	vertex[2] =v1.vertex[2];
	return *this;

}

bool operator== (const vector3f &v1, const vector3f &v2)
{
/*	return ((v1.vertex[0]/v2.vertex[0] > .95) && (v1.vertex[0]/v2.vertex[0] < 1.05) &&
			(v1.vertex[0]/v2.vertex[1] > .95) && (v1.vertex[0]/v2.vertex[1] < 1.05) &&
			(v1.vertex[0]/v2.vertex[2] > .95) && (v1.vertex[0]/v2.vertex[2] < 1.05)		);
*/
	return ((v1.vertex[0] == v2.vertex[0]) && 
		    (v1.vertex[1] == v2.vertex[1]) &&
		    (v1.vertex[2] == v2.vertex[2])); 
			
}

vector3f operator+ (const vector3f &v1, const vector3f &v2)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0] = v1.vertex[0] + v2.vertex[0];
	result.vertex[1] = v1.vertex[1] + v2.vertex[1];
	result.vertex[2] = v1.vertex[2] + v2.vertex[2];

	return result;
}


vector3f operator+ (const vector3f &v1, const double scalar)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0] = v1.vertex[0] + scalar;
	result.vertex[1] = v1.vertex[1] + scalar;
	result.vertex[2] = v1.vertex[2] + scalar;

	return result;
}

vector3f operator- (const vector3f &v1, const vector3f &v2)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0] = v1.vertex[0] - v2.vertex[0];
	result.vertex[1] = v1.vertex[1] - v2.vertex[1];
	result.vertex[2] = v1.vertex[2] - v2.vertex[2];

	return result;
}

vector3f operator- (const vector3f &v1, const double scalar)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0]= v1.vertex[0] - scalar;
	result.vertex[1]= v1.vertex[1] - scalar;
	result.vertex[2]= v1.vertex[2] - scalar;

	return result;
}


vector3f operator* (const vector3f &v1, const double scalar)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0]= v1.vertex[0] * scalar;
	result.vertex[1]= v1.vertex[1] * scalar;
	result.vertex[2]= v1.vertex[2] * scalar;

	return result;
}


vector3f operator* (const vector3f &v1, const vector3f &v2)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0]= v1.vertex[0] * v2.vertex[0];
	result.vertex[1]= v1.vertex[1] * v2.vertex[1];
	result.vertex[2]= v1.vertex[2] * v2.vertex[2];

	return result;
}


double dot(const vector3f &v1,const vector3f &v2)
{
	double dot;

	dot= (v1.vertex[0]*v2.vertex[0])+
		 (v1.vertex[1]*v2.vertex[1])+
		 (v1.vertex[2]*v2.vertex[2]);

	return dot;
}


vector3f Cross(const vector3f &v1, const vector3f &v2)
{
	vector3f result(0.0f, 0.0f, 0.0f);

	result.vertex[0]= (v1.vertex[1]*v2.vertex[2]) - (v1.vertex[2]*v2.vertex[1]);
	result.vertex[1]= (v1.vertex[2]*v2.vertex[0]) - (v1.vertex[0]*v2.vertex[2]);
	result.vertex[2]= (v1.vertex[0]*v2.vertex[1]) - (v1.vertex[1]*v2.vertex[0]);

	return result;
}


//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------

matrix9f::matrix9f(double x0, double x1, double x2, double x3, double x4, double x5, double x6, double x7, double x8)
{
	matrix[0]=x0;
	matrix[1]=x1;
	matrix[2]=x2;
	matrix[3]=x3;
	matrix[4]=x4;
	matrix[5]=x5;
	matrix[6]=x6;
	matrix[7]=x7;
	matrix[8]=x8;
}

void matrix9f::LoadZero(void)
{
	int loop;

	for(loop=0; loop<9; loop++)
		matrix[loop]=0.0f;
}

void matrix9f::LoadIdentity(void)
{
	matrix[0] = 1.0f; matrix[3] = 0.0f; matrix[6] = 0.0f; 
	matrix[1] = 0.0f; matrix[4] = 1.0f; matrix[7] = 0.0f; 
	matrix[2] = 0.0f; matrix[5] = 0.0f; matrix[8] = 1.0f; 

}

matrix9f matrix9f::Transpose(void)
{
	matrix9f result;

	result.matrix[0] = matrix[0]; result.matrix[3] = matrix[1]; result.matrix[6] = matrix[2]; 
	result.matrix[1] = matrix[3]; result.matrix[4] = matrix[4]; result.matrix[7] = matrix[5]; 
	result.matrix[2] = matrix[6]; result.matrix[5] = matrix[7]; result.matrix[8] = matrix[8];

	return result;
}


matrix9f& matrix9f::operator= (const matrix9f &m1)
{
	for (int i = 0; i<9; i++)	
			matrix[i]= m1.matrix[i];

	return *this;
}

matrix9f operator+ (const matrix9f &m1, const matrix9f &m2)
{
	matrix9f result;

	for (int i = 0; i<9; i++)
		result.matrix[i]= m1.matrix[i]+m2.matrix[i];

	return result;
}


matrix9f operator- (const matrix9f &m1, const matrix9f &m2)
{
	matrix9f result;

	for (int i = 0; i<9; i++)
		result.matrix[i]= m1.matrix[i]-m2.matrix[i];

	return result;
}


matrix9f operator* (const matrix9f &m1, const double scalar)
{
	matrix9f result;

	for (int i = 0; i<9; i++)
		result.matrix[i]= m1.matrix[i]*scalar;


	return result;
}


vector3f operator* (const matrix9f &m1, const vector3f &v2)
{
	vector3f result;

	result.vertex[0] = (m1.matrix[0]*v2.vertex[0]) + (m1.matrix[1]*v2.vertex[1]) + (m1.matrix[2]*v2.vertex[2]);
	result.vertex[1] = (m1.matrix[3]*v2.vertex[0]) + (m1.matrix[4]*v2.vertex[1]) + (m1.matrix[5]*v2.vertex[2]);
	result.vertex[2] = (m1.matrix[6]*v2.vertex[0]) + (m1.matrix[7]*v2.vertex[1]) + (m1.matrix[8]*v2.vertex[2]);

	return result;
}




matrix9f operator* (const matrix9f &m2, const matrix9f &m1)
{
	matrix9f result;

	// row 0
	result.matrix[0] = (m1.matrix[0]*m2.matrix[0]) + (m1.matrix[3]*m2.matrix[1]) + (m1.matrix[6]*m2.matrix[2]);
	result.matrix[1] = (m1.matrix[1]*m2.matrix[0]) + (m1.matrix[4]*m2.matrix[1]) + (m1.matrix[7]*m2.matrix[2]);
	result.matrix[2] = (m1.matrix[2]*m2.matrix[0]) + (m1.matrix[5]*m2.matrix[1]) + (m1.matrix[8]*m2.matrix[2]);
		
	// row 1
	result.matrix[3] = (m1.matrix[0]*m2.matrix[3]) + (m1.matrix[3]*m2.matrix[4]) + (m1.matrix[6]*m2.matrix[5]);
	result.matrix[4] = (m1.matrix[1]*m2.matrix[3]) + (m1.matrix[4]*m2.matrix[4]) + (m1.matrix[7]*m2.matrix[5]);
	result.matrix[5] = (m1.matrix[2]*m2.matrix[3]) + (m1.matrix[5]*m2.matrix[4]) + (m1.matrix[8]*m2.matrix[5]);

	// row 2
	result.matrix[6] = (m1.matrix[0]*m2.matrix[6]) + (m1.matrix[3]*m2.matrix[7]) + (m1.matrix[6]*m2.matrix[8]);
	result.matrix[7] = (m1.matrix[1]*m2.matrix[6]) + (m1.matrix[4]*m2.matrix[7]) + (m1.matrix[7]*m2.matrix[8]);
	result.matrix[8] = (m1.matrix[2]*m2.matrix[6]) + (m1.matrix[5]*m2.matrix[7]) + (m1.matrix[8]*m2.matrix[8]);


	return result;
}


bool operator== (const matrix9f &m1, const matrix9f &m2)
{
	bool result = true;
	for (int i = 0; i< 9; i++)
		if (m1.matrix[i] != m2.matrix[i]) result = false;

	return result;

}



void matrix9f::RotateX(float deg)
{
	matrix9f temp;
	temp.LoadIdentity();
        temp.matrix[4] = (float)cos(DEG_TO_RAD(deg));  temp.matrix[7]  = (float) -sin(DEG_TO_RAD(deg));
        temp.matrix[5] = (float)sin(DEG_TO_RAD(deg)); temp.matrix[8] = (float) cos(DEG_TO_RAD(deg));

        *this = (*this)*temp;
} 


// rotate about y-axis about center of this, not origin
// cos(x)	0	sin(x)	0 
// 0		1	 0		0
//-sin(x)	0	cos(x)	0  
// 0		0	 0		1 
void matrix9f::RotateY(float deg)
{
	matrix9f temp;

	temp.LoadIdentity();
        temp.matrix[0] = (float) cos(DEG_TO_RAD(deg)); temp.matrix[6]  = (float) sin(DEG_TO_RAD(deg));
        temp.matrix[2] = (float) -sin(DEG_TO_RAD(deg)); temp.matrix[8] = (float) cos(DEG_TO_RAD(deg));

        *this = (*this)*temp;
} 


// rotate about x-axis about center of this, not origin
// cos(x) -sin(x)	0	0 
// sin(x)  cos(x)	0	0  
// 0		0		1	0
// 0		0		0	1 
void matrix9f::RotateZ(float deg)
{
	matrix9f temp;
	temp.LoadIdentity();
        temp.matrix[0] = (float)cos(DEG_TO_RAD(deg));  temp.matrix[3] = (float)-sin(DEG_TO_RAD(deg));
        temp.matrix[1] = (float)sin(DEG_TO_RAD(deg)); temp.matrix[4] = (float) cos(DEG_TO_RAD(deg));

        *this = (*this)*temp;
} 