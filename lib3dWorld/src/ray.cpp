#include <iostream>
#include <opencv/cv.h>
#include "ray.h"

void Ray::show(char *msg)
{
	std::cout << msg << ": Ray("<<
		"A=("<<A.val[0]<<" "<<A.val[1]<<" "<<A.val[2]<<" "<<A.val[3]<<
		") B=("<<B.val[0]<<" "<<B.val[1]<<" "<<B.val[2]<<" "<<B.val[3]<<
		"), CamID="<<cameraID<<", origCamID="<<originalCameraID<<")"<<std::endl;
}

Matx41f Ray::getIntersection(Vector<Ray> rays, int firstRayIndex)
{
	// TODO: rendesen kezeli a homogen koordinatakat?
	// TODO: le van tesztelve?
	// TODO: nem ellenorzi, hogy az egyenesek metszespontja a sugarak pozitiv iranyaban van-e!
	int count = rays.size() - firstRayIndex;

	OPENCV_ASSERT(count >= 2, "Ray::getIntersection", "Cannot calculate intersection of less than 2 rays...");

	Matx33f sumN = Matx33f::zeros();
	Matx31f sumNa = Matx31f::zeros();
	Matx33f eye = Matx33f::eye();

	for(int c=0 ; c<count ; c++)
	{
		int i = c + firstRayIndex;
		Matx31f A(	rays[i].A(0,0),
					rays[i].A(1,0),
					rays[i].A(2,0));
		Matx31f B(	rays[i].B(0,0),
					rays[i].B(1,0),
					rays[i].B(2,0));
		Matx31f V = B-A;

		Matx31f a = A;
		float V_length = sqrt(V.dot(V));
		Matx31f v(	V(0,0) / V_length,
					V(1,0) / V_length,
					V(2,0) / V_length);

		Matx33f Ni = eye - v*v.t();
		sumN += Ni;
		sumNa += Ni*a;
	}

	Matx31f p = sumN.inv() * sumNa;

	Matx41f result( p(0,0), p(1,0), p(2,0), 1 );
	return result;
}


std::ostream& operator<<(std::ostream& stream, const Ray& ray)
{
	stream << "Ray("
		<< ray.A.val[0] << ";" << ray.A.val[1] << ";" << ray.A.val[2] << ";" << ray.A.val[3] <<
		")->("  
		<< ray.B.val[0] << ";" << ray.B.val[1] << ";" << ray.B.val[2] << ";" << ray.B.val[3] <<
		")";
    return stream;
 }