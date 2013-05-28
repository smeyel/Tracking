#include <iostream>
#include "ray.h"

void Ray::show(char *msg)
{
	std::cout << msg << ": Ray("<<
		"A=("<<A.val[0]<<" "<<A.val[1]<<" "<<A.val[2]<<" "<<A.val[3]<<
		") B=("<<B.val[0]<<" "<<B.val[1]<<" "<<B.val[2]<<" "<<B.val[3]<<
		"), CamID="<<cameraID<<", origCamID="<<originalCameraID<<")"<<std::endl;
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