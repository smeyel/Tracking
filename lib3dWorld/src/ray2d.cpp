#include <iostream>
#include "ray2d.h"

void Ray2D::show(char *msg)
{
	std::cout << msg << ": Ray2d(A="<<A.x<<" "<<A.y<<", B="<<B.x<<" "<<B.y<<")"<<std::endl;
}
