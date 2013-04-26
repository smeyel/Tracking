#ifndef __RAY2D_H_
#define __RAY2D_H_
#include <opencv2/core/core.hpp>

using namespace cv;

/** Represents a 2D ray in an image */
class Ray2D
{
public:
	/** ID of the camera this ray is represented for. */
	int cameraID;

	/** Starting point */
	Point2f A;

	/** Next point (direction vector is given by B-A) */
	Point2f B;

	void show(char *msg);
};

#endif
