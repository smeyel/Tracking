#ifndef __RAY_H_
#define __RAY_H_
#include <opencv2/core/core.hpp>

using namespace cv;

/** Representas a ray in 3D.
	This may have been an image point in one of the cameras,
	now represented in the 3D world coordinate system as a ray.
*/
class Ray
{
public:
	/** ID of the camera the ray is represented in.
		CAMID_WORLD indicates world coordinates.
		Warning: later if a camera gets a ray with with its own ID,
			transformation can be omitted only if the camera is stationary!
	*/
	int cameraID;

	/** Homog. coordinates of the origin position (like the camera focal point) (4x1 CV_64F matrix) */
	Matx41f A;
	/** Coordinates of the next point (orientation vector is given by B-A) */
	Matx41f B;
	
	/** ID of the source camera */
	int originalCameraID;
	/** Original image coordinates in the original cameras image coordinate system */
	Point2f originalImageLocation;

	void show(char *msg);
};

#endif
