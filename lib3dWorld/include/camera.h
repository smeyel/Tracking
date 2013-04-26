#ifndef __CAMERA_H_
#define __CAMERA_H_
#include "ray.h"
#include "ray2d.h"

using namespace cv;

/** Identifier of the camera representing the world coordinate system. */
#define CAMID_WORLD -1

/** Represents a camera (or coordinate system in general).
	This class is intended to be used as the main tool for transformations between coordinate systems.

	For setup, use loadCalibrationData to load intrinsic parameters from file. Then use calculateExtrinsicParams
	to calibrate extrinsic parameters.

	Wich stationary cameras, you may want to use calculateExtrinsicParamsIfNeeded. While using this, you can still
	change isStationary to indicate that the camera may have been moved since the last call.
*/
class Camera
{
	/** Camera matrix, intrinsic parameters */
	Matx33f cameraMatrix;
	/** Camera lense distortion parameters */
	Mat distortionCoeffs;
	/** Camera parameters extracted from cameraMatrix for faster access. */
	double cx, cy, fx, fy;
	/** Indicates wether calibration data is set. */
	bool isCalibrated;

	/** Transformation from camera coordinate system to world coordinate system (cam->world) */
	Matx44f T;
	/** Is this->T set? (or should be considered as NULL) */
	bool isTSet;

public:
	/** True if the camera is stationary (R and T do not change over time) */
	bool isStationary;

	/** ID of the camera (the coordinate system) */
	int cameraID;

	/** Constructor. */
	Camera();

	// Setters
	void setCameraMatrix(Mat camMtx);

	void setDistortionCoeffs(Mat distortCoeffMtx);

	Matx44f GetT()
	{
		return T;
	}
	bool getIsTSet()
	{
		return isTSet;
	}
	void resetT()
	{
		isTSet = false;
	}

	/** Point: image -> cam (results ray!) */
	Ray pointImg2Cam(Point2f pImg);

	/** Point: cam -> image */
	Point2f pointCam2Img(Matx41f pCam);

	/** Point: cam -> world */
	Matx41f pointCam2World(Matx41f pCam);

	/** Point: world -> cam */
	Matx41f pointWorld2Cam(Matx41f pWorld);

	/** Ray: cam -> image */
	Ray2D rayCam2Img(Ray rCam);

	/** Ray: cam -> image
		Warning: works only for rays created by the same, stationary camera! */
	Point2f rayOrigCam2Img(Ray rOrigCam);

	/** Ray: cam -> world */
	Ray rayCam2World(Ray rCam);

	/** Ray: world -> cam */
	Ray rayWorld2Cam(Ray rWorld);

	/** Point image -> world (results ray) (wrapper function) */
	Ray pointImg2World(Point2f pImg);

	/** Point world -> image (wrapper function) */
	Point2f pointWorld2Img(Mat pWorld);

	/** Load camera intrinsic parameters from XML
		Use the camera_calibration sample to calibrate the camera.
		After saving that, you have to modify the resulting file to contain only the
		data items contained in the following example:

		<?xml version="1.0"?>
		<opencv_storage>
		<Camera_Matrix type_id="opencv-matrix">
		  <rows>3</rows>
		  <cols>3</cols>
		  <dt>d</dt>
		  <data>
			7.9657299110744543e+002 0. 3.1950000000000000e+002 0.
			7.9657299110744543e+002 2.3950000000000000e+002 0. 0. 1.</data></Camera_Matrix>
		<Distortion_Coefficients type_id="opencv-matrix">
		  <rows>5</rows>
		  <cols>1</cols>
		  <dt>d</dt>
		  <data>
			-1.2934763752655315e-001 1.6554150366744376e+000 0. 0.
			-7.8517390502797522e+000</data></Distortion_Coefficients>
		</opencv_storage>
	*/
	bool loadCalibrationData(const char *filename);

	/** Find extrinsic camera parameters using 2D-3D correspondences.
		TODO: later, potential previous extrinsic parameters can be used as an initial guess...
	*/
	bool calculateExtrinsicParams(vector<Point3f> objectPoints, vector<Point2f> imagePoints);

	/** Calls calculateExtrinsicParams() unless the transformation matrix T is already known and the
		camera is known to be stationary (isStationary && isTSet).
	*/
	bool calculateExtrinsicParamsIfNeeded(vector<Point3f> objectPoints, vector<Point2f> imagePoints);

	// TODO: add polymorph conversion functions!

	// TODO: Should have a function to project 3D points to image using this function:
	// void projectPoints(InputArray objectPoints, InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, OutputArray imagePoints, OutputArray jacobian=noArray(), double aspectRatio=0 )¶

	/** Uses the camera calibration data (distortion coefficients) to undistort an image. */
	void undistortImage(Mat& src, Mat& dst);
};

#endif
