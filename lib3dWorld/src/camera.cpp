#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>	// for OPENCV_ASSERT
#include <opencv2/calib3d/calib3d.hpp>	// for solvePnP
#include <opencv2/imgproc/imgproc.hpp>	// for undistort
#include "camera.h"

Camera::Camera()
{
	isTSet=false;
	isCalibrated=false;
}

// Setters
void Camera::setCameraMatrix(Mat camMtx)
{
	OPENCV_ASSERT(camMtx.rows==3,"Camera.setCameraMatrix","Camera matrix does not have 8 rows.");
	OPENCV_ASSERT(camMtx.cols==3,"Camera.setCameraMatrix","Camera matrix does not have 1 column.");
	OPENCV_ASSERT(camMtx.type() == CV_64F,"Camera.setCameraMatrix","Camera matrix does not have type CV_64F.");

	cameraMatrix = camMtx;
	// Extract camera parameters for faster access
	fx = (float)camMtx.at<double>(0,0);
	fy = (float)camMtx.at<double>(1,1);
	cx = (float)camMtx.at<double>(0,2);
	cy = (float)camMtx.at<double>(1,2);
}

void Camera::setDistortionCoeffs(Mat distortCoeffMtx)
{
	OPENCV_ASSERT(distortCoeffMtx.rows==4 || distortCoeffMtx.rows==5 || distortCoeffMtx.rows==8,"Camera.setDistortionCoeffs","Camera distortion coefficient matrix does not have 4,5 or 8 rows.");
	OPENCV_ASSERT(distortCoeffMtx.cols==1,"Camera.setDistortionCoeffs","Camera distortion coefficient matrix does not have 1 column.");
	OPENCV_ASSERT(distortCoeffMtx.type() == CV_64F,"Camera.setDistortionCoeffs","Camera distortion coefficient matrix does not have type CV_64F.");
	
	distortionCoeffs = distortCoeffMtx;
}

/** Point: image -> cam (results ray!) */
Ray Camera::pointImg2Cam(Point2f pImg)
{
	// Orientation vector in the cameras own coordinate system
	float vx = pImg.x - cx;
	float vy = pImg.y - cy;
	float vz = fx;	// Warning! Aspect ratio is assumed to be 1 !!!

	// Create ray
	Ray ray;
	ray.originalCameraID = cameraID;
	ray.originalImageLocation = pImg;
	ray.A = Matx41f(0.0F,0.0F,0.0F,1.0F);
	ray.B = Matx41f(vx,vy,vz,1.0F);
	ray.cameraID = cameraID;

	return ray;
}

/** Point: cam -> image */
Point2f Camera::pointCam2Img(Matx41f pCam)
{
	// Get 3D coordinates (from homogeneous coordinates)
	float x = pCam.val[0] / pCam.val[3];
	float y = pCam.val[1] / pCam.val[3];
	float z = pCam.val[2] / pCam.val[3];

	OPENCV_ASSERT(z != 0,"Camera.pointCam2Img","z==0, unable to perform perspectivic projection");

	// Camera transformation
	Point2f pImg;
	pImg.x = fx * ( x / z ) + cx;
	pImg.y = fy * ( y / z ) + cy;

	return pImg;
}

/** Point: cam -> world */
Matx41f Camera::pointCam2World(Matx41f pCam)
{
	OPENCV_ASSERT(isTSet,"Camera.pointCam2World","Using unknown camera transformation!");
	Matx41f pWorld = T * pCam;
	return pWorld;
}

/** Point: world -> cam */
Matx41f Camera::pointWorld2Cam(Matx41f pWorld)
{
	OPENCV_ASSERT(isTSet,"Camera.pointWorld2Cam","Using unknown camera transformation!");
	Matx41f pCam = T.inv() * pWorld;
	return pCam;
}

/** Ray: cam -> image */
Ray2D Camera::rayCam2Img(Ray rCam)
{
	OPENCV_ASSERT(rCam.originalCameraID != cameraID,"Camera.rayCam2Img","Reprojection of ray into original camera is not supported. Use rayOrigCam2Img() instead!");

	Ray2D rImg;
	rImg.cameraID = cameraID;
	rImg.A = pointCam2Img(rCam.A);
	rImg.B = pointCam2Img(rCam.B);

	return rImg;
}

/** Ray: cam -> image
	Warning: works only for rays created by the same, stationary camera! */
Point2f Camera::rayOrigCam2Img(Ray rOrigCam)
{
	OPENCV_ASSERT(rOrigCam.originalCameraID == cameraID,"Camera.rayOrigCam2Img","This is not the original camera. Use rayCam2Img() instead!");
	// TODO: should be able to throw error in release mode as well!
	Point2f pImg;
	pImg = rOrigCam.originalImageLocation;
	return pImg;
}

/** Ray: cam -> world */
Ray Camera::rayCam2World(Ray rCam)
{
	OPENCV_ASSERT(rCam.cameraID == cameraID,"Camera.rayCam2World","Ray is given in some other camera's coordinate system. Cannot transform...");
	OPENCV_ASSERT(isTSet,"Camera.rayCam2World","Using unknown camera transformation!");

	// Create ray
	Ray rWorld = rCam;
	rWorld.A = T * rCam.A;
	rWorld.B = T * rCam.B;
	rWorld.cameraID = CAMID_WORLD;
	return rWorld;
}

/** Ray: world -> cam */
Ray Camera::rayWorld2Cam(Ray rWorld)
{
	OPENCV_ASSERT(isTSet,"Camera.rayWorld2Cam","Using unknown camera transformation!");
	// Create ray
	Ray rCam = rWorld;
	rCam.A = T.inv() * rWorld.A;
	rCam.B = T.inv() * rWorld.B;
	rCam.cameraID = cameraID;
	return rCam;
}

/** Point image -> world (results ray) (wrapper function) */
Ray Camera::pointImg2World(Point2f pImg)
{
	Ray rCam = pointImg2Cam(pImg);
	Ray rWorld = rayCam2World(rCam);
	return rWorld;
}

/** Point world -> image (wrapper function) */
Point2f Camera::pointWorld2Img(Matx41f pWorld)
{
	Matx41f pCam = pointWorld2Cam(pWorld);
	Point2f pImg = pointCam2Img(pCam);
	return pImg;
}

bool Camera::loadCalibrationData(const char *filename)
{
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		OPENCV_ASSERT(false,"Camera.loadCalibrationData","Cannot load calibration data!");
		return false;
	}

	Mat camMat = Mat::eye(3, 3, CV_64F);
	Mat dCoeffs = Mat::zeros(8, 1, CV_64F);
	Size cameraResolution;

	fs["Camera_Matrix"] >> camMat;
	fs["Distortion_Coefficients"] >> dCoeffs;
	FileNode fn = fs["Camera_Resolution"];
	fn["Width"] >> cameraResolution.width;
	fn["Height"] >> cameraResolution.height;

	if(cameraResolution.width == 0 || cameraResolution.height == 0)
	{
		OPENCV_ASSERT(false,"Camera.loadCalibrationData","Camera resolution parameter not found.");
		return false;
	}
	
	setCameraMatrix(camMat);
	setDistortionCoeffs(dCoeffs);
	setCameraResolution(cameraResolution);
	
	fs.release(); 
	isCalibrated=true;
	return true;
}

bool Camera::calculateExtrinsicParams(vector<Point3f> objectPoints, vector<Point2f> imagePoints)
{
	OPENCV_ASSERT(isCalibrated,"Camera.calculateExtrinsicParams","Cannot calculate extrinsic parameters before camera calibration!");
	Mat rvec, tvec;
	Mat rotMtx;
	bool solverResult = solvePnP(
		Mat(objectPoints), Mat(imagePoints),	// Input correspondences
		cameraMatrix, distortionCoeffs,	// Intrinsic camera parameters (have to be already available)
		rvec, tvec);

	if (solverResult)
	{
		// Create this->T from rvec and tvec
		Rodrigues(rvec, rotMtx);
		Matx44f T_inv = Matx44f(
			(float)rotMtx.at<double>(0,0), (float)rotMtx.at<double>(0,1), (float)rotMtx.at<double>(0,2), (float)tvec.at<double>(0,0),
			(float)rotMtx.at<double>(1,0), (float)rotMtx.at<double>(1,1), (float)rotMtx.at<double>(1,2), (float)tvec.at<double>(1,0),
			(float)rotMtx.at<double>(2,0), (float)rotMtx.at<double>(2,1), (float)rotMtx.at<double>(2,2), (float)tvec.at<double>(2,0),
			0.0F, 0.0F, 0.0F, 1.0F
			);
		T = T_inv.inv();
		isTSet=true;
	}

	return solverResult;
}

bool Camera::calculateExtrinsicParamsIfNeeded(vector<Point3f> objectPoints, vector<Point2f> imagePoints)
{
	if (!isStationary || !isTSet)
	{
		return calculateExtrinsicParams(objectPoints, imagePoints);
	}
	return false;
}

void Camera::undistortImage(Mat& src, Mat& dst)
{
	OPENCV_ASSERT(isCalibrated,"Camera.undistortImage","Cannot undistort before camera calibration!");
	undistort(src, dst, cameraMatrix, distortionCoeffs);
}

void Camera::setCameraResolution(Size resolution)
{
	cameraResolution = resolution;
}

void Camera::drawOrigin(Mat* src, int axisLength)
{
	OPENCV_ASSERT(isCalibrated,"Camera.drawOrigin","Cannot draw origin before camera calibration!");
	
	Matx44d axis = Matx44d(
		axisLength, 0, 0, 1,
		0, axisLength, 0, 1,
		0, 0, axisLength, 1,
		0, 0, 0, 1
		);

	Point x = pointWorld2Img(axis.row(0).t());
	Point y = pointWorld2Img(axis.row(1).t());
	Point z = pointWorld2Img(axis.row(2).t());
	Point origin = pointWorld2Img(axis.row(3).t());


	putText(*src, "X", Point(x.x+10, x.y+10), 1, 1, Scalar(255, 0, 0));
	putText(*src, "Y", Point(y.x+10, y.y+10), 1, 1, Scalar(0, 255, 0));
	putText(*src, "Z", Point(z.x+10, z.y+10), 1, 1, Scalar(0, 0, 255));

	line(*src, origin, x, Scalar(255, 0, 0), 2);
	line(*src, origin, y, Scalar(0, 255, 0), 2);
	line(*src, origin, z, Scalar(0, 0, 255), 2);
}