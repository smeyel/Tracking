#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "chessboarddetector.h"

using namespace cv;
using namespace std;

ChessboardDetector::ChessboardDetector(Size iBoardSize, int iSquareSize)
{
	found = false;
	chessboard = Chessboard(iBoardSize,iSquareSize);
	pointBuf.resize(chessboard.boardSize.width * chessboard.boardSize.height);
}

ChessboardDetector::ChessboardDetector(Chessboard& iChessboard)
{
	// Copy the chessboard instance
	this->chessboard = iChessboard;
}


bool ChessboardDetector::findChessboardInFrame(Mat& frame)
{
	//int succeses = 0;

	int cornersDetected = 0;
	vector<CvPoint2D32f> corners;
	IplImage dst_img = frame;
	int sumCornerNum = chessboard.boardSize.width * chessboard.boardSize.height;
	corners.resize(sumCornerNum);
	int result = cvFindChessboardCorners(&dst_img, cvSize(chessboard.boardSize.width, chessboard.boardSize.height), &corners[0], &cornersDetected, 
		CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
	found = (cornersDetected == sumCornerNum);

	if (found)                // If done with success,
	{
		// Copy results into pointBuf
		for(int i=0; i<sumCornerNum; i++)
		{
			pointBuf[i]=Point2f(corners[i]);
		}

		// improve the found corners' coordinate accuracy for chessboard
		Mat viewGray;
		cvtColor(frame, viewGray, CV_BGR2GRAY);
		cornerSubPix( viewGray, pointBuf, Size(11,11),
		Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

		// Draw the corners.
		drawChessboardCorners( frame, chessboard.boardSize, Mat(pointBuf), found );

		cout << "(Chessboard detected in image.)" << endl;
	}

	return found;
}
