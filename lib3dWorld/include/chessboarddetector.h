#ifndef __CHESSBOARDDETECTOR_H
#define __CHESSBOARDDETECTOR_H
#include <opencv2/core/core.hpp>
#include "chessboard.h"


/** Detects a Chessboard on a given image. It creates a Chessboard instance and looks for it
	in a given image. If found, chessboard corners are stored in the pointBuf vector.
	Designed to be used with Camera.calculateExtrinsicParams().

	Example:

	ChessboardDetector detector(Size(9,6),50);
	if (detector.findChessboardInFrame(frame))
	{
		drawChessboardCorners(frame,Size(9,6),detector.pointBuf,true);
		cam.calculateExtrinsicParams(detector.chessboard.corners,detector.pointBuf);
	}
*/
class ChessboardDetector
{
public:
	/** The used chassboard instance, initialized by constructors. Mainly for internal use. */
	Chessboard chessboard;

	/** Constructor */
	ChessboardDetector(cv::Size iBoardSize, int iSquareSize);
	/** Constructor */
	ChessboardDetector(Chessboard& iChessboard);

	/** Did the last findChessboardInFrame() found the chessboard? Every call overwrites this value. */
	bool found;
	/** Coordinates of the chessboard corners after successful detection. */
	std::vector<cv::Point2f> pointBuf;

	/** Searches for a chessboard in the given image. If it is found, results are stored in pointbuf and
		found is set true. Otherwise, found is set false.

		@param	frame	Color input image frame 
		@return			Returns true if detection was successful.
	*/
	bool findChessboardInFrame(cv::Mat& frame);
};

#endif
