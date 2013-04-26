#ifndef __CHESSBOARD_H
#define __CHESSBOARD_H

#include <opencv2/core/core.hpp>

/** Represents a physical chessboard with given number of rows and columns and sizes in mm.
	Used internally by ChessboardDetector, but you may also access it when necessary.
	To set up, simply call one of the constructors and you are done.
*/
class Chessboard
{
public:
	/** Vector of 3D corner points. Calculated by the constructors using calcCorners() */
	std::vector<cv::Point3f> corners;
	/** Edge size of a single square, in mm. */
	float squareSize;
	/** Size of the board: number of internal corners (not number of squares!) as row and column number. */
	cv::Size boardSize;

	/** Constructor */
	Chessboard(cv::Size iBoardSize, float iSquareSize);
	/** Constructor */
	Chessboard(Chessboard& iChessboard);
	/** Default constructor. Warning: no initialization is performed! Call calcCorners() after setting boardSize and squareSize! */
	Chessboard();

	/** Calculates coordinates of corners automatically. Called by the constrcutors automatically. */
	void calcCorners(cv::Size boardSize, float squareSize);
};



#endif
