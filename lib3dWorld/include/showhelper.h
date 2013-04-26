#ifndef __SHOWHELPER_H
#define __SHOWHELPER_H
#include <opencv2/core/core.hpp>

class ShowHelper
{
public:
	static void show(char *msg, cv::Matx41f value);
	static void show(char *msg, cv::Matx44f value);
};

#endif
