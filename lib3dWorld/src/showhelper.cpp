#include <iostream>

#include "showhelper.h"

using namespace cv;
using namespace std;

void ShowHelper::show(char *msg, Matx41f value)
{
	cout << msg << ": ";
	for(int i=0; i<4; i++)
	{
		cout << value.val[i] << " ";
	}
	cout << endl;
}

void ShowHelper::show(char *msg, Matx44f value)
{
	cout << msg << ":" << endl;
	int i=0;
	for(int r=0; r<4; r++)
	{
		for(int c=0; c<4; c++)
		{
			cout << value.val[i++] << " ";
		}
		cout << endl;
	}
	cout << endl;
}