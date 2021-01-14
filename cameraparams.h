#ifndef _CAMERAPARAMS_H
#define _CAMERAPARAMS_H
#include <opencv2/core/core.hpp>
using namespace cv;

	CvMat* intrinsic = (CvMat*)cvLoad("intrinsic.xml");	
	CvMat* distor = (CvMat*)cvLoad("distortion.xml");

	Mat cameraMatrix = cvarrToMat(intrinsic);
	Mat distortions = cvarrToMat(distor);
	//CvMat cameraMatrix = CvMat(intrinsic);
	//CvMat distortions = (distor);

	//void read( )                          //Read serialization for this class
	//  {
	//  string filename = "I.xml";
	//  FileStorage fs(filename, FileStorage::WRITE);
	//  //...
	//  fs.open(filename, FileStorage::READ);
	//  int itNr;
	//  fs["iterationNr"] >> itNr;
	//  itNr = (int)fs["iterationNr"];
	//	A = (int)fs["A"];
	//	X = (double)fs["X"];
	//	id = (string)fs["id"];
	//}

#endif


