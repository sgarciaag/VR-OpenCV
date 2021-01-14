#ifndef _ARMA_PATTERN_
#define _ARMA_PATTERN_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;
using namespace std;

namespace ARma {

class Pattern
{
	public:
		vector<Point2f> vertices;
		int id;
		int orientation;//{0,1,2,3}
		float size; //in milimeters
		double confidence;//min: -1, max: 1
		Mat rotVec, transVec, rotMat, toInsert;
		Rect pBox;

		Pattern(double param1=80);

		~Pattern(){};
		
		//solves the exterior orientation problem between patten and camera
		void getExtrinsics(int patternSize,  Mat cameraMatrix,  Mat distortions);

		//augments image with 3D cubes. It;s too simple augmentation jsut for checking
		void draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix, Mat toInsert, Rect box);

		//computes the rotation matrix from the rotation vector using Rodrigues
		void rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix);
		
		//prints the properties of the patten and its transformation matrix
		void showPattern(void);

		void pointsicosahedro(Mat &points, Mat &faces);

		void insertImage(Mat toInsert, Mat output, Rect& rec);

		void Transformation(Mat toInsert, Mat output, int x, int y);

		void Growth(Mat toInsert, Mat output, int x, int y);

		void rotation(Mat toInsert, Mat output, int theta);
};


}

#endif
