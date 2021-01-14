#include "pattern.h"
#include <iostream>

using namespace cv;
using namespace std;


namespace ARma {

	Pattern::Pattern(double param1) {
		id = -1;
		size = param1;
		orientation = -1;
		confidence = -1;

		rotVec = (Mat_<float>(3, 1) << 0, 0, 0);
		transVec = (Mat_<float>(3, 1) << 0, 0, 0);
		rotMat = Mat::eye(3, 3, CV_32F);

	}

	//convert rotation vector to rotation matrix (if you want to proceed with other libraries)
	void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);
	}

	void Pattern::showPattern()
	{
		cout << "Pattern ID: " << id << endl;
		cout << "Pattern Size: " << size << endl;
		cout << "Pattern Confedince Value: " << confidence << endl;
		cout << "Pattern Orientation: " << orientation << endl;
		rotationMatrix(rotVec, rotMat);
		cout << "Exterior Matrix (from pattern to camera): " << endl;
		for (int i = 0; i < 3; i++) {
			cout << rotMat.at<float>(i, 0) << "\t" << rotMat.at<float>(i, 1) << "\t" << rotMat.at<float>(i, 2) << " |\t" << transVec.at<float>(i, 0) << endl;
		}
	}

	void Pattern::getExtrinsics(int patternSize, Mat cameraMatrix, Mat distortions)
	{

		Mat intrinsics = cameraMatrix;
		Mat distCoeff = distortions;
		Mat rot = rotVec;
		Mat tra = transVec;
		Mat rotationMatrix = rotMat;

		CvPoint2D32f pat2DPts[4];
		for (int i = 0; i < 4; i++) {
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;
		}

		//3D points in pattern coordinate system
		CvPoint3D32f pat3DPts[4];
		pat3DPts[0].x = 0.0;
		pat3DPts[0].y = 0.0;
		pat3DPts[0].z = 0.0;
		pat3DPts[1].x = patternSize;
		pat3DPts[1].y = 0.0;
		pat3DPts[1].z = 0.0;
		pat3DPts[2].x = patternSize;
		pat3DPts[2].y = patternSize;
		pat3DPts[2].z = 0.0;
		pat3DPts[3].x = 0.0;
		pat3DPts[3].y = patternSize;
		pat3DPts[3].z = 0.0;

		Mat objectPts(4, 3, CV_32FC1, pat3DPts);//header for 3D points of pat3Dpts
		Mat imagePts(4, 2, CV_32FC1, pat2DPts);//header for 2D image points of pat2Dpts 
		Mat inliers_idx;
		vector<Point2f> list_points2d_inliers;


		solvePnP(objectPts, imagePts, intrinsics, distCoeff, rotVec, transVec);
	}


	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix, Mat toInsert, Rect box)
	{
		CvScalar color = cvScalar(255, 255, 255);
		std::vector<cv::Point2f> model2ImagePts;

		int op = 0;
		switch (id) {
		case 1:
			op = 1;
			color = cvScalar(255, 0, 255);
			break;
		case 2:
			op = 2;
			color = cvScalar(255, 255, 0);
			break;
		case 3:
			op = 3;
			color = cvScalar(0, 255, 255);
			break;
		case 4:
			op = 4;
			color = cvScalar(0, 0, 0);
			break;
		}

		//model 3D points: they must be projected to the image plane
		Mat modelPts1;
		if (op == 1) {
			//index 1 patron de 1 cuadrado
			insertImage(toInsert, frame, box);
			/*
			Mat modelPts = (Mat_<float>(8, 3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,
				0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size);
			modelPts.copyTo(modelPts1);
			*/

			//Method 2 rotates image
			Mat newtoInsert = toInsert.clone();
			//cv::rotate(toInsert, newtoInsert, ROTATE_90_CLOCKWISE);

			//Method 3 translates image position
			//Transformation(toInsert, newtoInsert, 15, 15);
			

			//Method 4
			//Growth(toInsert, newtoInsert, 2, 2);

			//insertImage(newtoInsert, frame, box);

			//op = 3;
			
		}



		//************************************************************************
		Mat modelFaces(16, 3, CV_32F, Scalar(0.0f));
		if (op == 2) {
			//index 2 patron de 2 cuadrados
			Mat newtoInsert;

			insertImage(newtoInsert, frame, box);
			
		}


		//************************************************************************
		if (op == 3) {
			//index 3 patron de 1 triangulo
			Mat modelPts(10, 3, CV_32F, Scalar(0.0f));
			pointsicosahedro(modelPts, modelFaces);
			modelPts.copyTo(modelPts1);
			projectPoints(modelPts1, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);
			
		}

		//************************************************************************
		//************************************************************************
		if (op == 4) {
			//index 4 patron de 2 triangulos
			Mat newtoInsert;
			
			insertImage(newtoInsert, frame, box);
		}

		//************************************************************************
		

		/* project model 3D points to the image. Points through the transformation matrix
		(defined by rotVec and transVec) "are transfered" from the pattern CS to the
		camera CS, and then, points are projected using camera parameters
		(camera matrix, distortion matrix) from the camera 3D CS to its image plane
		*/
		
		cout << rotVec << endl;
		//draw cube, or whatever
		int i;
		/*
		if (op == 0) {
			for (i = 0; i < 4; i++) {
				cv::line(frame, model2ImagePts[i % 4], model2ImagePts[(i + 1) % 4], color, 3);
			}
			for (i = 4; i < 7; i++) {
				cv::line(frame, model2ImagePts[i % 8], model2ImagePts[(i + 1) % 8], color, 3);
			}
			cv::line(frame, model2ImagePts[7], model2ImagePts[4], color, 3);
			for (i = 0; i < 4; i++) {
				int iAUX = i + 4;
				cv::line(frame, model2ImagePts[i], model2ImagePts[iAUX], color, 3);
			}

			//draw the line that reflects the orientation. It indicates the bottom side of the pattern
			cv::line(frame, model2ImagePts[2], model2ImagePts[3], cvScalar(80, 255, 80), 3);
		}
		*/
		
		if (op == 3)
		{

			for (i = 0; i < modelFaces.size().height; i++) {
				cv::line(frame, model2ImagePts.at(modelFaces.at<float>(i, 0)), model2ImagePts.at(modelFaces.at<float>(i, 1)), color, 3);
				cv::line(frame, model2ImagePts.at(modelFaces.at<float>(i, 1)), model2ImagePts.at(modelFaces.at<float>(i, 2)), color, 3);
				cv::line(frame, model2ImagePts.at(modelFaces.at<float>(i, 2)), model2ImagePts.at(modelFaces.at<float>(i, 0)), color, 3);
			}
			cv::line(frame, model2ImagePts.at(5), model2ImagePts.at(4), cvScalar(80, 255, 80), 3);
			cv::line(frame, model2ImagePts.at(5), model2ImagePts.at(6), cvScalar(80, 255, 80), 3);
			cv::line(frame, model2ImagePts.at(4), model2ImagePts.at(0), cvScalar(80, 255, 80), 3);
			cv::line(frame, model2ImagePts.at(6), model2ImagePts.at(1), cvScalar(80, 255, 80), 3);
			cv::line(frame, model2ImagePts.at(0), model2ImagePts.at(1), cvScalar(80, 255, 80), 3);


		}
		
		model2ImagePts.clear();


	}


	void Pattern::pointsicosahedro(Mat& points, Mat& faces)
	{
		static float v[10][3] = {
			0.000000, 0.000000, 1.000000,
			1.000000, 0.000000, 0.000000,
			-1.000000, 0.000000, 0.000000,
			0.000000, -1.000000, 0.000000,
			0.000000, 0.000000, -1.000000,
			0.707107, 1.035813, -0.707107,
			0.000000, 2.035813, 0.000000,
			0.707107, 1.035813, 0.707107,
			-0.707107, 1.035813, 0.707107,
			-0.707107, 1.035813, -0.707107,
		};
		static int f[16][3] = {
			5, 6, 7,
			6, 8, 7,
			2, 3, 0,
			0, 3, 1,
			9, 6, 5,
			9, 8, 6,
			4, 3, 4,
			4, 1, 3,
			7, 1, 5,
			8, 0, 7,
			1, 4, 5,
			9, 2, 8,
			0, 1, 7,
			5, 4, 9,
			4, 2, 9,
			2, 0, 8,
		};
		int i;
		float* p;
		float a, b;
		float len;

		a = 0.5 * (1 + sqrt(5));
		b = 1;

		len = sqrt(a * a + b * b);
		a /= len;
		b /= len;

		for (i = 0, p = (float*)v; i < 10 * 3; i++, p++) {
			if (fabs(*p) == 1)
				*p *= a;
			if (fabs(*p) == 2)
				*p *= b / 2.0;
		}

		int nverts = 10;
		int nfaces = 16;

		/*for (i = 0; i < nverts; i++) {
			vlist[i].x = v[i][0];
			vlist[i].y = v[i][1];
			vlist[i].z = v[i][2];
		}
*/
		for (i = 0; i < nverts; i++) {
			points.at<float>(i, 0) = (v[i][0] + 1) * size / 2;
			points.at<float>(i, 1) = (v[i][1] + 1) * size / 2;
			points.at<float>(i, 2) = (v[i][2]) * size / 2;
		}

		/*for (i = 0; i < nfaces; i++) {
			flist[i].nverts = 3;
			flist[i].verts = (int *)malloc(sizeof(int)* 3);
			flist[i].verts[0] = f[i][0];
			flist[i].verts[1] = f[i][1];
			flist[i].verts[2] = f[i][2];
		}*/
		for (i = 0; i < nfaces; i++) {
			faces.at<float>(i, 0) = f[i][0];
			faces.at<float>(i, 1) = f[i][1];
			faces.at<float>(i, 2) = f[i][2];
		}
	}

	void Pattern::insertImage(Mat toInsert, Mat output, Rect& rec) {

		//Mat temp = output.colRange(rec.x, rec.x + rec.width).rowRange(rec.y,rec.y + rec.height);
		//temp.setTo(125);

		for (int y = rec.x; y < rec.x + toInsert.cols; y++)
		{
			for (int x = rec.y; x < rec.y + toInsert.rows; x++)
			{
				if (toInsert.at<Vec3b>(x-rec.y, y-rec.x) != Vec3b(0, 0, 0))
				{
					output.at<Vec3b>(x, y) = toInsert.at<Vec3b>(x - rec.y, y - rec.x);
				}
			}
		}
	}

	void Pattern::Transformation(Mat toInsert, Mat output, int x, int y) {

		output = Scalar(0);

		for (int i = 0; i < toInsert.rows; i++) {
			for (int j = 0; j < toInsert.cols; j++) {

				if (i + x < output.rows && j + y < output.cols) {
					output.at<Vec3b>(i + x, j + y) = toInsert.at<Vec3b>(i, j);
				}

			}
		}

	}

	void Pattern::Growth(Mat toInsert, Mat output, int x, int y) {

		output = Scalar(0);

		for (int i = 0; i < toInsert.rows * x; i++) {
			for (int j = 0; j < toInsert.cols * y; j++) {

				int u = (int)i / x;
				int v = (int)j / y;

				if (i + x < output.rows && j + y < output.cols) {
					output.at<Vec3b>(i, j) = toInsert.at<Vec3b>(u, v);
				}

			}
		}

	}

}


