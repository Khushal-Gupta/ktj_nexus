/*

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<cmath>
#include<cstring>
using namespace std;
using namespace cv;

int main() {
	Mat img = imread("rubik.jpg",1);
	Mat img_gray;
	Mat hsv = img.clone();
	Mat bgr[3];
	cvtColor(img, hsv, COLOR_BGR2HSV);
	cvtColor(img, img_gray, COLOR_BGR2GRAY);
	namedWindow("img", WINDOW_NORMAL);
	namedWindow("hsv", WINDOW_NORMAL);


	split(hsv, bgr);

	imshow("img", hsv);
	imshow("hsv", bgr[0]);
	waitKey(0);
}
*/


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<iostream>
#include<vector>
using namespace std;
using namespace cv;

int after_poly = 0;
/*

struct pnt {
	int x, y;	
};


pnt poly_coor(Mat img) {
	int bmax = 15, gmax = 15, rmax = 15;
	Mat a = img.clone(), imgPoly;
	inRange(a, Scalar(0,0,0), Scalar(bmax, gmax, rmax), imgPoly);
	for(int i=0; i<3; i++) {
  		erode(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
	}
	for(int i=0; i<10; i++) {
		dilate(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
	}
	Canny(imgPoly, imgPoly, 50,100,3);
	vector <vector<Point>> contoursPoly;
	vector < Vec4i > hierarchyPoly; 
	findContours(d, contoursPoly, hierarchyPoly, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


	// Finding centre of triangle or rectangle
	vector<Moments> muPoly(contoursPoly.size());
	float areaPoly ;
	int x_poly = 0, y_poly = 0, i_poly;
	for(int i=0; i< contours.size(); i++) {
		muPoly[i] = moments(contours[i]);
		if((int)mu[i].m01 / mu[i].m00 > x_poly) {
			x_poly = (int)mu[i].m01 / mu[i].m00;
			y_poly = (int)mu[i].m10 / mu[i].m00;
			i_poly = i;
		}
	
	}
	pnt p1;
	p1.x = x_poly;
	p1.y = y_poly;
	return p1;

}
 


*/ 
int main(int , char **)
{
	VideoCapture cap(0);
	if(!cap.isOpened())
		return -1;

	int i,j,hmin=55,smin=0,vmin=0,hmax=65,smax=255,vmax=255;
	namedWindow("original",WINDOW_NORMAL);
	namedWindow("hsv",WINDOW_NORMAL);
	//namedWindow("after_canny",WINDOW_NORMAL);
	// namedWindow("after_canny",WINDOW_NORMAL);
	namedWindow("final_output",WINDOW_NORMAL);


	createTrackbar("vmin","hsv",&vmin,255);
	createTrackbar("vmax","hsv",&vmax,255);
	createTrackbar("hmin","hsv",&hmin,180);
	createTrackbar("hmax","hsv",&hmax,180);
	createTrackbar("smin","hsv",&smin,255);
	createTrackbar("smax","hsv",&smax,255);


	for(int u=0 ; u<1; u++)
	{
		Mat img;
		cap>>img;
		img = imread("B1.jpg",1);
		Mat a = img.clone();

		int bmax = 10, gmax = 10, rmax = 10;

		Mat imgHSV;
		cvtColor(a,imgHSV,CV_BGR2HSV);
		
		// for(i=0;i<img.rows;++i)
		// {
		// 	for(j=0;j<img.cols;++j)
		// 	{
		// 		if(img.at<Vec3b>(i,j)[0]>=hmin && img.at<Vec3b>(i,j)[1]>=smin && img.at<Vec3b>(i,j)[2]>=vmin && img.at<Vec3b>(i,j)[0]<=hmax && img.at<Vec3b>(i,j)[1]<=smax && img.at<Vec3b>(i,j)[2]<=vmax)
		// 		{
		// 			a.at<Vec3b>(i,j)[0] = img.at<Vec3b>(i,j)[0];
		// 			a.at<Vec3b>(i,j)[1] = img.at<Vec3b>(i,j)[1];
		// 			a.at<Vec3b>(i,j)[2] = img.at<Vec3b>(i,j)[2];

		// 		}
		// 		else {
		// 			a.at<Vec3b>(i,j)[0] = 0;
		// 			a.at<Vec3b>(i,j)[1] = 0;
		// 			a.at<Vec3b>(i,j)[2] = 0;
		// 		}

		// 	}
		// }
		// int thres = 190;

		Mat imgPoly = img.clone();
		
		cvtColor(imgPoly, imgPoly, CV_BGR2GRAY);
		

		for(int i=0; i < 1; i++) {                                           //  Thresholding first time
			GaussianBlur(imgPoly, imgPoly, Size( 9, 9 ), 9, 9 );
			threshold(imgPoly, imgPoly, 50, 255, THRESH_BINARY);
		}

		
		

		for(int i=0; i<5; i++) {											// Thresholding second time
			GaussianBlur(imgPoly, imgPoly, Size( 7, 7 ), 5, 5 );
			threshold(imgPoly, imgPoly, 120, 255, THRESH_BINARY);
		}
		



		// namedWindow("fun", WINDOW_NORMAL);
		// imshow("fun", imgPoly);
	
		//threshold(imgPoly, imgPoly, 120, 255, THRESH_BINARY_INV);
		
		//inRange(a, Scalar(0,0,0), Scalar(bmax, gmax, rmax), imgPoly);



		Mat imgThresholded ;

		inRange(imgHSV, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), imgThresholded);      // detects green colour


		

		for(int i=0; i<3; i++) {
  			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  			erode(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
		}
  		for(int i=0; i<5; i++) {
  			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
  			dilate(imgPoly, imgPoly, getStructuringElement(MORPH_RECT, Size(3, 3)));
  		}



  		// Applying Gaussian Blur and Thresholding to image
  		for(int i = 0 ;i < 1; i++) {
  			erode(imgPoly, imgPoly, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
  			GaussianBlur( imgPoly, imgPoly, Size( 7, 7 ), 3, 3 );
			threshold(imgPoly, imgPoly, 230, 255, THRESH_BINARY);
  		}

  		

  		// Turning boundary pixels to black
		
  		for(int i=0; i<img.cols; i++) {
  			for(int k = 0; k < 8; k++) {
  			imgThresholded.at<uchar>(k, i) = 0;
  			}
  		}
  		for(int i=0; i<img.cols; i++) {
  			for(int k = 1; k < 8 ;k++){
  				imgThresholded.at<uchar>(img.rows-k, i) = 0;
  			}
  		}
  		for(int i=0; i<img.rows; i++) {
  			for(int k = 0 ;k <8; k++ ){
  				imgThresholded.at<uchar>(i, k) = 0;
  			}
		}
  		for(int i=0; i<img.rows; i++) {
  			for(int k = 1; k < 8 ;k++) {
  				imgThresholded.at<uchar>(i, img.cols-k) = 0;
  			}
  		}
  		for(int i = 0 ;i < 1; i++) {
  			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
  			GaussianBlur( imgThresholded, imgThresholded, Size( 7, 7 ), 3, 3 );
			threshold(imgThresholded, imgThresholded, 230, 255, THRESH_BINARY);
  		}

  		

  		

  		

		Mat d = imgThresholded.clone();

		namedWindow("d",WINDOW_NORMAL);
		imshow("d", imgThresholded);
		//split(a, d);

		
		//Applying canny filter
		Canny(imgThresholded, d,50,100,3);
		Canny(imgPoly, imgPoly, 50,100,3);




		// Finding contour for Green Circles

		vector <vector<Point>> contours;
		vector < Vec4i > hierarchy; 
		findContours(d, contours,hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


		

		// Finding contour for Triangle or Rectangle
		vector <vector<Point>> contoursPoly;
		vector < Vec4i > hierarchyPoly; 
		findContours(imgPoly, contoursPoly, hierarchyPoly, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


		// Finding centre of triangle or rectangle
		vector<Moments> muPoly(contoursPoly.size());
		float areaPoly = img.rows*img.cols/1000;
		int x_poly = 0, y_poly = 0, i_poly;
		for(int i=0; i< contoursPoly.size(); i++) {
			muPoly[i] = moments(contoursPoly[i]);
			if( contourArea(contoursPoly[i]) > areaPoly ) {
				Point2f p(muPoly[i].m01 / muPoly[i].m00, muPoly[i].m10 / muPoly[i].m00 );
				x_poly = p.x;
				y_poly = p.y;
				i_poly = i;
				areaPoly = contourArea(contoursPoly[i]);
			}
		
		}		

		


		cout<<"Area Poly Big = "<<areaPoly<<endl;
		// Drawing largest contour for polygon
		vector <vector<Point>> contoursPolyBig;
		contoursPolyBig.push_back(contoursPoly[i_poly]);

		cout<<"contoursPolyBig.size() = "<<contoursPolyBig.size()<<endl;

		drawContours(imgPoly, contoursPolyBig, -1, Scalar(232,112,114), 2, 8 );

		namedWindow("imgPoly", WINDOW_NORMAL);
		imshow("imgPoly", imgPoly);

		vector<Point> approx;
		approxPolyDP(Mat(contoursPoly[i_poly]), approx, arcLength(Mat(contoursPoly[i_poly]), true) * 0.01, true);
		cout<<"approx.size() = "<<approx.size()<<endl;

		//cout<<"contour size = "<<contours.size()<<endl;
		imshow("hsv", imgThresholded);
  		waitKey(1);

		//Drawing Contours for circle if circle found i.e contours size greater than zero

  		if(contours.size() > 0) {
			vector <vector<Point>> contourI;
			contourI.push_back(contours[0]);
			double AA = contourArea(contourI[0]);
			cout<<" area contour ka = "<<AA<<endl;



			Mat f(d.rows, d.cols, CV_8UC3, Scalar(0,0,0));
			drawContours(f, contours, -1, Scalar(232,112,114), 2, 8 );

			

			//imshow("WebCam",img);
			imshow("final_output",f);


			
			float Area = (img.rows*img.cols)/1200;
			cout<<"Area = "<<Area<<endl;

			//cout<<"X_coor = "<<X_coor<<" Y_coor = "<<Y_coor<<endl;

			// Removing element having area less than Area;
			for(auto i=contours.begin(); i!= contours.end(); i++) {
				cout<<"contourArea(*i) = "<<contourArea(*i)<<endl;
				if(contourArea(*i) < Area) {
					contours.erase(i);
				}
			}

		}
		vector<Moments> mu(contours.size());

		for(int i=0; i< contours.size(); i++) {
			
				mu[i] = moments(contours[i]);
				// Y_coor = (int)mu[i].m10 / mu[i].m00;
				// X_coor = (int)mu[i].m01 / mu[i].m00;
				// Area = contourArea(contours[i]);
			

		}
		int X_coor, Y_coor;

		X_coor = 0; Y_coor = 0; 
		int x_temp = 0, y_temp = 0;
		cout<<"contours.size() = "<<contours.size()<<endl;
		for(int i=0; i< contours.size(); i++) {
			Point2f p(mu[i].m01 / mu[i].m00, mu[i].m10 / mu[i].m00 );
			x_temp = p.x;
			y_temp = p.y;
			cout<<"x_temp = "<<x_temp<<" y_temp = "<<y_temp<<endl;
			if(x_temp > 9*(float)img.rows/10 )
				continue;
			if(x_temp > X_coor) {
				X_coor = x_temp;
				Y_coor = y_temp;
			}
		}

		cout<<"x_coor = "<<X_coor<<" y_coor = "<<Y_coor<<endl;
		int vert_centre_strip = img.cols/6;
		// Condition means next obstacle is polygon

		cout<<"x_poly = "<<x_poly<<" y_poly = "<<y_poly<<endl;

		cout<<"9*(img.rows/10) = "<<9*(img.rows/10)<<endl;

		if(contours.size() == 0) {
			cout<<"Stop"<<endl;
		}
		else if(x_poly > X_coor && x_poly < 9*(img.rows/10))  {
			if( y_poly > img.cols/2 + vert_centre_strip )  {
				// turn right;
				cout<<"D"<<endl;
			}
			else if( y_poly < img.cols/2 - vert_centre_strip) {
				// turn left;
				cout<<"A"<<endl;
			}
			else {
				// check triangle or rectangle
				vector<Point> approx;

				approxPolyDP(Mat(contoursPoly[i_poly]), approx, arcLength(Mat(contoursPoly[i_poly]), true) * 0.01, true);
				if(approx.size() == 3) {
					cout<<"Triangle Found"<<endl;
					while(0) {
						// turn left
						// move forward
						/*
						cap>>img
						pnt p;
						p = poly_coor(img);
						if(p.x == 0 && p.y == 0)
							break;
						if(p.x > 9*(img.rows/10) || p.y > img.cols-img.cols/8)
							break;

						*/
					}
				}
				else if(approx.size() == 4) {
					cout<<"Rectangle Found"<<endl;
					while(0) {
						// turn right
						// move forward
						cout<<"D and W"<<endl;
						/*
						cap>>img
						pnt p;
						p = poly_coor(img);
						if(p.x == 0 && p.y == 0)
							break;
						if(p.x > 9*(img.rows/10) || p.y < img.cols/8)
							break;

						*/
					}
				}
			
				// if triangle turn left and simultneously move forward till x_poly > 9*(img.rows/10)

				// else turn right and simultaneously move forward
			}
		}
		else {
		
			if( Y_coor > img.cols/2 + vert_centre_strip )  {
				// turn right;
				cout<<"D"<<endl;
			}
			else if( Y_coor < img.cols/2 - vert_centre_strip){
				// turn left;
				cout<<"A"<<endl;
			}
			else {
				// move forward;
				cout<<"W"<<endl;
			}
		}

		/*
		char x;
		float areaScreen = img.rows * img.cols;

		//cout<<"areaScreen = "<<areaScreen<<" threshold = "<<threshold<<endl;
		if(contours.size() == 0)
			cout<<"None"<<endl;
		else if(Area < (areaScreen/20) ) {
			if(X_coor > 2*(img.cols/3) ){
				x = 'D';
				//sendCommand(&x);
				cout<<"D"<<endl;
			}
			else if(X_coor > img.cols/3 && Y_coor < (float)2*img.cols/3  ) {
				x = 'W';
				//sendCommand(&x);
				cout<<"W"<<endl;
			}
			else if(X_coor < (int)2*img.cols/3 ) {
				x = 'A';
				//sendCommand(&x);
				cout<<"A"<<endl;
			}
			
		}
		else if(Area > (areaScreen/15) ){
			x = 'S';
			//sendCommand(&x);
			cout<<"S"<<endl;
		}

		*/


		imshow("hsv",imgThresholded);
		imshow("original",a); 
		// imshow("after_erosion",d); 
		//imshow("after_canny",d); 
		//imshow("final_output",f);


		waitKey(0);
		

		
	

		// namedWindow("WebCam",WINDOW_NORMAL);
		// 	namedWindow("Canny",WINDOW_NORMAL);

		// imshow("WebCam",img);
		// imshow("Canny",a);
		//waitKey(1);
	}
}