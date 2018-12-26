#include "sensors/camera.h"
#include "utils/utils.h"

using namespace std;

int flag_exit = 0;

// double getAbsoluteScale(int frame_id, int sequence_id, double z_cal){
//
// 	string line;
// 	int i = 0;
// 	ifstream myfile ("/home/avisingh/Datasets/KITTI_VO/00.txt");
// 	double x =0, y=0, z = 0;
// 	double x_prev, y_prev, z_prev;
// 	if (myfile.is_open()){
// 		while (( getline (myfile,line) ) && (i<=frame_id)){
// 			z_prev = z;
// 			x_prev = x;
// 			y_prev = y;
// 			std::istringstream in(line);
// 			//cout << line << '\n';
// 			for (int j=0; j<12; j++){
// 				in >> z ;
// 				if (j==7) y=z;
// 				if (j==3)  x=z;
// 			}
//
// 			i++;
// 		}
// 		myfile.close();
// 	}else{
// 		cout << "Unable to open file";
//
// 		return 0;
// 	}
//
// 	return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev));
// }

void my_handler(int s){
	printf("Caught signal %d\n",s);
	flag_exit = 1;
}

int main(int argc, char *argv[]){
	cv::Mat frame, frame_rect, image, image2, prev_img, cur_img, E, R, R_f, t_f, t, mask;
	vector<Point2f> prev_pts, cur_pts;
	vector<KeyPoint> keypts, prev_keypts, cur_keypts;
	vector<Point2f> points[2];
	bool addRemovePt = false;
	cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
	int numFrame = 0;
	float scale = 1.0;
	int MIN_NUM_FEAT = 10;
	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	float fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);

	attach_CtrlZ(my_handler);
	string dev = "/dev/video1";
	string pic = "/home/hunter/devel/robo-dev/data/camera/pic.jpg";
	string calib  = "/home/hunter/devel/robo-dev/config/sensors/camera/calibrationdata/ost.yaml";

	Camera cam(dev);
	cam.load_calibration(calib);

	cam.update();
	frame = cam.get_corrected_frame();
	prev_img = cam.greyscale_frame(frame);
	cam.detect_features(prev_img, prev_keypts, prev_pts);

	namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

	printf("Press Ctrl+Z to Stop...\r\n");
	while(!flag_exit){
		cv::Mat E, R, t, mask;
		vector<Point2f> cur_pts;

		// Start timer
        	double timer = (double)getTickCount();

		cam.update();
		frame = cam.get_corrected_frame();
		cur_img = cam.greyscale_frame(frame);
		cam.detect_features(cur_img, cur_keypts, cur_pts);

		cam.track_feautres(prev_img, cur_img, prev_pts, cur_pts);

		// cout << "			DEBUGGING:		1" << endl<<endl;
		cout << "Size pts1, pts2: " << prev_pts.size() << ", " << cur_pts.size() << endl;
		E = findEssentialMat(prev_pts, cur_pts, cam.focal, cam.pp, RANSAC, 0.999, 1.0, mask);
		// cout << "			DEBUGGING:		2" << endl<<endl;
		recoverPose(E, prev_pts, cur_pts, R, t, cam.focal, cam.pp, mask);
		// cout << "			DEBUGGING:		3" << endl<<endl;
		if(numFrame == 0){
			R_f = R.clone();
			t_f = t.clone();
		}


		// cv::Mat prev_fts(2,prev_pts.size(), CV_32F), curr_fts(2,cur_pts.size(), CV_32F);
		//
		// for(int i=0;i<prev_pts.size();i++){   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
		// 	prev_fts.at<float>(0,i) = prev_pts.at(i).x;
		// 	prev_fts.at<float>(1,i) = prev_pts.at(i).y;
		//
		// 	curr_fts.at<float>(0,i) = cur_pts.at(i).x;
		// 	curr_fts.at<float>(1,i) = cur_pts.at(i).y;
		// }

		// scale = getAbsoluteScale(numFrame, 0, t.at<float>(2));

		if((scale>0.1)&&(t.at<float>(2) > t.at<float>(0)) && (t.at<float>(2) > t.at<float>(1))){
			t_f = t_f + scale*(R_f*t);
			R_f = R*R_f;
		}else{
			cout << "scale below 0.1, or incorrect translation" << endl;
		}

		if(cur_pts.size() < (size_t)MIN_NUM_FEAT){
			//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
			cout << "trigerring re-detection" << endl;
			cam.detect_features(cur_img, cur_keypts, cur_pts);
			cam.track_feautres(prev_img, cur_img, prev_pts, cur_pts);

			// vector<Point2f> tmp;
               // tmp.push_back(point);
               // cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
               // points[1].push_back(tmp[0]);
               // addRemovePt = false;
		}

		// Calculate Frames per second (FPS)
        	float fps = getTickFrequency() / ((double)getTickCount() - timer);
		printf("Output FPS: %.3f\r\n",fps);

		drawKeypoints(frame,cur_keypts, image2, Scalar(255,0,0));
		// imshow("distorted", frame);
		imshow("rectified", image2);

		if (waitKey(10) == 27){
			cout << "Esc key is pressed by user. Stoppig the video" << endl;
			break;
		}
		std::swap(cur_pts, prev_pts);
		std::swap(cur_keypts, prev_keypts);
		cv::swap(prev_img, cur_img);
		numFrame++;

		int x = int(t_f.at<float>(0)) + 300;
		int y = int(t_f.at<float>(2)) + 100;
		circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

		rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<float>(0), t_f.at<float>(1), t_f.at<float>(2));
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

		imshow( "Trajectory", traj);

     }

     return 0;
}
