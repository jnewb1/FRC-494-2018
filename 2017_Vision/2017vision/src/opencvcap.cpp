#define TEAMNUM 14


#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>


#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <stdio.h>

#include "math.h"
using namespace cv;
using namespace std;


void FilterGreen(Mat &frame, int x, int y, int(&greenFiltered)[320][240]);
void objectScan(unsigned char* Dest, unsigned char* Source, int yMin, int yMax, int fillColor);
int setupSocket();
void UDCReceiverTest();
void UDCReceiverTestInit();
void UDCSendTestInit();
int sendUDCtorio(char *buf, int size);
int sendUDC(char *buf, int size);
void sendImage(unsigned char* imgRaw0, int size);
//void createBlob(int (&gF)[320][240]));

#define PORT 1131
#define PORT_SEND 1140
#define PORT_SENDtorio 1137
#define BUFLEN 8000
struct sockaddr_in   recipient;
struct sockaddr_in   riorecipient;
struct sockaddr_in si_me, si_other;
int hsocket;
int socketActive;
int CLOCKS_PER_SEC1 = CLOCKS_PER_SEC;
const clock_t begin_time = clock();
int lastUDC = 0;
int sendSocket;
int sendSocketrio;
int sendSocketActive = 0;
int sendSocketrioActive = 0;
int startTimeUDC = 0;
int startTimeUDCtorio = 0;
int sendMesCount = 0;
//const char* TeamDriveStationIP = "169.254.205.20";

const char* TeamDriveStationIP;
const char* TeamRioIP;



float rioBuf[400];



int maybePeg = 0;
int maybeGoal = 0;
int maybeCube = 0;

int bestGoalCenterX;
int bestGoalCenterY;
int bestGoalSize;
float bestPegAngle;
int bestPegCenterX;
int bestPegCenterY;
int bestPegSize;

int bestCubeCenterX;
int bestCubeCenterY;

int foundPeg;
int foundGoal;
int foundCube;

float selectedCam = 1;

float curFPS;

float cam1dr = 0;
float cam1dg = 8;
float cam2dr = 0;
float cam2dg = 8;

float lastcam1DriveMode = 1;

float cam1DriveMode = 1;




using namespace std;


//  CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds or video capture timestamp.
//  CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
//  CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
//  CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
//  CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
//  CV_CAP_PROP_FPS Frame rate.
//  CV_CAP_PROP_FOURCC 4-character code of codec.
//  CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
//  CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
//  CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
//  CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
//  CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
//  CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
//  CV_CAP_PROP_HUE Hue of the image (only for cameras).
//  CV_CAP_PROP_GAIN Gain of the image (only for cameras).
//  CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
//  CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
//  CV_CAP_PROP_WHITE_BALANCE_U The U value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
//  CV_CAP_PROP_WHITE_BALANCE_V The V value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
//  CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
//  CV_CAP_PROP_ISO_SPEED The ISO speed of the camera (note: only supported by DC1394 v 2.x backend currently)
//  CV_CAP_PROP_BUFFERSIZE Amount of frames stored in internal buffer memory (note: only supported by DC1394 v 2.x backend currently)

VideoCapture cap1("/dev/v4l/by-id/usb-KYE_Systems_Corp._USB_Camera_200901010001-video-index0");
VideoCapture cap2("/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0");

void setupcam1() {
	cap1.open(0);
	cap1.set(CAP_PROP_FRAME_WIDTH, 320); //1280); //320);
	cap1.set(CAP_PROP_FRAME_HEIGHT, 240); //720); //240);
	//cap1.set(CAP_PROP_BRIGHTNESS,0.1);
	cap1.set(CAP_PROP_FPS, 30);


}

void setupcam2() {
	/*
	cap2.open(1);
	cap2.set(CAP_PROP_FRAME_WIDTH,320); //1280); //320);
	cap2.set(CAP_PROP_FRAME_HEIGHT,240); //720); //240);
	//cap2.set(CAP_PROP_BRIGHTNESS,0.01);
	cap2.set(CAP_PROP_FPS,30);
	*/
}

//cv::VideoWriter video("out.mp4", -1 ,30,cv::Size(320,240));
//VideoWriter video("out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(320, 240), true);

int main(int, char**)
{
	printf("Starting!\n");
	//system("v4l2-ctl -d /dev/v4l/by-id/usb-KYE_Systems_Corp._USB_Camera_200901010001-video-index0 --set-ctrl brightness=100 --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=1000 --set-ctrl gamma=0");

	if (TEAMNUM == 70) {
		TeamDriveStationIP = "10.0.70.5";
		TeamRioIP = "10.0.70.2";
	}
	if (TEAMNUM == 494) {
		TeamDriveStationIP = "10.4.94.5";
		TeamRioIP = "10.4.94.2";
	}
	if (TEAMNUM == 14) {
		TeamDriveStationIP = "10.0.14.5";
		TeamRioIP = "10.0.14.2";
	}
	if (TEAMNUM == 77) {
		TeamDriveStationIP = "192.168.1.8";
		TeamRioIP = "10.0.14.2";
	}

	printf("Detected Team: %d\n", TEAMNUM);
	printf("TeamDriveStationIP: %s\n", TeamDriveStationIP);
	printf("TeamRioIP: %s\n", TeamRioIP);
	printf("OpenCV (main)\n");


	printf("Open Default Camera\n");




	setupcam1();
	setupcam2();


	//   cap.set(CV_CAP_PROP_GAIN, 0.5); // values range from 0 to 100
	   //cap1.set(CV_CAP_PROP_EXPOSURE, 100); //-1 is auto, values range from 0 to 100
	 //  cap.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 50); //values range from 0 to 100, -1 auto whitebalance
	 //  cap.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U,50); //values range from 0 to 100,  -1 auto whitebalance

	  // cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 5);

	if (!cap1.isOpened()) // check if we succeeded
	{
		printf("Open Camera 1 Failed\n");
	}
	else {
		printf("Open Camera 1 Succeeded\n");
	}
	if (!cap2.isOpened()) {
		printf("Open Camera 2 Failed\n");
	}
	else {
		printf("Open Camera 2 Succeeded");
	}
	//Mat edges;
	Mat cam1frame;
	Mat cam2frame;
	Mat frame;
	Mat frame2;
	//cap >> frame; // get a new frame from camera
	//Mat origFrame = frame;
	//    imwrite("y.jpg", origFrame);

	//char* rgbaframe = new char[frame.total()*4];
	//Mat continuousRGBA(frame.size(), CV_8UC4, rgbaframe);
	//cv::cvtColor(frame, continuousRGBA, CV_BGR2RGBA, 4);

	 //int greenFiltered[320][240]; 
	Mat dst;

	//unsigned char * ptr = &frame.at<cv::Vec3b>(0, 0)[0];

	//objectScan(ptr,ptr, 1, 239, 200);

	std::vector<uchar> buff;

	float count = 0;

	int quality = 200;
	int frameSize = 0;
	//cap >> frame;
	//cv::imwrite("x.jpg",frame);

	double lastClock = 0;
	double lastClock2 = 0;
	double curClock = 1;

	printf("Begginning Main Loop\n");
	while (true) {
		UDCReceiverTest();
		curClock = clock() / (float)CLOCKS_PER_SEC1;
		count++;
		if (count > 100) { count = 0; lastClock = curClock; }
		curFPS = 1.0 / (curClock - lastClock);

		lastClock = curClock;

		if (!cap1.isOpened()) { setupcam1(); }
		if (!cap2.isOpened()) { setupcam2(); }
		if (!cap1.isOpened() && !cap2.isOpened()) { continue; }

		if (selectedCam != 2 && selectedCam != 1) { selectedCam = 1; }

		if (selectedCam == 1 && !cap1.isOpened()) { selectedCam = 2; }
		if (selectedCam == 2 && !cap2.isOpened()) { selectedCam = 1; }


		if (selectedCam == 1 && cap1.isOpened()) { cap1 >> cam1frame; }
		if (selectedCam == 2 && cap2.isOpened()) { cap2 >> cam2frame; }

		//Size size(320,240);
		//resize(frame,frame,size);
		//FilterGreen(frame, 320, 240, greenFiltered);

		if (selectedCam == 1) { frame = cam1frame; }
		else if (selectedCam == 2) { frame = cam2frame; }
		else { continue; }


		//unsigned char * ptr = &frame.at<cv::Vec3b>(0, 0)[0];

		//objectScan(ptr, ptr, 1, 319, 255);

		int name = count;
		std::vector<int> param(2);
		param[0] = cv::IMWRITE_JPEG_QUALITY;
		param[1] = quality / 10.0;

		//cv::resize(frame, frame2, cv::Size(320, 240));
		cv::imencode(".jpeg", frame, buff, param);

		frameSize = buff.size();

		if (frameSize < 7500) quality += 10; else quality -= 10;

		if (quality < 100) quality = 100;
		if (quality > 1000) quality = 1000;

		//printf("Sending Image %d\n", buff.size());
		sendImage(reinterpret_cast<unsigned char*>(buff.data()), buff.size());
		//
		//printf("bestGoalCenterX: %d bestGoalCenterY: %d foundGoal: %d bestPegCenterX: %d bestPegCenterY: %d foundPeg: %d\n", bestGoalCenterX, bestGoalCenterY, foundGoal, bestPegCenterX, bestPegCenterY, foundPeg);

		rioBuf[0] = bestGoalCenterX;
		rioBuf[1] = bestGoalCenterY;
		rioBuf[2] = foundCube;
		rioBuf[6] = curFPS;


		sendUDCtorio((char *)rioBuf, 40);

	}

	//for (int x = 0; x<320*3; x+=3){
//	ptr[x] = 0; //blue
//	ptr[x+1] = 0; //green
//	ptr[x+2] = 255; //red
//}

/*
FilterGreen(frame, 320, 240, greenFiltered);
for (int y = 0; y<240; y++){
	for (int x = 0; x<320; x++){
		if(greenFiltered[x][y]){
		 frame.at<cv::Vec3b>(y,x)[2] = 255;
	 }
	}
}
*/

// createBlob(greenFiltered);
// imwrite("x.jpg", frame);

// namedWindow("greenfilter", WINDOW_AUTOSIZE);
//imshow("greenfilter", frame);



}



void FilterGreen(Mat &frame, int sizeX, int sizeY, int(&greenFiltered)[320][240])
{

	for (int y = 0; y < sizeY; y++) {
		for (int x = 0; x < sizeX; x++) {

			int b = frame.at<cv::Vec3b>(y, x)[0];
			int g = frame.at<cv::Vec3b>(y, x)[1];
			int r = frame.at<cv::Vec3b>(y, x)[2];

			int intensity = (b + g + r) / 3;

			int db = b - intensity;
			int dg = g - intensity;
			int dr = r - intensity;


			g = g * 10; if (g > 255) g = 255;


			//  if (r<50)
			  //{

			// frame.at<cv::Vec3b>(y,x)[0] = 0;
			// frame.at<cv::Vec3b>(y,x)[1] = 0;
			// frame.at<cv::Vec3b>(y,x)[2] = 0;

			// }

			if (dg < 11) goto reject;

			if (db > dg || dr > dg)
			{
			reject:

				//frame.at<cv::Vec3b>(y,x)[0] = 0;
				//frame.at<cv::Vec3b>(y,x)[1] = 0;
				//frame.at<cv::Vec3b>(y,x)[2] = 0;
				greenFiltered[x][y] = 0;
			}
			else {
				greenFiltered[x][y] = 1;
			}
		}
	}
}
/*
void createBlob(int (&gF)[320][240])
{
	int sx = 0;
	int sy = 0;

	int beg = 0;
	int end = 0;

	int blobx[2000];
	int bloby[2000];

	for (int y = 1; y<319; y++){
		for (int x = 1; x<239; x++){
			if gF[x][y]{
				blobx[0]=x;
				bloby[0]=y;
				if(gF[x+1][y]){
					end+=1;
					blobx[end] = x+1;
					bloby[end] = y;
			}
				if(gF[x-1][y]){
					end+=1;
					blobx[end] = x-1;
					bloby[end] = y;
			}
				if(gF[x][y+1]){
					end+=1;
					blobx[end] = x;
					bloby[end] = y+1;
			}
				if(gF[x][y-1]){
					end+=1;
					blobx[end] = x;
					bloby[end] = y-1;
			}
		}
	}
}
*/

#define bOff  0
#define gOff  1
#define rOff  2

int camWide = 320;
int camHigh = 240;

int zapX[100000];
int zapY[100000];


int xMaxBlobM[101];
int yMaxBlobM[101];
int xMinBlobM[101];
int yMinBlobM[101];

int bcIndex = 0;

int xMaxBlobC[101];
int yMaxBlobC[101];
int xMinBlobC[101];
int yMinBlobC[101];

int xMaxMaxYBlobC[101];
int xMinMaxYBlobC[101];

int xMaxMaxXBlobC[101];
int xMinMaxXBlobC[101];


int blobPixCountC[101];

int blobBotCount[101];
int blobTopCount[101];


int blobUsedC[101];
int blobUsedM[101];




int xMaxBlobG[101];
int yMaxBlobG[101];
int xMinBlobG[101];
int yMinBlobG[101];

int xMidBlobG[101];
int yMidBlobG[101];
int blobSizeG[101];



int gfMinLevel = 180;

bool quickFlag = false;

unsigned char scanGrid[320][241];
unsigned char linkGrid[320][241];

void objectScan(unsigned char* Dest, unsigned char* Source, int yMin, int yMax, int fillColor)
{

	int idx = 0;

	int minGreen = gfMinLevel; if (minGreen < 180) minGreen = 180;

	camWide = 320;
	camHigh = 240;

	memset(&scanGrid[0][0], 0, 77120);
	memset(&linkGrid[0][0], 0, 77120);

	//printf("ObjectScan (green Filter)\n");

	for (int Y = 0; Y < 240; Y++) {

		for (int X = 0; X < 320; X++) {

			unsigned char g = Source[idx + gOff];
			unsigned char b = Source[idx + bOff];
			unsigned char r = Source[idx + rOff];
			int bw = (r + g + b) / 3;

			int dg = g - bw;
			int dr = r - bw;
			int db = b - bw;

			// printf("cam1dr: %f cam1dg: %f cam2dr: %f cam2dg: %f\n", cam1dr, cam1dg, cam2dr, cam2dg);
			 /*
		  if(selectedCam==1){
			 if(db < dg && dr < cam1dr && dg>cam1dg)
				 {
				scanGrid[X][Y]=1;
			 }
		   }
		   else if(selectedCam==2){
			 if(db-20 < dg && dr < cam2dr && dg>cam2dg)
				 {
				scanGrid[X][Y]=1;
			 }
		   }
		   */
			if (r > 190 && r < 220 && g > 190 && g < 220 && b < 80 && b > 40)
			{
				scanGrid[X][Y] = 1;
			}


			idx += 3;

		};

	};


	//printf("ObjectScan (blob)\n");

	int XX, YY;

	for (int XX = 0; XX < 320; XX++) { scanGrid[XX][0] = 0; scanGrid[XX][238] = 0; };
	for (int XX = 0; XX < 320; XX++) { scanGrid[XX][1] = 0; scanGrid[XX][239] = 0; };

	for (int YY = 0; YY < 240; YY++) { scanGrid[0][YY] = 0; scanGrid[318][YY] = 0; };
	for (int YY = 0; YY < 240; YY++) { scanGrid[1][YY] = 0; scanGrid[319][YY] = 0; };

	int bobIndex = 0;
	int overflow = 0;

	int goodBlobIndex = 0;

	for (int sY = 2; sY < 238; sY++) {
		for (int sX = 2; sX < 318; sX++) {
			maybeGoal = 0;
			maybePeg = 0;

			maybeCube = 1;
			//printf("<--> X: %d Y: %d $d\n",sX,sY);

			if (scanGrid[sX][sY]) {

				int X = sX;
				int Y = sY;
				int zIdx = 0;
				int zScan = 0;

				zapX[zIdx] = X;
				zapY[zIdx] = Y;

			again:             //  printf("X: %d Y: %d\n",sX,sY);

				if (scanGrid[X - 1][Y]) { zIdx++; zapX[zIdx] = X - 1; zapY[zIdx] = Y; scanGrid[X - 1][Y] = 0; }
				if (scanGrid[X + 1][Y]) { zIdx++; zapX[zIdx] = X + 1; zapY[zIdx] = Y; scanGrid[X + 1][Y] = 0; }
				if (scanGrid[X][Y - 1]) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y - 1; scanGrid[X][Y - 1] = 0; }
				if (scanGrid[X][Y + 1]) { zIdx++; zapX[zIdx] = X; zapY[zIdx] = Y + 1; scanGrid[X][Y + 1] = 0; }

				zScan++;

				if (zScan <= zIdx) {

					X = zapX[zScan];
					Y = zapY[zScan];

					if (zScan < 19900 && zIdx < 19900) goto again;

				};
				/*
								 //printf("zIdx %d  X: %d Y: %d\n",zIdx,sX,sY);

						if (zIdx<10 || zIdx>19000 || overflow) maybeGoal = 0;

						if (zIdx<60) maybeGoal = 0;

						if (zIdx<10 || zIdx>19000 || overflow) maybePeg = 0;

						if (zIdx<10) maybePeg = 0;

				*/
				if (zIdx < 10) maybeCube = 0;



				// printf("Blob %d\n",zIdx);

				bobIndex++;

				int xMax = 0;
				int yMax = 0;
				int xMin = 320;
				int yMin = 240;

				for (int L = 0; L < zIdx; L++) {

					int x = zapX[L];
					int y = zapY[L];

					if (x > xMax) xMax = x;
					if (y > yMax) yMax = y;
					if (x < xMin) xMin = x;
					if (y < yMin) yMin = y;

				};


				int midX = xMin + (xMax - xMin) / 2;
				int midY = yMin + (yMax - yMin) / 2;

				int width = xMax - xMin;
				int height = yMax - yMin;
				float ratio = (double)width / (double)height;
				float density = zIdx / (float)(width * height);


				if (density < 0.3) maybeCube = 0;
				if (!maybeCube) { continue; }


				/*
				//gear
				if (density<0.1) maybeGoal = 0;

				if(ratio<1.5){maybeGoal = 0;}

			   // -- peg --


				if (density<0.1) maybePeg = 0;
				if (ratio > 10) { maybePeg = 0; }
				*/

				for (int L = 0; L <= zIdx; L++) {

					linkGrid[zapX[L]][zapY[L]] = bobIndex;

				};
				// printf("density %f ratio %f size %d\n", density, ratio, zIdx);

				if (!maybeGoal && !maybePeg) { continue; }


				//printf("Width:%d\n", width);
				//printf("Height:%d\n", height);
				//printf("Ratio:%f\n", ratio);

   /*

				if(width<5){continue;}
				if(height<3){continue;}
				if(ratio<2){continue;}

				//printf("w*h:%d\n", (width * height));

				float density = zIdx / (float)(width * height);

				//printf("Density:%f\n", density);

				if (density<0.6) continue;

   */


   // printf("goodBlobIndex %d\n", goodBlobIndex);

				xMaxBlobG[goodBlobIndex] = xMax;
				yMaxBlobG[goodBlobIndex] = yMax;
				xMinBlobG[goodBlobIndex] = xMin;
				yMinBlobG[goodBlobIndex] = yMin;
				xMidBlobG[goodBlobIndex] = midX;
				yMidBlobG[goodBlobIndex] = midY;
				blobSizeG[goodBlobIndex] = zIdx;

				if (goodBlobIndex < 100) goodBlobIndex++;


				/*
								 int bad=0;

								 for(int L = 0; L<zIdx; L++) {

									 int x = midX-zapX[L]; if (x<-5 || x>5) continue;
									 int y = midY-zapY[L]; if (y<-5 || y>5) continue;

									 bad=1;

									 break;

								 };

								 if (bad) continue;

								 //double avgX = (totalX / (double)zIdx)-xMin;
								 //double avgY = (totalY / (double)zIdx)-yMin;

								 //double dX=xMax-xMin+1;
								 //double dY=yMax-yMin+1;

								 //double levelX=avgX/dX;
								 //double levelY=avgY/dY;

								 //
									// DisplayValue(3, 0, "levelX ##.###",levelX, 15, 1, 4);
								 //    DisplayValue(4, 0, "levelY ##.###",levelY, 15, 1, 4);


								 int wide=xMax-xMin+1;
								 int high=yMax-yMin+1;

								  if (wide<2 || high<3) continue;

								 double density=zIdx/(double)(wide*high);


								 double ratio=wide/(double)high;

								 if (high<20 && wide<30) continue;

								  //if (ratio>3) continue;

								 // if (ratio<0.25) continue;

								  if (ratio<1.0 || ratio>3.0) continue;

								  int yLevel[320];

								 for (int L=xMin; L<=xMax; L++) yLevel[L]=0;

								//  for (int L=0; L<320; L++) yLevel[L]=0;

								 for (int L=0; L<zIdx; L++ ) {

									 int x=zapX[L];
									 int y=zapY[L];

									 if (y>yLevel[x]) yLevel[x]=y;

								 };
				*/

				int hit = 0;
				int begX = 0;
				int begY = 0;
				int endX = 0;
				int endY = 0;

				int bCnt = 0;
				/*
								 for (int L=xMin; L<midX; L++) {

									 int y=yLevel[L];

									 if ( !hit && y) { hit=1; begY=y; begX=L; }

									 if (!hit || !y) continue;

									 if (y>begY) {begY=y; begX=L; bCnt=0;} else {bCnt++; if (bCnt>10) break;}

								 };

								 hit=0;
								 bCnt=0;

								 for (int L=xMax; L>midX; L--) {

											 int y=yLevel[L];

											 if ( !hit && y) { hit=1; endY=y; endX=L; }

											 if (!hit || !y) continue;

											 if (y>endY) {endY=y; endX=L; bCnt=0;} else { bCnt++; if (bCnt>10) break; }

										 };


								 for (int L=xMin; L<=xMax; L++) {

										   linkGrid[L][yLevel[L]]=bcIndex+2;

								 };
								 */

								 // DisplayValue(3 + bcIndex, 0, "ratio ##.######", ratio, 15, 1, 3);

				xMaxBlobC[bcIndex] = xMax;
				yMaxBlobC[bcIndex] = yMax;
				xMinBlobC[bcIndex] = xMin;
				yMinBlobC[bcIndex] = yMin;

				xMaxMaxYBlobC[bcIndex] = endY;
				xMinMaxYBlobC[bcIndex] = begY;

				xMaxMaxXBlobC[bcIndex] = endX;
				xMinMaxXBlobC[bcIndex] = begX;



				blobPixCountC[bcIndex] = zIdx;

				if (bcIndex < 100) bcIndex++;

			edgemiss:  continue;

			};




		};
	};


	//printf("Blobs %d  \n",goodBlobIndex);


	//printf("Blob Filtering\n");
	float bestXError = 100;
	if (maybeCube) {
		foundCube = 1;
		printf("found Cube\n");

		for (int h = 0; h < goodBlobIndex; h++) {
			if (fabs((xMidBlobG[h] - 160)) < bestXError) {
				bestGoalCenterX = xMidBlobG[h];
				bestGoalCenterY = yMidBlobG[h];
				bestXError = fabs((xMidBlobG[h] - 160));
			}
		}
	}

	if (maybeGoal) {



		int goodBlobCount = 0;
		int topGoalX;
		int botGoalX;
		int topGoalY;
		int botGoalY;
		int centerX;
		int centerY;

		int goalSize;

		bestGoalSize = 0;
		foundGoal = 0;
		for (int p = 0; p < goodBlobIndex; p++) {
			for (int h = 0; h < goodBlobIndex; h++) {
				if (h != p) {
					if (yMidBlobG[h] >= yMidBlobG[p]) {
						topGoalX = xMidBlobG[h];
						botGoalX = xMidBlobG[p];
						topGoalY = yMidBlobG[h];
						botGoalY = yMidBlobG[p];
					}
					if (yMidBlobG[p] > yMidBlobG[h]) {
						topGoalX = xMidBlobG[p];
						botGoalX = xMidBlobG[h];
						topGoalY = yMidBlobG[p];
						botGoalY = yMidBlobG[h];
					}
					if (topGoalX - 10 < botGoalX && topGoalX + 10 > botGoalX) {
						foundGoal = 1;
						//BlobPairs[goodBlobCount].TopGoalMidX = xMidBlobG[p];
						//BlobPairs[goodBlobCount].BottomGoalMidX = xMidBlobG[h];
						//printf("Found Good Blob Pair"); printf("\n");

						//printf("TopGoalX: %d\n", topGoalX);
						//printf("BotGoalX: %d\n", botGoalX);
						//printf("TopGoalY: %d\n", topGoalY);
						//printf("BotGoalY: %d\n", botGoalY);
						centerX = (topGoalX + botGoalX) / 2;
						centerY = (topGoalY + botGoalY) / 2;
						goalSize = blobSizeG[h] + blobSizeG[p];


						if (goalSize > bestGoalSize) {
							bestGoalSize = goalSize;
							bestGoalCenterX = centerX;
							bestGoalCenterY = centerY;
						}





					}




				}



			}
		}
		maybeGoal = 0;
	}
	if (maybePeg) {

		int rightPegX;
		int leftPegX;
		int rightPegY;
		int leftPegY;

		int centerPegX;
		int centerPegY;
		int pegSize;
		int pegTapeDistance;


		bestPegSize = 0;
		foundPeg = 0;
		for (int p = 0; p < goodBlobIndex; p++) {
			for (int h = 0; h < goodBlobIndex; h++) {
				if (h != p) {
					if (xMidBlobG[h] <= xMidBlobG[p]) {
						rightPegX = xMidBlobG[h];
						leftPegX = xMidBlobG[p];
						rightPegY = yMidBlobG[h];
						leftPegY = yMidBlobG[p];
					}
					if (xMidBlobG[p] < xMidBlobG[h]) {
						rightPegX = xMidBlobG[p];
						leftPegX = xMidBlobG[h];
						rightPegY = yMidBlobG[p];
						leftPegY = yMidBlobG[h];
					}
					pegTapeDistance = (leftPegX - rightPegX);


					if (rightPegY - 30 < leftPegY && rightPegY + 30 > leftPegY && blobSizeG[h] * 6 / 3 > blobSizeG[p] && blobSizeG[h] * (1 / (6 / 3)) < blobSizeG[p]) {
						foundPeg = 1;
						//BlobPairs[goodBlobCount].TopPegMidX = xMidBlobG[p];
						//BlobPairs[goodBlobCount].BottomPegMidX = xMidBlobG[h];
						//printf("Found Good Blob Pair"); printf("\n");\


						//printf("Peg: %d\n", Peg);
						//printf("leftPegX: %d\n", leftPegX);
						//printf("rightPegY: %d\n", rightPegY);
						//printf("leftPegY: %d\n", leftPegY);
						centerPegX = (rightPegX + leftPegX) / 2;
						centerPegY = (rightPegY + leftPegY) / 2;
						pegSize = blobSizeG[h] + blobSizeG[p];
						if (centerPegY > 160) {
							pegSize = 0;
						}
						if (pegSize > bestPegSize) {
							bestPegSize = pegSize;
							bestPegCenterX = centerPegX;
							bestPegCenterY = centerPegY;
							bestPegAngle = std::atan((float)(rightPegY - leftPegY) / (float)(rightPegX - leftPegX));
						}





					}




				}



			}
		}
		maybePeg = 0;
	}




	if (quickFlag) return;

	for (int Y = 0; Y < 240; Y++) {

		for (int X = 0; X < 320; X++) {

			int idx = Y * 320 * 3 + X * 3;

			if (linkGrid[X][Y]) { // && (cam1DriveMode == 1 || selectedCam == 2)
				if ((Dest[idx + bOff] != 255 && Dest[idx + gOff] != 223 && Dest[idx + rOff] != 0) && (Dest[idx + bOff] != 223 && Dest[idx + gOff] != 0 && Dest[idx + rOff] != 255)) {
					Dest[idx + bOff] = 1;
					Dest[idx + gOff] = 1;   //if (linkGrid[X][Y]>1) Dest[idx+gOff]=40*linkGrid[X][Y];
					Dest[idx + rOff] = 200;
				}
			}

		};

	};
	if (foundPeg) {
		for (int x = (bestPegCenterX - 20); x <= bestPegCenterX + 20; x++) {
			for (int y = (bestPegCenterY - 2); y < (bestPegCenterY + 2); y++) {
				int idx = y * 320 * 3 + x * 3;
				Dest[idx + bOff] = 223;
				Dest[idx + gOff] = 0;
				Dest[idx + rOff] = 255;
			}

		}
		for (int y = (bestPegCenterY - 20); y <= bestPegCenterY + 20; y++) {
			for (int x = (bestPegCenterX - 2); x < (bestPegCenterX + 2); x++) {
				int idx = y * 320 * 3 + x * 3;
				Dest[idx + bOff] = 223;
				Dest[idx + gOff] = 0;
				Dest[idx + rOff] = 255;
			}
		}
	}
	if (foundGoal) {
		for (int x = (bestGoalCenterX - 20); x <= bestGoalCenterX + 20; x++) {
			for (int y = (bestGoalCenterY - 2); y < (bestGoalCenterY + 2); y++) {
				int idx = y * 320 * 3 + x * 3;
				Dest[idx + bOff] = 255;
				Dest[idx + gOff] = 223;
				Dest[idx + rOff] = 0;
			}

		}
		for (int y = (bestGoalCenterY - 20); y <= bestGoalCenterY + 20; y++) {
			for (int x = (bestGoalCenterX - 2); x < (bestGoalCenterX + 2); x++) {
				int idx = y * 320 * 3 + x * 3;
				Dest[idx + bOff] = 255;
				Dest[idx + gOff] = 223;
				Dest[idx + rOff] = 0;
			}
		}
	}
	if (foundCube) {
		for (int y = (bestGoalCenterY - 20); y <= bestGoalCenterY + 20; y++) {
			for (int x = (bestGoalCenterX - 2); x < (bestGoalCenterX + 2); x++) {
				int idx = y * 320 * 3 + x * 3;
				printf("centerX: %f centerY: %f\n", bestGoalCenterX, bestGoalCenterY);
				//Dest[idx + bOff] = 255;
				//Dest[idx + gOff] = 223;
				//Dest[idx + rOff] = 0;
			}
		}
	}


}; //objectScan

int setupSocket() {
	if ((hsocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("SocketError");
		return 0;
	}
	memset((char *)&si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(hsocket, (struct sockaddr *)&si_me, sizeof(si_me)) == 1) {
		printf("bidningerror");
		return 0;
	}
	socketActive = 1;
	return 1;
}

void UDCReceiverTest() {
	if (!socketActive) UDCReceiverTestInit();

	if (!socketActive) return;


	//int slen=sizeof(si_other);
	char buf[BUFLEN];

	memset((char *)&buf[0], 0, BUFLEN);

	//int result=recvfrom(hSocket, buf, BUFLEN, MSG_DONTWAIT, (struct sockaddr *)&si_other, &slen);

	struct sockaddr *from = 0;
	int len = sizeof(from);

	int result = recvfrom(hsocket, buf, BUFLEN, MSG_DONTWAIT, from, (socklen_t*)&len);

	//
		//printf(buf);
	if (result != -1)
	{
		//	printf("Robot::UDCReceiverTest %d\n",result);
		  // printf("Result %d\n",result);

		   //processCommands(buf,result);

		float *floatVal = (float*)&buf[0];

		//printf("%f\n",floatVal[0]);

		selectedCam = floatVal[0];
		cam1dr = floatVal[1];
		cam1dg = floatVal[2];
		cam2dr = floatVal[3];
		cam2dg = floatVal[4];
		cam1DriveMode = floatVal[5];


		if (lastcam1DriveMode != cam1DriveMode) {
			printf("switching drive cam mode\n");
			if (cam1DriveMode == 1) {
				//system("v4l2-ctl -d /dev/v4l/by-id/usb-KYE_Systems_Corp._USB_Camera_200901010001-video-index0 --set-ctrl brightness=-64 --set-ctrl exposure_auto=1 --set-ctrl exposure_absolute=200 --set-ctrl gamma=0");
			}

			if (cam1DriveMode == 2) {
				//system("v4l2-ctl -d /dev/v4l/by-id/usb-KYE_Systems_Corp._USB_Camera_200901010001-video-index0 --set-ctrl brightness=100 --set-ctrl exposure_auto=3 --set-ctrl exposure_absolute=1000 --set-ctrl gamma=0");
			}
			lastcam1DriveMode = cam1DriveMode;
		}




		lastUDC = float(clock() - begin_time) / CLOCKS_PER_SEC1;

	}
	else
	{

		if ((float(clock() - begin_time) / CLOCKS_PER_SEC1) - lastUDC > 2) //socket timeout (reset socket)
		{
			close(hsocket);

			socketActive = 0;

			lastUDC = float(clock() - begin_time) / CLOCKS_PER_SEC1;
		}
	}
	return;

	//close(hSocket); // unreachable with this simple example
}
void
UDCReceiverTestInit()
{

	//whm

	//return;

	if ((hsocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("Socket Error\n");
		return;
	}
	memset((char *)&si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("10.4.94.5"); //


	if (bind(hsocket, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
		printf("Binding Error\n");
		return;
	}

	socketActive = 1;

	// printf("UDCReceiverTestInit (%d)\n",(int)PORT);



}
void
UDCSendTestInit()
{

	//printf("UDCSendTestInit ( %d )\n",teamNum);

	//if (!alive) return;

	if ((sendSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("Socket Error\n");
		return;
	}
	memset((char *)&recipient, 0, sizeof(recipient));

	recipient.sin_family = AF_INET;
	recipient.sin_port = htons(PORT_SEND);

	//recipient.sin_addr.s_addr =inet_addr("10.4.94.5");


	//recipient.sin_addr.s_addr =inet_addr("10.0.14.5");

	//WHM



	//recipient.sin_addr.s_addr = inet_addr(TeamIP);

	recipient.sin_addr.s_addr = inet_addr(TeamDriveStationIP);


	//recipient.sin_addr.s_addr =inet_addr("10.0.70.5");

	//if (teamNum==494)
	//	recipient.sin_addr.s_addr = inet_addr("10.4.94.5");
	//if (teamNum== 70)
	//	recipient.sin_addr.s_addr = inet_addr("10.0.70.5");

	//if (teamNum== 14)
	//	recipient.sin_addr.s_addr = inet_addr("10.0.14.5");


	sendSocketActive = 1;

	startTimeUDC = float(clock() - begin_time) / CLOCKS_PER_SEC;

	//if (teamNum!=494 && teamNum!=70 && teamNum!=14)
	//{

		//DriverStation *ds = DriverStation::GetInstance();
			//ds->WaitForData();
			//teamNum = ds->GetTeamNumber();

	//}

	//printf("sendSocketActive %d sendMesCount %d\n",sendSocketActive,sendMesCount);

	if (sendMesCount == 0)
	{

		//if (teamNum==494)
			//printf("UDCSendTestInit( 494 )\n");

					//if (teamNum==70) printf("UDCSendTestInit( 70 )\n");
					//if (teamNum==14) printf("UDCSendTestInit( 14 )\n");

					//else printf("USCSendTestInit( Unknown! %d )\n",teamNum);
	}

	sendMesCount++; if (sendMesCount > 11) sendMesCount = 0;

}

int sendUDC(char *buf, int size)
{

	if (!sendSocketActive) UDCSendTestInit();

	double curTime = float(clock() - begin_time) / CLOCKS_PER_SEC;

	if (curTime - startTimeUDC > 10.0) { close(sendSocket); UDCSendTestInit(); }

	if (!sendSocketActive) return -1;

	if (size > 8192) size = 8192;

	int slen = sizeof(recipient);

	//printf("size %d slen %d\n",size,slen);

	int result = sendto(sendSocket, buf, size, 0, (struct sockaddr *)&recipient, slen);

	//printf("result %d\n",result);

	return result;
}
void
UDCSendTestInittorio()
{

	if ((sendSocketrio = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		printf("Socket Error\n");
		return;
	}
	memset((char *)&riorecipient, 0, sizeof(riorecipient));

	riorecipient.sin_family = AF_INET;
	riorecipient.sin_port = htons(PORT_SENDtorio);

	riorecipient.sin_addr.s_addr = inet_addr(TeamRioIP);


	sendSocketrioActive = 1;

	startTimeUDCtorio = float(clock() - begin_time) / CLOCKS_PER_SEC;

	sendMesCount++; if (sendMesCount > 11) sendMesCount = 0;

}

int sendUDCtorio(char *buf, int size)
{

	if (!sendSocketrioActive) UDCSendTestInittorio();

	double curTime = float(clock() - begin_time) / CLOCKS_PER_SEC;

	if (curTime - startTimeUDCtorio > 10.0) { close(sendSocketrio); UDCSendTestInittorio(); }

	if (!sendSocketrioActive) return -1;

	if (size > 8192) size = 8192;

	int slen = sizeof(riorecipient);

	int result = sendto(sendSocketrio, buf, size, 0, (struct sockaddr *)&riorecipient, slen);
	//printf("sent ");
	//printf("%d\n", result);
	return result;
}
/*
void UDCReceiverTest()
{

	if (!socketActive) UDCReceiverTestInit();

	if (!socketActive) return;


	//int slen=sizeof(si_other);
	char buf[BUFLEN];

	memset((char *) &buf[0], 0, BUFLEN);

	//int result=recvfrom(hSocket, buf, BUFLEN, MSG_DONTWAIT, (struct sockaddr *)&si_other, &slen);

	struct sockaddr *from=0;
	int len=sizeof(from);

	int result=recvfrom(hsocket, buf, BUFLEN, MSG_DONTWAIT, from,(socklen_t*) &len);

//

	if (result!=-1)
	{
	//	printf("Robot::UDCReceiverTest %d\n",result);
	  // printf("Result %d\n",result);

	   //processCommands(buf,result);

	   lastUDC=float( clock() - begin_time ) / CLOCKS_PER_SEC;

	}
	else
	{

	   if (float( clock() - begin_time ) / CLOCKS_PER_SEC-lastUDC > 2) //socket timeout (reset socket)
	   {
		   close(hsocket);

		   socketActive=0;

		   lastUDC=float( clock() - begin_time ) / CLOCKS_PER_SEC;
	   }
	}

	//close(hSocket); // unreachable with this simple example
}
*/
char tmpBuf[500000];

int imgRawSize0;
int imgRawSize1;
void sendImage(unsigned char* imgRaw0, int imgSize)
{

	unsigned char * imgRaw = imgRaw0;

	int ptr = 0;

	int i = 0;

	int payloadSize = 0;

	unsigned char payloadSizeLB = 0;
	unsigned char payloadSizeMB = 0;

	static int frameId = 0;

	//frameId++; if (frameId>10) frameId=0;

	for (int line = 119; line < 120; line++)
	{
		int size = imgSize; if (size > 8000) size = 8000;

		imgSize -= 8000;

		payloadSize = size;

		payloadSizeLB = payloadSize & 0xFF;
		payloadSizeMB = (payloadSize & 0xFF00) >> 8;

		//printf("%d %d\n",line,payloadSize);

		i = 0;

		tmpBuf[i++] = 1;
		tmpBuf[i++] = 19;
		tmpBuf[i++] = 2;
		tmpBuf[i++] = 105 + frameId;
		tmpBuf[i++] = (unsigned char)line;
		tmpBuf[i++] = (char)payloadSizeLB;
		tmpBuf[i++] = (char)payloadSizeMB;

		for (int M = 0; M < payloadSize; M++)
		{
			tmpBuf[i++] = imgRaw[ptr++];
		}

		sendUDC(tmpBuf, i);

		if (imgSize <= 0) break;
	}

	i = 0;

	payloadSize = 400;

	payloadSizeLB = payloadSize & 0xFF;
	payloadSizeMB = (payloadSize & 0xFF00) >> 8;

	tmpBuf[i++] = 1;
	tmpBuf[i++] = 19;
	tmpBuf[i++] = 2;
	tmpBuf[i++] = 103;
	tmpBuf[i++] = (unsigned char)255; //254;
	tmpBuf[i++] = (char)payloadSizeLB;
	tmpBuf[i++] = (char)payloadSizeMB;

	int size = 400;

	for (int M = 0; M < 400; M++)
	{
		tmpBuf[i++] = 0;
	}

	sendUDC(tmpBuf, i);

}







