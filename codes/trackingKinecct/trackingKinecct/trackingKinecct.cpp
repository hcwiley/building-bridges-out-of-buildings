// Jim Vaughn
// Displays the color and depth information using OpenCV

#include "stdafx.h"

#include "cv.h"
#include "highgui.h"

#include "CLNUIDevice.h"


using namespace std;

IplImage* imgRedThresh;// = cvCreateImageHeader(cvSize(640, 480), 8, 3);
IplImage* frame;// = cvCreateImageHeader(cvSize(640, 480), 8, 4);
IplImage* depth32;
IplImage* depth0;// = cvCreateImage(cvGetSize(frame), 8, 4);
//Model image stuff
int imgNum;
char imagePath[20];
int numImages;
double normalX;
IplImage* images[24];
IplImage* curImage;

time_t prevTime, curTime;

float thresh1 = 0.0;
float thresh2 = 0.0;
float thresh3 = 0.0;

static double redArea = 0.0;
// showVideo, 1 to show the windows with the videos
int showVideo = 1;
int depthThresh = 0;

void GetRedThresholdedImage() {
	// Convert the image into an HSV image
    IplImage* imgHSV = cvCreateImage(cvGetSize(frame), 8, 3);
    cvCvtColor(depth32, imgHSV, CV_RGB2HSV);
	
	cvReleaseImage(&imgRedThresh);
	imgRedThresh = cvCreateImage(cvGetSize(frame), 8, 1);
	
	cvInRangeS(imgHSV, cvScalar(thresh1,thresh2,thresh3), cvScalar(1.0, 1.0, 1.0), imgRedThresh);

	cvReleaseImage(&imgHSV);

}
//IplImage* trackingRed(){
void trackingRed(){
	// If this is the first frame, we need to initialize it
    /*
	if(imgScribble == NULL)
    {
        imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
	}
	*/
	// Holds the red thresholded image (red = white, rest = black)
    //GetRedThresholdedImage();

	// Calculate the moments to estimate the position of the ball
	//cvInRange(depth0, 0, c ,depth0);
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(depth0, moments, 1);
 
    // The actual moment values
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    double area = cvGetCentralMoment(moments, 0, 0);

	// Holding the last and current ball positions
    static int posX = 0;
    static int posY = 0;
	double color = 0;
 
    int lastX = posX;
    int lastY = posY;
	redArea = area;
 
    posX = moment10/area;
    posY = moment01/area;
	//posX -= 320;
	//posY -= 240;
	//posX *= 10;
	//posY *= 10;
	//area -= 300;

	color = (abs((lastX - posX))+abs((lastY - posY)))/2;

	color = (color/80)*255;

	if(abs(lastX - posX) > 5 && abs(lastY - posY) > 5 && abs(area)/2> 10){
			// Print it out for debugging purposes
		printf("red: %d,%d,%f,%f\n", posX, posY, abs(area), color);
		cvRectangle(frame, cvPoint(posX, posY), cvPoint(posX + area/2, posY+ area/2), cvScalar(0,0,255) , 0);
	}
	if(showVideo == 1)
		cvShowImage("redThresh", imgRedThresh);
	// Release the moments... we need no memory leaks.. 
	delete moments;
	//return imgScribble;
}
void changeImage(int dir){
	if(dir + imgNum > numImages){
		imgNum = 0;
	}
	else if(dir + imgNum < 1){
		imgNum = numImages;
	}
	else{
		imgNum += dir;
	}
	curImage = images[imgNum];
	cvShowImage("image",curImage);
}

int main(){
	//data for kinect rgb and depth
	PDWORD rgb32_data = (PDWORD) malloc(640*480*4);
    PDWORD depth32_data = (PDWORD) malloc(640*480*4);
	depth32 = cvCreateImageHeader(cvSize(640,480), 8, 4);
	frame = cvCreateImageHeader(cvSize(640,480), 8, 4);
	depth0 = cvCreateImage(cvSize(640,480), 8, 1);

	// Initialize capturing live feed from the camera
    CLNUICamera capture = CreateNUICamera(GetNUIDeviceSerial(0));
    //CLNUIMotor motor = CreateNUIMotor(GetNUIDeviceSerial(0));
	// Couldn't get a device? Throw an error and quit
    if(!capture)
    {
        printf("Could not initialize capturing...\n");
        return -1;
    }
	StartNUICamera(capture);
	// The windows
    if(showVideo == 1){
		cvNamedWindow("video", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("depth", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("depth0", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("redThresh", CV_WINDOW_AUTOSIZE);
		//cvNamedWindow("greenThresh", CV_WINDOW_AUTOSIZE);
	}
	imgNum = 1;
	numImages = 24;
	normalX = 0.0;
	cvNamedWindow("image", CV_WINDOW_AUTOSIZE);
	*images = new IplImage[24];
	for(int i =0; i < numImages; i++){
		images[i] = cvCreateImage(cvSize(1280,1024),8,3);
		sprintf(imagePath,"..\\images\\%d.png", i+1);
		images[i] = cvLoadImage(imagePath);
	}
	
	curImage = images[0];
	cvShowImage("image",curImage);
	// An infinite loop
	//time(&prevTime);
    while(true)
    {
        // Will hold a frame captured from the 
        GetNUICameraColorFrameRGB32(capture, rgb32_data);
		GetNUICameraDepthFrameRGB32(capture, depth32_data);
		
		cvSetData(frame, rgb32_data, frame->widthStep);
        cvSetData(depth32, depth32_data, depth32->widthStep);
		// If we couldn't grab a frame... quit
        if(!frame)
            break;
		cvFlip(frame, frame, 1);
		cvCvtColor(depth32, depth0, CV_RGB2GRAY);
		trackingRed();

		// Wait for a keypress
        int c = cvWaitKey(10);

		time(&curTime);
		if(difftime(curTime, prevTime) > .005){//every .05 seconds
			time(&prevTime);
			changeImage(1);
		}
		
		if(c==27){
			// If pressed, break out of the loop
			if(showVideo == 1){
				cvDestroyWindow("video");
				cvDestroyWindow("depth");
				cvDestroyWindow("depth0");
				cvDestroyWindow("redThresh");
				cvDestroyWindow("image");
			}
			for(int i =0; i < numImages; i++){
				cvReleaseImage(&images[i]);
			}
			cvReleaseImage(&imgRedThresh);
			cvReleaseImage(&depth0);
			free(rgb32_data);
			free(depth32_data);
			cvReleaseImageHeader(&frame);
			cvReleaseImageHeader(&depth32);
			break;
		}
		else if(c==32){
			//cvCopy(depth32, depth0);
		}
		else if(c==49){
			thresh1+=0.1;
			printf("%f,%f,%f\n", thresh1,thresh2,thresh3);
		}
		else if(c==50){
			thresh1-=0.1;
			printf("%d,%d,%d\n", thresh1,thresh2,thresh3);
		}
		else if(c==51){
			thresh2+=0.1;
			printf("%d,%d,%d\n", thresh1,thresh2,thresh3);
		}
		else if(c==52){
			thresh2-=0.1;
			printf("%d,%d,%d\n", thresh1,thresh2,thresh3);
		}
		else if(c==53){
			thresh3+=0.1;
			printf("%d,%d,%d\n", thresh1,thresh2,thresh3);
		}
		else if(c==54){
			thresh3-=0.1;
			printf("%d,%d,%d\n", thresh1,thresh2,thresh3);
		}
		else if(c!=-1){
			printf("%d",c);
		}
		if(showVideo == 1){
			cvShowImage("video", frame);
			cvShowImage("depth", depth32);
			cvShowImage("depth0", depth0);
		}
	}
	// We're done using the camera. Other applications can now use it
    //cvReleaseCapture(&capture);
	return 0;
}