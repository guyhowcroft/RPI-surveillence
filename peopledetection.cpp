#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "/home/pi/git/robidouille/raspicam_cv/RaspiCamCV.h"
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <cmath>

using namespace cv;
using namespace std;

/*GLOBAL VARIABLES*/
int threadDone=1;
int servo0Pos;
int servo1Pos;
int send = 0;
int countMove=0;
Rect lastRect;
int reInitialise = 1;
int initialiseCount = 0;
int newPerson =1;
Mat trackHist;
int histcount=0;
int preServo0pos=0;
int preServo1pos=0;
Point nextPredict;
int cameraMoved =0;
Point oldPredict;
int whatMove = 0;
int showPanoramaExits =0;
			//y    x
//if i change this need to change the make zero function
int panorama[433][577];//added one more just in case
/*----------------*/

struct thread_data{
	Mat img;
	KalmanFilter KF;
};

Point kalmanPredict(KalmanFilter &KF);
Point kalmanCorrect(KalmanFilter &KF, Rect &trackRect);
 
int moveServo(int servo){
	int position;
	if(servo ==0){
		position = servo0Pos;
	}else{
		position = servo1Pos;
	}
	//set the servo position
	FILE * servoDev;
	printf("moving the servo number %d to: %d\n\n",servo,position);
   	servoDev=fopen("/dev/servoblaster","w");
   	fprintf(servoDev, "%d=%d\n",servo,position);
   	fflush(servoDev);
    fclose(servoDev);
	return 0;
}

int moveCamera(Rect &trackRect, Mat &img, KalmanFilter &KF){

	int imgMiddleX,imgMiddleY,rectMiddleX,rectMiddleY,moveY,moveX;

	//if 0 then center, if 1 then edge
	int moveFlag = whatMove;

	//servo:pixel
	double ratioX = 30.0;
	double ratioY = 24.0;

	printf("trackRect = (%d,%d)\n",trackRect.tl().x,trackRect.tl().y);

	//get the center of the rect box
	rectMiddleX = trackRect.tl().x + (trackRect.br().x - trackRect.tl().x)/2;
	rectMiddleY = trackRect.tl().y + (trackRect.br().y - trackRect.tl().y)/2;
	//get the centre of the image
	imgMiddleX = img.cols/2;
	imgMiddleY = img.rows/2;

	int centerRange=5;

	//before updating the movement of the servos, save the last movement
	preServo0pos=servo0Pos;
	preServo1pos=servo1Pos;

	//center the image
	if (moveFlag ==0){

		//compare the center of frame to center of Rect
		moveY = rectMiddleY-imgMiddleY;
		moveX = rectMiddleX-imgMiddleX;
		//printf("move x= %d, move y= %d\n",moveX,moveY);
		
		//if center of rect and image arent the same then calculate the move
		//x
		if(moveX>centerRange||moveX<-centerRange){
				//move right by decreasing- ratio of pixels to servo movement
				//move left my increasing (plus 0.5 for rounding errors)
				servo0Pos = (servo0Pos -  (moveX*(ratioX/imgMiddleX)))+0.5;
		}
		//y
		if(moveY>centerRange||moveY<-centerRange){
				//move down by increasing, up by decreasing
				servo1Pos = (servo1Pos + (moveY*(ratioY/imgMiddleY)))+0.5;
		}

	//edge the image
	}else{

		//if the first rect for a while
		if(newPerson == 1){

			//calculate whether the rect is left or right side of the image
			//x far right of image so person going to left
			if(rectMiddleX>imgMiddleX){

				printf("new person at right of image !: move to the left\n");
				//move left - distance between the rect br().x and the right hand edge of img
				servo0Pos = (servo0Pos + ((img.cols - trackRect.br().x)*(ratioX/imgMiddleX)))+0.5;
			}else{

				//x far left of image so person going to right
				printf("new person at left of image !: move to the right\n");
				//move right - distance between the rect tl().x and the left hand edge of img
				servo0Pos = (servo0Pos - (trackRect.tl().x*(ratioX/imgMiddleX)))+0.5;
			}

			//calculate whether the rect is up or down side of the image
			//y is up
			if(rectMiddleY>imgMiddleY){

				//move up - destance between the rect br().y and the bottom of the img
				servo1Pos = (servo1Pos - ((img.rows - trackRect.br().y)*(ratioY/imgMiddleY)))+0.5;
			}else{

				//y is down
				//move down - distance between rect tl().y and the top edge of the img
				servo1Pos = (servo1Pos +(trackRect.tl().y* (ratioY/imgMiddleY))) +0.5;
			}
			newPerson =0;
		}else{

			//DONT NEED IT HERE- maybe we do
			printf("predicting because want to calculate distance for camera movement\n");
			Point kPredict = kalmanPredict(KF);
			//most up to date prediction as it has just corrected to get here to move
			//Point kPredict = nextPredict;

			//distance between top left prediction and top left real
			// moveY = trackRect.tl().y - kPredict.y;
			// moveX = trackRect.tl().x - kPredict.x;

			printf("oldPredict = (%d,%d) kPredict = (%d,%d)\n",oldPredict.x,oldPredict.y,kPredict.x,kPredict.y);
			moveY = oldPredict.y - kPredict.y;
			moveX = oldPredict.x - kPredict.x;


			//printf("distance between predict and rect in x = %d\n distance between predict and rect in Y = %d \n",moveX,moveY);

			//prediction is to the left
			if(moveX>0){
				printf("move to the left\n");
				//move left - distance between the rect br().x and the right hand edge of img
				servo0Pos = (servo0Pos + ((img.cols - trackRect.br().x)*(ratioX/imgMiddleX)))+0.5;
			}
			//prediction is to the right
			if(moveX<0){
				printf("move to the right\n");
				//move right - distance between the rect tl().x and the left hand edge of img
				servo0Pos = (servo0Pos - (trackRect.tl().x*(ratioX/imgMiddleX)))+0.5;
			}
			//prediction is up
			if(moveY>0){
				//move up - destance between the rect br().y and the bottom of the img
				servo1Pos = (servo1Pos - ((img.rows - trackRect.br().y)*(ratioY/imgMiddleY)))+0.5;
			}
			//prediction is down
			if(moveY<0){
				//move down - distance between rect tl().y and the top edge of the img
				servo1Pos = (servo1Pos +(trackRect.tl().y* (ratioY/imgMiddleY))) +0.5;
			}
		}
		
	}

	//if the servo pos is a position which is not available set to the least or max
	if(servo0Pos>240)servo0Pos=240;
	if(servo0Pos<60)servo0Pos=60;
	if(servo1Pos>240)servo1Pos=240;
	if(servo1Pos<60)servo1Pos=60;
	//finally move the servos
	moveServo(0);
	moveServo(1);

	return 0;
}

Point kalmanPredict(KalmanFilter &KF){

    Mat prediction = KF.predict();
    Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

    KF.statePre.copyTo(KF.statePost);
    KF.errorCovPre.copyTo(KF.errorCovPost);

   	printf("Kalman prediction = (%d,%d)\n", predictPt.x,predictPt.y);
    return predictPt;
}

Point kalmanCorrect(KalmanFilter &KF, Rect &trackRect){

	//point converted to the panorama
	int pointX,pointY;
	pointX = ((240- servo0Pos)*(192/60)) + trackRect.tl().x;
	pointY = ((servo1Pos - 60)*(144/48)) + trackRect.tl().y;

	Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
	// First predict, to update the internal statePre variable
	printf("predicting because of Correction\n");
	//printf("correcting with (%d,%d)\n",trackRect.tl().x,trackRect.tl().y);
	printf("CORRECTING with (%d,%d)\n",pointX,pointY);

	kalmanPredict(KF);
	measurement(0) = pointX;
	measurement(1) = pointY; 
    Mat estimated = KF.correct(measurement);
    Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	//printf("correcting measurement =  (%f,%f)\n",measurement(0),measurement(1));
   	printf("Kalman Correction = (%d,%d)\n", statePt.x,statePt.y);
    return statePt;
}

void initKalman(int x, int y, KalmanFilter &KF){
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	KF.statePre.setTo(0);
    KF.statePre.at<float>(0, 0) = x;
    KF.statePre.at<float>(1, 0) = y;
    KF.statePost.setTo(0);
    KF.statePost.at<float>(0, 0) = x;
    KF.statePost.at<float>(1, 0) = y; 
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	printf("reinitialising KalmanFilter \n");
}

void calcHist(Mat &hist,Mat &img, Rect &trackRect){
	
	Rect r = trackRect;
	r.x += cvRound(r.width*0.3);
	r.width = cvRound(r.width*0.5);
   	r.y += cvRound(r.height*0.2);
    r.height = cvRound(r.height*0.5);

	Rect imgBounds(0,0,img.cols,img.rows);
	//trackRect = trackRect & imgBounds;
	r = r & imgBounds;
	//copy the ROI to a matrix
	Mat croppedImg = img(r);

	int imgCount = 1;
	int dims = 3;
	const int sizes[] = {8,8,8};
	const int channels[] = {0,1,2};
	float rRange[] = {0,256};
	float gRange[] = {0,256};
	float bRange[] = {0,256};
	const float *ranges[] = {rRange,gRange,bRange};
	Mat mask = Mat();
	calcHist(&croppedImg, imgCount, channels, mask, hist, dims, sizes, ranges);
	hist.convertTo(hist, CV_32F);

	//imshow("histogram", croppedImg);
}

//shrinks rectangles and draws them
void drawRect(Mat &img, Rect rect,int b, int g, int red){
	Rect r = rect;
	r.x += cvRound(r.width*0.1);
	r.width = cvRound(r.width*0.8);
   	r.y += cvRound(r.height*0.07);
    r.height = cvRound(r.height*0.8);
    //draws rectangle, scalar defines colour
    rectangle(img, r.tl(), r.br(), Scalar(b,g,red), 3);
}

void makeZero(){
	for (int i=0; i<432; i++){
    	for (int j=0; j<576; j++)
    	{
      		panorama[i][j]=0;
    	}
    }
}

//put exit in to panorama
void calcPanorama(){

	//when doing this, the coord is in the image plane - so this is correct,not the same as when 
	//checking if the coord is in the image plane

	int x = lastRect.tl().x;
	int y = lastRect.tl().y;
	int cameraPosX;
	int cameraPosY;
	int distanceOverlapX;
	int distanceOverlapY;
	int newX=0;
	int newY=0;
	//section of the panorama- 1,2,3
	int sectionX=0;
	int sectionY=0;
	//camera hasnt moved since last found so take current position of camera
	if(cameraMoved == 0){
		printf("panorama: camera HASN'T moved\npanorma: using servoPos (%d,%d)\n",servo0Pos,servo1Pos);
		cameraPosX = servo0Pos;
		cameraPosY = servo1Pos;
	//camera has moved
	}else{
		printf("panorama: camera HAS moved\npanorama: using servoPos (%d,%d)\n",preServo0pos,preServo1pos);
		cameraPosX = preServo0pos;
		cameraPosY = preServo1pos;
	}

	printf("panorama: Point of loss (%d,%d)\n",x,y);

	//between sevo position 210 and 150 in X
	if(cameraPosX<=210 && cameraPosX>150){
		distanceOverlapX = (cameraPosX - 150) * (192.0/60);
		//coord is in the left section
		if(x<distanceOverlapX){
			sectionX = 1;
			newX = (192-distanceOverlapX)+ x;
		//coord is in the right section
		}else{
			sectionX = 2;
			newX = x - distanceOverlapX;
		}
	}
	//between sevo position 150 and 90 in X
	if(cameraPosX<=150 && cameraPosX>=90){
		distanceOverlapX = (cameraPosX - 90) * (192.0/60);
		//coord is in the left section
		if(x<distanceOverlapX){
			sectionX = 2;
			newX = (192-distanceOverlapX)+ x;
		//coord is in the right section
		}else{
			sectionX = 3;
			newX = x - distanceOverlapX;
		}
	}

	//between servo position 102 and 150 in Y
	if(cameraPosY>=102 && cameraPosY<150){
		distanceOverlapY = (cameraPosY - 102) * (144.0/48);
		//coord is in the bottom section
		if(y>=(144-distanceOverlapY)){
			sectionY = 2;
			newY = y - (144- distanceOverlapY);
		//coord is in the up section
		}else{
			sectionY = 1;
			newY = y + distanceOverlapY;
		}
	}
	//between servo position 150 and 198 in Y
	if(cameraPosY>=150 && cameraPosY<198){
		distanceOverlapY = (cameraPosY - 150) * (144.0/48);
		//coord is in the bottom section
		if(y>=(144-distanceOverlapY)){
			sectionY = 3;
			newY = y - (144- distanceOverlapY);
		//coord is in the up section
		}else{
			sectionY = 2;
			newY = y + distanceOverlapY;
		}
	}

	//update the panoramas correct section
	if(sectionX == 1 && sectionY ==1)panorama[newY][newX] += 1; 
	if(sectionX == 2 && sectionY ==1)panorama[newY][newX+192] += 1;
	if(sectionX == 3 && sectionY ==1)panorama[newY][newX+384] += 1;
	if(sectionX == 1 && sectionY ==2)panorama[newY+144][newX] += 1; 
	if(sectionX == 2 && sectionY ==2)panorama[newY+144][newX+192] += 1;
	if(sectionX == 3 && sectionY ==2)panorama[newY+144][newX+384] += 1;
	if(sectionX == 1 && sectionY ==3)panorama[newY+288][newX] += 1; 
	if(sectionX == 2 && sectionY ==3)panorama[newY+288][newX+192] += 1;
	if(sectionX == 3 && sectionY ==3)panorama[newY+288][newX+384] += 1;

	/*//gaussian code
	if(sectionX == 1 && sectionY ==1){newX = newX; newY=newY;} 
	if(sectionX == 2 && sectionY ==1){newX = newX+192; newY=newY;} 
	if(sectionX == 3 && sectionY ==1){newX = newX+384; newY=newY;} 
	if(sectionX == 1 && sectionY ==2){newX = newX; newY=newY+144;} 
	if(sectionX == 2 && sectionY ==2){newX = newX+192; newY=newY+144;} 
	if(sectionX == 3 && sectionY ==2){newX = newX+384; newY=newY+144;}
	if(sectionX == 1 && sectionY ==3){newX = newX; newY=newY+288;}
	if(sectionX == 2 && sectionY ==3){newX = newX+192; newY=newY+288;}
	if(sectionX == 3 && sectionY ==3){newX = newX+384; newY=newY+288;}

	panorama[newY][newX-1]+=0.6;*/

	printf("panorama: position in panorama y,x (%d,%d)\n", newY, newX);
}

//moves the servos to the position of the best exit
void getExit(){

	int bestExitI =0;//y
	int bestExitJ =0;//x
	int sectionX=0;
	int sectionY=0;
	int servoX =0;
	int servoY =0;
	int bestExit =0;

	for (int i=0; i<432; i++){
    	for (int j=0; j<576; j++)
    	{
    		//		y   x
      		if(panorama[i][j] > bestExit){
      			bestExit = panorama[i][j];
      			bestExitI = i;
      			bestExitJ = j;
      		}
    	}
    }

    //printf("best exit has value %d\n",bestExit);
    //printf("best exit i = %d, best exit j = %d\n",bestExitI,bestExitJ );

    //if there has been an exit recognised
    if(bestExit!=0){
	    //get x section
	    if(bestExitJ<=192 && bestExitJ>-1)sectionX =1;
	    if(bestExitJ<=384 && bestExitJ>192)sectionX =2;
	    if(bestExitJ<=576 && bestExitJ>384)sectionX =3;
	    //get y section
	    if(bestExitI<=144 && bestExitI>-1)sectionY =1;
	    if(bestExitI<=288 && bestExitI>144)sectionY =2;
	    if(bestExitI<=432 && bestExitI>288)sectionY =3;

	    //calculate the position of the servo x
	    //if(sectionX ==1)servoX = 210 + (((96-(bestExitJ+lastRect.width))*(30/96))+0.5);//old one
	    if(sectionX ==1)servoX = 240 - (((bestExitJ + (lastRect.width/2.0))*(30.0/96))+0.5);
   	    if(sectionX ==2)servoX = 180 - ((((bestExitJ + (lastRect.width/2.0))-192)*(30.0/96))+0.5);
   	    if(sectionX ==3)servoX = 120 - ((((bestExitJ + (lastRect.width/2.0))-384)*(30.0/96))+0.5);

	    //calculate the position of the servo y
	    //if(sectionY ==1)servoY = 102 - (((72-(bestExitI+lastRect.height))*(24/72))+0.5);//old one
   	    if(sectionY ==1)servoY = 78 + (((bestExitI +(lastRect.height/3.0))*(24.0/72))+0.5);
   	    if(sectionY ==2)servoY = 126 + ((((bestExitI +(lastRect.height/3.0))-144)*(24.0/72))+0.5);
	    if(sectionY ==3)servoY = 174 + ((((bestExitI + (lastRect.height/3.0))-288)*(24.0/72))+0.5);
	    

		//printf("sectionX = %d, sectioY = %d\n",sectionX,sectionY);
		 //printf("calculation = %f\n",lastRect.height/2);
		//printf("Rect.width = %d, Rect.height = %d\n",lastRect.width,lastRect.height);
	    //printf("servoY= %d, servoX = %d\n",servoY,servoX);
	

	    if(servoX>240)servoX=240;
		if(servoX<60)servoX=60;
		if(servoY>240)servoY=240;
		if(servoY<60)servoY=60;

		//change the pre servo pos to the same as the servo pos becasue otherwise, if detection are moved to exit,
		// it will look at teh previous one before exit and use this with the correct on the new detection
		servo0Pos = servoX;
		servo1Pos = servoY;
		preServo0pos = servo0Pos;
		preServo1pos = servo1Pos;

		printf("get exit: position in panorama y,x (%d,%d)\n", bestExitI,bestExitJ);

		//move to exit
		moveServo(0);
		moveServo(1);
	}else{
		printf("no exit found so far!\n");
	}

}

//based on the current servo position show the current exit
void showExit(Mat &img){

	int bestExitI =0;//y
	int bestExitJ =0;//x
	int bestExit =0;//value
	int sectionX=0;
	int sectionY=0;
	int xCoord=-20; //x value in the image plane
	int yCoord=-20;
	

	vector<Point>results;

	//find the best exit value and position in panorama
	for (int i=0; i<432; i++){
    	for (int j=0; j<576; j++)
    	{
    		//		y   x
      		//if(panorama[i][j] > bestExit){
    		if(panorama[i][j] > 0){
      			bestExit = panorama[i][j];
      			bestExitI = i;
      			bestExitJ = j;
      			Point pointL = Point(j,i);
	  			results.push_back(pointL);
      		}
    	}
    }


    for (int i=0; i<results.size(); i++){ 
    	printf("results.size = %d\n",results.size());
    	bestExitJ = results[i].x;
    	bestExitI = results[i].y;

	    //if there has been an exit recognised
	    if(bestExit!=0){
		    //get x section
		    if(bestExitJ<=192 && bestExitJ>-1)sectionX =1;
		    if(bestExitJ<=384 && bestExitJ>192)sectionX =2;
		    if(bestExitJ<=576 && bestExitJ>384)sectionX =3;
		    //get y section
		    if(bestExitI<=144 && bestExitI>-1)sectionX =1;
		    if(bestExitI<=288 && bestExitI>144)sectionX =2;
		    if(bestExitI<=432 && bestExitI>288)sectionX =3;

		    //calculate if the camera is pointing to the section with the exit in
		    //see what section the camera is by looking at teh left corner of teh image plane
		    int imgLeftX=0;
		    int imgleftY=0;
		    
		    //between sevo position 210 and 150 in X
			if(servo0Pos<=210 && servo0Pos>150){
				int distanceOverlapX = (servo0Pos - 150) * (192.0/60);
				//bestExitX is in current frame
				if(bestExitJ>=(192-distanceOverlapX)&&bestExitJ<=(384-distanceOverlapX)){
					xCoord = bestExitJ - (192-distanceOverlapX);
				}	
			}
			//between sevo position 150 and 90 in X
			if(servo0Pos<=150 && servo0Pos>=90){
				int distanceOverlapX = (servo0Pos - 90) * (192.0/60);
				//bestExitX is in current frame
				if(bestExitJ>=(384-distanceOverlapX)&&bestExitJ<=(576-distanceOverlapX)){
					xCoord = bestExitJ - (384-distanceOverlapX);
				}
			}

			//between sevo position 102 and 150 in Y
			if(servo1Pos<150 && servo1Pos>=102){
				int distanceOverlapY = (servo1Pos - 102) * (144.0/48);
				//bestExitY is in current frame
				if(bestExitI>=distanceOverlapY&&bestExitI<=(288-(144-distanceOverlapY))){
					yCoord = bestExitI - distanceOverlapY;
				}
			}
			//between sevo position 198 and 150 in Y
			if(servo1Pos<=198 && servo1Pos>=150){
				int distanceOverlapY = (servo1Pos - 150) * (144.0/48);
				//bestExitY is in current frame
				if(bestExitI>=(distanceOverlapY+144)&&bestExitI<=(432-(144-distanceOverlapY))){//changthise
					yCoord = bestExitI - 144 - distanceOverlapY;
				}
			}

			//draw the exit
			if(xCoord!=-20&&yCoord!=-20){
				Point pointL = Point(xCoord,yCoord);
				circle(img,pointL,5,Scalar(0,0,255),1);//red
			}
		}
	}
}

void *HOGDetect(void *threadarg)
{
   struct thread_data *my_data;
   my_data = (struct thread_data *) threadarg;
   size_t i,j;
   HOGDescriptor hog;
   hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
   vector<Rect> found, found_filtered;

   hog.detectMultiScale(my_data->img,found, 0, Size(8,8), Size(16,16),1.05, 2); 

   //show possible exit everytime around the loop
   showExit(my_data->img);

    //if servos are already moving send =0
	if(send == 0){
		countMove++;
	}
	if(countMove==3){
		send=1;
		countMove=0;		
	}
	//printf("send = %d\n", send);
	//printf("countMove = %d\n", countMove);

    //(whole for loop, doesnt push rectangle if it is the same as any other)
	for (i=0; i<found.size(); i++) 
	{
	    Rect r = found[i];
	    for (j=0; j<found.size(); j++)
	        if (j!=i && (r & found[j]) == r)
	            break;
	    if (j== found.size())
	        found_filtered.push_back(r);
	}
	//HOG returns slightly larger rectangles, so this shrinks them
	for (i=0; i<found_filtered.size(); i++) 
	{
	    drawRect(my_data->img,found_filtered[i],255,0,0);
	    printf("people detected!!!!!\n");
	}

	imwrite("peopleDetected.jpg",my_data->img);

	//printf("found_filtered size = %d\n", found_filtered.size());


	if(found_filtered.size()!=0){

		//initialiseCount=0;//CHECK THIS AND THE ONES I PUT IN

		//re initialise Kalman filter
		if(reInitialise == 1){
			//printf("initialise with (%d,%d)\n",found_filtered[0].tl().x,found_filtered[0].tl().y);
			//init kalman with the point converted to the panorama
			int pointX,pointY;
			pointX = ((240- servo0Pos)*(192/60)) + found_filtered[0].tl().x;
			pointY = ((servo1Pos - 60)*(144/48)) + found_filtered[0].tl().y;
			printf("initialise with (%d,%d)\n",pointX,pointY);

   			//initKalman(found_filtered[0].tl().x,found_filtered[0].tl().y, my_data->KF);

   			initKalman(pointX,pointY,my_data->KF);

   			//calc hist of person i want to track and save to gloabl variable
   			calcHist(trackHist,my_data->img, found_filtered[0]);
   			reInitialise =0;
  		}

  		//for calculating the rect of the person we want to follow
  		vector<double>results;
  		double d;
  		//calculate histograms for all the rects found and compare
  		for(i=0; i<found_filtered.size(); i++){
  			Mat rectHist;
  			calcHist(rectHist,my_data->img,found_filtered[i]);
  			d = compareHist(trackHist,rectHist,CV_COMP_CORREL);
  			
  			printf("COMPARISON VALUE = %f\n",d);
  			//results of comparison with all rects,
  			results.push_back(d);
  		}
  		//indexes of found_filtered
  		int closestMatch=0;
  		int secondClosest=0;
  		double resultsValue =0;
  		double secondResultsValue =0;

  		//get the best comparisons index for histogram value of all the rects
  		for (int i=0; i<results.size(); i++){
  			//printf("results[%d] = %f\n",i,results[i]);
  			if(results[i]>resultsValue){
  				resultsValue = results[i];
  				closestMatch = i;
  			}else{
  				if(results[i]>secondResultsValue){
  					secondResultsValue = results[i];
  					secondClosest = i;
  				}
  			}
   		}


		//printf("closestMatch index = %d, secondClosest = %d\n",closestMatch,secondClosest);
		//printf("resultsValue = %f, secondResultsValue = %f\n",resultsValue,secondResultsValue);
   		//check to see that the closest match rect to track rect is near the predicted rect
   		printf("predicting because checking the closest match rect\n");
   		Point kPredict = kalmanPredict(my_data->KF);

   		printf("saving old prediction! as (%d,%d)\n",kPredict.x,kPredict.y);
   		//save the prediction as after correction i will compare the new prediction with it.
   		oldPredict = kPredict;

   		//thnk this is fine rather than calling predict becasue we have moved the camera based on this 
   		//prediction and so need to see where the person is now in relation, cos why would prediction
   		//change just because camera has moved(maybe becasue change in time)
   		//Point kPredict = nextPredict;
		Point bottomR = Point(kPredict.x+lastRect.width,kPredict.y+lastRect.height);

        //rectangle(my_data->img, kPredict, bottomR, Scalar(0,0,255), 3);

        //point converted to the panorama
		int pointX,pointY,pointX1,pointY1;
		//first hist
		pointX = ((240- servo0Pos)*(192/60)) + found_filtered[closestMatch].tl().x;
		pointY = ((servo1Pos - 60)*(144/48)) + found_filtered[closestMatch].tl().y;
		//second hist
		pointX1 = ((240- servo0Pos)*(192/60)) + found_filtered[secondClosest].tl().x;
		pointY1 = ((servo1Pos - 60)*(144/48)) + found_filtered[secondClosest].tl().y;
        
   		double histDist,histDist2;
   		//find the dist between the best hist and the prediction
   		histDist = sqrt(pow(pointX-kPredict.x,2) + pow(pointY-kPredict.y,2));
   		histDist2 = sqrt(pow(pointX1-kPredict.x,2) + pow(pointY1-kPredict.y,2));
   		printf("histDist from prediction = %f\n,histDist2 from prediction = %f\n",histDist,histDist2);

   		//check the best match distance, if too far away then check the seond best match, if too far then dont correct
   		// if(results[closestMatch]>0.8||(histDist<60&&results[closestMatch]>0.65)){
   		if(results[closestMatch]>0.9||(histDist<60&&results[closestMatch]>0.75)){
   		 	//printf("drawing the closestMatch\n");
			//correct kalman filter
			printf("correcting in hog\n");
			Point kCorrect = kalmanCorrect(my_data->KF, found_filtered[closestMatch]);
			//rectangle(my_data->img, found_filtered[closestMatch].tl(), found_filtered[closestMatch].br(), Scalar(0,255,0), 3);
			drawRect(my_data->img,found_filtered[closestMatch],0,255,0);
			imwrite("trackedPerson.jpg",my_data->img);

			//imwrite("newImage.jpg",my_data->img);
			//set the values to the last rect found
			lastRect = found_filtered[closestMatch];
			initialiseCount=0;//CHECK THIS
			printf("rect NOT in panorama representation = (%d,%d) \n",lastRect.tl().x,lastRect.tl().y);

			if(send == 1){
				//moving the camera
				moveCamera(found_filtered[closestMatch], my_data->img,my_data->KF);
				send=0;
				cameraMoved = 1;
			}else{
				//because if we detect a person and move and then detect and dont move it will still look
				//at the previous movement if we dont set this to 0
				cameraMoved = 0;
			}

		}else{
			//printf("drawing the secondClosest\n");
			//if the second best hist is closer
			if(results[secondClosest]>0.9||(histDist2<60&&results[secondClosest]>0.75)){

	   			//correct kalman filter
				printf("correcting in HOG seond closest\n");
				Point kCorrect = kalmanCorrect(my_data->KF, found_filtered[secondClosest]);
				drawRect(my_data->img,found_filtered[secondClosest],0,255,0);
				imwrite("trackedPerson.jpg",my_data->img);
		   		//circle(my_data->img,kCorrect,5,Scalar(255,0,0),1);//blue
				//set the values to the last rect found
				lastRect = found_filtered[secondClosest];

				initialiseCount=0;//CHECK THIS

				if(send == 1){
					//moving the camera
					moveCamera(found_filtered[secondClosest], my_data->img,my_data->KF);
					send=0;
					cameraMoved = 1;
				}else{
					cameraMoved = 0;
				}
   			}else{
   				//both are too far away so count as person not found like below
   				initialiseCount++;
   			}
		}
			
	}else{
		//if it needs to be reinitialised then do nothing otherwise predict
		//because it only gets initialised when it has detected people
		if(reInitialise == 0){
			//predict using the kalman filter
			//NEED IT HERE
			printf("predicting because nobody found\n");

			//Point kPredict = kalmanPredict(my_data->KF);
			//Point bottomR = Point(kPredict.x+lastRect.width,kPredict.y+lastRect.height);
			//circle(my_data->img,kPredict,5,Scalar(0,0,255),1);//red
			//drawrect
			//rectangle(my_data->img, kPredict, bottomR, Scalar(0,0,255), 3);
			initialiseCount++;

			//for adding to the panorama
			//if person hasnt been found in 15 tries
			if(initialiseCount == 15){
				//calc panorama because person has been lost
				calcPanorama();
				showExit(my_data->img);

				showPanoramaExits+=1;	
				
				//move camera to exit as nobody has been found
				//getExit();
			}
			//imwrite("img2.jpg",my_data->img);
		}		
	}

	imshow("img2", my_data->img);
	printf("********thread done***********\n");
	threadDone = 1;
	pthread_exit(NULL);
}

int main (int argc, const char * argv[]){

    RaspiCamCvCapture * cap = raspiCamCvCreateCameraCapture(0); 
    namedWindow("opencv");
    namedWindow("img2");
   // namedWindow("histogram");

    if(argc <2){
       printf("Error: too few/many arguments\n");
        return 0;
    }

    whatMove = atoi(argv[1]);
    if(whatMove == 1)printf("Edge\n");
    if(whatMove == 0)printf("Center\n");
  
    IplImage* image;
    Mat img,img2;
    
    //set the servo position to the default start
    servo0Pos = 150;
    //servo1Pos = 160;
    servo1Pos = 160;
    moveServo(0);
    moveServo(1);
    preServo0pos = servo0Pos;
    preServo1pos = servo1Pos;
   
    //initialise the kalman filter
    KalmanFilter KF(4, 2, 0);
    //have to do this(dont know why but it makes it work!)
    initKalman(0,0, KF);
   
	//for the prediction so can put rectangle on screen
	lastRect.height = 1;
	lastRect.width = 1;

	//for the sending to servos
	send = 1;
	countMove=0;

	//threading
	pthread_t threads;//1 thread
	struct thread_data td;

	//set the panorama to 0
	makeZero();
	//panorama[216][158] = 10;
	//panorama[116][158] = 10;

	// Setup output VideoWriter 											//was 120
	//cv::VideoWriter output_cap("outputVideo.avi",CV_FOURCC('D','I','V','X'), 1, cv::Size ( 192,144), true );
	//cv::VideoWriter output_cap2("outputVideo2.avi",CV_FOURCC('D','I','V','X'), 1, cv::Size ( 192,144), true );


    while (true){

    	image = raspiCamCvQueryFrame(cap);
    	//img1 original, img shruk original, img2 hog detected
		//for demo dont resize!!!! this is just for over the network
		Mat img1(image);
		resize(img1,img,Size(),0.3,0.3);

		//Mat img1 = imread("smallImage.jpg", CV_LOAD_IMAGE_COLOR);	
		//resize(img1,img,Size(),1,1);

		//reinitialise if no person found in 15 goes
		if (initialiseCount == 15){
			newPerson = 1;
			reInitialise = 1;
			initialiseCount =0;
		} 

		//HOG detect people
		if(threadDone == 1){
			//could copy this instead to speed up? why arent i? because- for a demo,
			//when im not resizing img1 so ill have to rezize here anyway so slower for testing but does matter
			resize(img1,img2,Size(),0.3,0.3);
			//resize(img1,img2,Size(),1,1);
			threadDone = 0;
			int rc;
			td.img = img2;
			td.KF = KF;

			rc = pthread_create(&threads, NULL,HOGDetect, (void *)&td);
	      	if (rc){
	         	cout << "Error:unable to create thread," << rc << endl;
	         	exit(-1);
	  		}
		}
		
		// output_cap.write(img2);
		// output_cap2.write(img);
		
		imshow("opencv", img);
		//imshow("opencv",img1);
	    if (waitKey(1)>=0 &&threadDone ==1)
	    	break;

	}

	// output_cap2.release();	
	// output_cap.release();	
    pthread_exit(NULL);
    return 0;
}

