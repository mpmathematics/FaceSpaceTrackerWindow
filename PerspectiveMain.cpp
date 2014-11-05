/*
* FaceSpaceTracker.cpp
*
*  Created on: Jul 31, 2014
*      Author: Marcel Padilla
*/

#include <iostream>
#include <stdio.h>
#include <fstream>
#include  <string>

#include <GL/gl.h>		   // Open Graphics Library
#include <GL/glut.h>	   // The GL Utility Toolkit (GLUT)

#include <vector>
#include <math.h> // use M_PI from here
#include <Eigen/Dense> // Eigen header. it is a header only library

// include my own Functions
#include "FaceSpaceTracker.h"

using namespace Eigen;
using namespace std;
using namespace cv; // opencv

#define KEY_ESCAPE 27 // just for the escape key press

// for the purpose of headtracking, we will scale everything to the units of real life mm.

// METHODS that need to be decleared early
Vector3d playerRightVector();
double getPlayerViewAngleVerThisVector(Vector3d);
double getPlayerViewAngleHorThisVector(Vector3d);

// window variables
int windowHeight;
int windowWidth;
char* windowTitle = "FaceSpaceTracker Window Demo. See more on http://mpmathematics.wordpress.com/facespacetracker/";
double windowNearDistance=10;
double windowFarDistance=10000;
double verticalFOV=40; // this is the visualisation FOV, not the camera FOV
double realLifeScreenHeight;
double realLifeScreenWidth;



// view variables
Vector3d playerPosition(0,0,1); // I sit about 830mm away form the screen center
Vector3d playerUp(0,1,0); // Y is up, just like in the facespacetracker coordsystem
Vector3d playerViewDirection = -playerPosition;
double playerViewAngleHor=getPlayerViewAngleHorThisVector(playerPosition); //initial view, radians
double playerViewAngleVer=getPlayerViewAngleVerThisVector(playerPosition);


// opencv headtracking Object
//FaceSpaceTracker Tracker("FaceSpaceTrackerDefault.cfg");
FaceSpaceTracker Tracker("FaceSpaceTracker medion27inch.cfg");

// vectors to manage realative positioning in space
Vector3d humanPosition(1,0,0);
Vector3d humanStartPosition(1,0,0);
Vector3d cameraOffset(0,0,0); // this Vector will describe the change to the calibrated position.
double worldScaleFactor=1;


// toggle variables
bool fullscreen=false;
bool headtracking=true;
bool drawboxes=true;
bool drawmidbox=true;
bool showborder = false;
int translateamount = 0;
bool translatefront = false;
bool drawfaceposition = false;
bool propsorbit = false;


// mouse interaction
int mouseX=0;
int mouseY=0;
int mouseState=0;
int mouseButton=0;
double mousespeed=1;
double mousesensitivity=0.5;//0.001;
double mouseRotationSpeed=1;
bool togglemousespeed=true;

// key press holdings
bool holdingw=false;
bool holdings=false;
bool holdinga=false;
bool holdingd=false;
bool holdingspacebar=false;
bool holdingc=false;


// global variables
double globalRotation = 0;
double globaRotationSpeed = 0.1f;
double movementSpeed=150; // mm per second
double localMovementSpeed=0; // this gets updated by deltatime
double localMouseRotationSpeed=0;
double lastTime=0; //
double deltaTime=0;
GLfloat lightpos[] = {400, 300, 500, 0};
int framecount=0;



//=======================================================================
//////////////////////////////////////////////////////////////////////////
///////////****** INITIALIZATION METHODS *****************//////////////////
//////////////////////////////////////////////////////////////////////////
//=======================================================================


void initializeOpenGl ()
{
	// here we set up a bunch of opengl stuff

	glClearDepth( 1.0f );
	glEnable( GL_DEPTH_TEST );
	glDepthFunc( GL_LEQUAL );
	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
	float amblight=0.3;
	GLfloat amb_light[] = { amblight,amblight,amblight, 1.0 };
	float diffuselight=0.7;
	GLfloat diffuse[] = {diffuselight,diffuselight,diffuselight, 1 };
	float specularlight=0.5;
	GLfloat specular[] = { specularlight,specularlight,specularlight, 1 };
	glLightModelfv( GL_LIGHT_MODEL_AMBIENT, amb_light );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, specular );
	glEnable( GL_LIGHT0 );
	glEnable( GL_COLOR_MATERIAL );
	glShadeModel( GL_SMOOTH );
	glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
	glDepthFunc( GL_LEQUAL );
	glEnable( GL_DEPTH_TEST );
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE); // to avoid changing brigthness with scale
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glLineWidth(1.3);
}

void standardtDisplaySetUp(){
	//this function happens when we turn off the tracking

	// Clear Screen and Depth Buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// set projection matrix
	glMatrixMode(GL_PROJECTION);
	// reset projection matrix
	glLoadIdentity();
	GLfloat aspect = (GLfloat) windowWidth / windowHeight;
	// set up a perspective projection matrix
	gluPerspective(verticalFOV, aspect, windowNearDistance, windowFarDistance);


	// Define a viewing transformation
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(
		playerPosition(0),
		playerPosition(1),
		playerPosition(2),
		playerPosition(0)+playerViewDirection(0),
		playerPosition(1)+playerViewDirection(1),
		playerPosition(2)+playerViewDirection(2),
		playerUp(0),playerUp(1),playerUp(2)
		);

}

void virtualDisplaySetUp(){
	// this is the projection & view matrix set up respecting the tracker

	// Clear Screen and Depth Buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// set matrix mode
	glMatrixMode(GL_PROJECTION);
	// reset projection matrix
	glLoadIdentity();


	// set up a perspective projection matrix
	double tanfov = Tracker.getScreenHeight()/2/fabs(Tracker.getAbsoluteZposition());//tan(verticalFOV);// sync with human
	double ratio = (double)glutGet(GLUT_WINDOW_WIDTH)/(double)glutGet(GLUT_WINDOW_HEIGHT);
	double headProjectionX = Tracker.getXposition()/fabs(Tracker.getAbsoluteZposition());
	double headProjectionY = Tracker.getYposition()/fabs(Tracker.getAbsoluteZposition());

	glFrustum( // left,right,bottom,top,nearVal, farVal
		windowNearDistance*(-tanfov * ratio - headProjectionX),
		windowNearDistance*(tanfov * ratio - headProjectionX),
		windowNearDistance*(-tanfov - headProjectionY),
		windowNearDistance*(tanfov - headProjectionY),
		windowNearDistance, windowFarDistance);


	// Define a viewing transformation
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(
		playerPosition(0) + cameraOffset(0),
		playerPosition(1) + cameraOffset(1),
		playerPosition(2) + cameraOffset(2),
		playerPosition(0) + cameraOffset(0) + playerViewDirection(0),
		playerPosition(1) + cameraOffset(1) + playerViewDirection(1),
		playerPosition(2) + cameraOffset(2) + playerViewDirection(2),
		playerUp(0),playerUp(1),playerUp(2)
		);

}




void variableSetUp(){

	// read the details of the screen from the tracker
	realLifeScreenHeight=Tracker.getScreenHeight();
	realLifeScreenWidth=Tracker.getScreenWidth();

	// specify the human start position
	//for(int i=0 ; i < Tracker.getSkipAmount()*Tracker.getPreviousPixelDataCols()*Tracker.getPreviousVectorDataCols(); i++ )Tracker.updateTrackData();
	humanPosition(0)=Tracker.getAbsoluteXposition();
	humanPosition(1)=Tracker.getAbsoluteYposition();
	humanPosition(2)=Tracker.getAbsoluteZposition();
	//playerPosition = humanStartPosition;
	//playerPosition = humanPosition;
	//humanStartPosition = playerPosition;
}


//=======================================================================
//////////////////////////////////////////////////////////////////////////
///////////****** HELPING METHODS *****************//////////////////
//////////////////////////////////////////////////////////////////////////
//=======================================================================


void upDateViewDirection() {
	if(playerViewAngleVer>=M_PI/2)  playerViewAngleVer=M_PI/2-.000001;
	if(playerViewAngleVer<=-M_PI/2)  playerViewAngleVer=-M_PI/2+.000001;
	playerViewDirection(0)= cos(playerViewAngleVer)*sin(playerViewAngleHor);
	playerViewDirection(1)= sin(playerViewAngleVer);
	playerViewDirection(2)= cos(playerViewAngleVer)*cos(playerViewAngleHor);
}

Vector3d playerRightVector(){
	Vector3d a(
		sin(playerViewAngleHor-M_PI/2),
		0,
		cos(playerViewAngleHor-M_PI/2)
		);
	return a.normalized();
}

double getPlayerViewAngleVerThisVector(Vector3d vec){
	return asin(-vec(1)/vec.norm());
}

double getPlayerViewAngleHorThisVector(Vector3d vec){
	double element = atan(vec(0)/vec(2));
	double correction=M_PI;
	if( vec(2)<0 ) correction = 0;
	return element+correction;
}


//=======================================================================
//////////////////////////////////////////////////////////////////////////
///////////****** DRAW METHODS *****************//////////////////
//////////////////////////////////////////////////////////////////////////
//=======================================================================
// there is nothing specia about these function. The only remarkable thing is that the size of the Box was made to fit the initial field of view perfectly


void drawPropsOnSticks(double x, double y , double z, double scale, int prop){

	GLdouble oldlength[] = {0.5f};
	glGetDoublev(GL_LINE_WIDTH, oldlength);
	glPushMatrix();
	glLineWidth(7);
	glTranslatef(x,y,z);
	glScalef(scale, scale, scale);

	// real time rotation
	glRotatef(globalRotation, 0, 1, 0);

	switch(prop) {
	case 1:
		glutSolidTeapot(1);
		break;
	case 2:
		glutSolidCube(1);
		break;
	case 3:
		glutSolidDodecahedron();
		break;
	case 4:
		glutSolidTorus(.2,.2,10,10);
		break;
	case 5:
		glutSolidIcosahedron();
		break;
	case 6:
		glutSolidSphere(1,10,10);
	}

	// draw the stick
	/*
	glBegin(GL_LINES);
	glVertex3d(0,0,0);
	glColor3f(0,0,0);
	glVertex3d(0,0,-100);
	glEnd(); */
	glPopMatrix();
	glLineWidth(oldlength[0]);


}


// a method to visualise space better
void drawManyBoxes(int depth, int horstripes, int verstripes){
	GLdouble oldlength[] = {0.5f};
	glGetDoublev(GL_LINE_WIDTH, oldlength);
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);

	// draw the lines going into depth, fading into black
	double x,y;
	for ( int i= 0 ; i < horstripes ; i++) {
		glBegin(GL_LINES);

		x = i*((double)2)/(double)(horstripes-1)-1;
		y=1;
		glColor3d(1,1,1); // white
		glVertex3d(x,y,0);
		glColor3d(0,0,0); // black
		glVertex3d(x,y,-depth);
		y=-1;
		glColor3d(1,1,1); // white
		glVertex3d(x,y,0);
		glColor3d(0,0,0); // black
		glVertex3d(x,y,-depth);

		glEnd();
	}

	for ( int i= 0 ; i < verstripes ; i++) {
		glBegin(GL_LINES);

		y = i*((double)2)/(double)(horstripes-1)-1;
		x=1;
		glColor3d(1,1,1); // white
		glVertex3d(x,y,0);
		glColor3d(0,0,0); // black
		glVertex3d(x,y,-depth);
		x=-1;
		glColor3d(1,1,1); // white
		glVertex3d(x,y,0);
		glColor3d(0,0,0); // black
		glVertex3d(x,y,-depth);

		glEnd();
	}
	
	// draw the boxes moving into depth
	for (int i = 0 ; i < depth ; i++ ){
		float grey=1*(depth-i-1)/(double)(depth-1);
		glColor3f(grey,grey,grey); // fading to black
		glBegin(GL_LINE_STRIP);
		glVertex3d(1,1,-i);
		glVertex3d(-1,1,-i);
		glVertex3d(-1,-1,-i);
		glVertex3d(1,-1,-i);
		glVertex3d(1,1,-i);
		glEnd();
	}
	glPopAttrib();

	glLineWidth(oldlength[0]);

	glPopAttrib();
	glLineWidth(oldlength[0]);
	
}


void drawBorder(){
	// to give a white border around the box that serves like lit on the box
	// the idea is to leave the center open so object can then float over the with border and give a pop-out effect
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);

	glBegin(GL_QUADS);
	glColor3d(1,1,1);
	// top
	glVertex3d(-realLifeScreenWidth/2,realLifeScreenHeight/2,0);
	glVertex3d(-realLifeScreenWidth/2,realLifeScreenHeight/4,0);
	glVertex3d(realLifeScreenWidth/2,realLifeScreenHeight/4,0);
	glVertex3d(realLifeScreenWidth/2,realLifeScreenHeight/2,0);
	// bot
	glVertex3d(-realLifeScreenWidth/2,-realLifeScreenHeight/2,0);
	glVertex3d(-realLifeScreenWidth/2,-realLifeScreenHeight/4,0);
	glVertex3d(realLifeScreenWidth/2,-realLifeScreenHeight/4,0);
	glVertex3d(realLifeScreenWidth/2,-realLifeScreenHeight/2,0);
	// right
	glVertex3d(-realLifeScreenWidth/2,realLifeScreenHeight/2,0);
	glVertex3d(-realLifeScreenWidth/4,realLifeScreenHeight/2,0);
	glVertex3d(-realLifeScreenWidth/4,-realLifeScreenHeight/2,0);
	glVertex3d(-realLifeScreenWidth/2,-realLifeScreenHeight/2,0);
	// left
	glVertex3d(realLifeScreenWidth/2,realLifeScreenHeight/2,0);
	glVertex3d(realLifeScreenWidth/4,realLifeScreenHeight/2,0);
	glVertex3d(realLifeScreenWidth/4,-realLifeScreenHeight/2,0);
	glVertex3d(realLifeScreenWidth/2,-realLifeScreenHeight/2,0);

	glEnd();
	glPopAttrib();
}

void drawAllProps(){
		// draw props
	glPushMatrix();
	//glRotated(globalRotation,1,0,0);
	glTranslated(0,0, translateamount);
	if (showborder) glScaled(.5,0.5,.5);

	// rotate like planetary movement
	if( propsorbit) glRotated(globalRotation/4, 0, 1, 0);

	//scales to move the objects without distorting them. good for all screen ratios
	double xs = realLifeScreenWidth/2;
	double ys = realLifeScreenHeight/2;

	// now we place multiple props. since the sizes are choosen to be in millimeters just like in the real world the third component determines the mm how for in/out the object floats in space.
	glColor3d(0,1,1);
	drawPropsOnSticks(0*xs,0*ys,0,25,5);
	glColor3f(0,1,1);
	drawPropsOnSticks(0.3*xs, 0.7*ys ,0, 10 , 1);
	glColor3f(1,0,1);
	drawPropsOnSticks(-.6*xs,-0.4*ys,40,11,2);
	glColor3f(1,1,0);
	drawPropsOnSticks(-.8*xs,.5*ys,70,7,3);
	glColor3f(0,1,0);
	drawPropsOnSticks(.8*xs,0.8*ys,20,10,4);
	glColor3f(0.5,0.5,0.5);
	drawPropsOnSticks(.7*xs,-.3*ys,-120,10,3);
	glColor3f(1,0,0);
	drawPropsOnSticks(.8*xs,.4*ys,-100,10,5);
	glColor3f(0,0,1);
	drawPropsOnSticks(-.2*xs,.7*ys,-150,10,6);
	glColor3f(1,0,1);
	drawPropsOnSticks(.6*xs,-0.5*ys,100,7,2);
	glColor3f(1,.5,0);
	drawPropsOnSticks(-0.1*xs,-0.8*ys,-50,10,3);
	glColor3f(0,1,0);
	drawPropsOnSticks(-.4*xs,0*ys,-100,20,4);
	glColor3f(1,0,0);
	drawPropsOnSticks(-.2*xs,0.2*ys,200,5,5);
	glColor3f(0,0,1);
	drawPropsOnSticks(-.4*xs,-0.6*ys,-250,5,6);
	glColor3f(1,1,1);
	drawPropsOnSticks(-.4*xs,-.3*ys,150,15,1);
	glColor3f(.5,.5,1);
	drawPropsOnSticks(-.4*xs,-.2*ys,-700,20,2);
	glColor3f(.5,.5,1);
	drawPropsOnSticks(-.8*xs,.9*ys,150,20,5);
	glColor3f(.7,.5,.1);
	drawPropsOnSticks(.8*xs,-.7*ys,150,15,2);

	glPopMatrix();
}

void drawRooms(){
	// Draw Rooms
	glPushMatrix(); // edge box
	glScaled(realLifeScreenWidth/2,realLifeScreenHeight/2,40);
	drawManyBoxes(10 , 6, 6);
	glPopMatrix();
	if(drawmidbox) {
		glPushMatrix(); // in box
		glTranslated(0,0, translateamount );
		glScaled(realLifeScreenWidth/4,realLifeScreenHeight/4,50);
		glColor3d(0.5,0.5,0.5);
		drawManyBoxes(10 , 6, 6);
		glPopMatrix();
	}
}

void drawFacePointer(){
	  // draw an object that flies with your face
		double sensitivity=1; // you can edit the sensitivity to exepriment with 
		glColor3f(1,0,0);
		drawPropsOnSticks(humanPosition(0), humanPosition(1)*sensitivity, sensitivity*400/2, 4, 6);
		glColor3f(1,1,1);
		drawPropsOnSticks(humanPosition(0), humanPosition(1)*sensitivity, sensitivity*400/2+10, 3, 6);
		glColor3f(1,0,0);
		drawPropsOnSticks(humanPosition(0), humanPosition(1)*sensitivity, sensitivity*400/2+20, 2, 6);
		glColor3f(1,1,1);
		drawPropsOnSticks(humanPosition(0), humanPosition(1)*sensitivity, sensitivity*400/2+30, 1, 6);
}

void drawWall(){
	// draw a big wall where the user has to move his head to look beyond it.
	// TODO. preferably textured. I would also like to see a textured object in space
}

//=======================================================================
//////////////////////////////////////////////////////////////////////////
///////////****** INTERACTION METHODS *****************//////////////////
//////////////////////////////////////////////////////////////////////////
//=======================================================================


void checkKeyboardButtons(){
	if(holdingw){
		playerPosition+=localMovementSpeed*playerViewDirection;
	}
	if(holdings){
		playerPosition-=localMovementSpeed*playerViewDirection;
	}
	if(holdinga){
		playerPosition-=localMovementSpeed*playerRightVector();
	}
	if(holdingd){
		playerPosition+=localMovementSpeed*playerRightVector();
	}
	if(holdingspacebar){
		playerPosition+=localMovementSpeed*playerUp;
	}
	if(holdingc){
		playerPosition-=localMovementSpeed*playerUp;
	}

}
// normal ASCII keyboard handle
void keyboard ( unsigned char key, int mousePositionX, int mousePositionY )
{

	switch ( key )
	{
	case KEY_ESCAPE:
		//stop the tracker
		//		linuxtrack_shutdown();
		exit ( 0 );
		break;

		// translate the object
	case 'w':
		holdingw=true;
		break;

	case 's':
		holdings=true;
		break;

	case 'a':
		holdinga=true;
		break;

	case 'd':
		holdingd=true;
		break;
	case ' ':
		holdingspacebar=true;
		break;
	case 'c':
		holdingc=true;
		break;


		// save the storage data in file
	case 'm':
		Tracker.setCamVerticalFOV(42);
		Tracker.saveConfigFile();
		break;

	case 'n':
		Tracker.readConfigFile();
		break;

		// calibrate
	case 'l':
		cout << "Calibrate Center" << endl;
		playerPosition(2) = Tracker.getAbsoluteZposition();
		Tracker.calibrateCenter();
		break;

		// reset calibrate
	case 'o':
		Tracker.resetCalibration();
		break;
		// toggle see faceposition
	case 'k':
		drawfaceposition = !drawfaceposition;
		break;
		//toggle border
	case 'j':
		showborder=!showborder;
		break;
		//toggle boxes
	case 'u':
		drawboxes=!drawboxes;
		break;
		//toggle mid boxes
	case 'z':
		drawmidbox=!drawmidbox;
		break;
		// toggle show webcam
	case 'g':
		Tracker.setShowCamFrame(!Tracker.getShowCamFrame());
		break;
		
		// change depth of props
	case 'b':
		if(translateamount == 0) translateamount = 120;
		else if(translateamount == 120) translateamount = -300;
		else if(translateamount == -300) translateamount = 0;
		break;

		// change depth of mid box
	case 't':
		translatefront=!translatefront;
		break;

	case '1':
		Tracker.reportStatus();
		break;

		//swap the .cfg file for another 
	case '2':
		propsorbit=!propsorbit;
		//Tracker.setCurrentConfigFile("FaceSpaceTracker medion27inch.cfg");
		//Tracker.loadConfigFile("FaceSpaceTracker medion27inch.cfg");
		break;

		// toggle headtracking
	case 'h':
		if(headtracking) {
			headtracking=!headtracking;
			cout << "---- end headtracking ----" << endl;
		}
		else {
			headtracking=!headtracking;
			cout << "---- start headtracking ----" << endl;
		}
		break;

		//toggle fullscreen
	case 'f':
		if(fullscreen){
			fullscreen=!fullscreen;
			glutPositionWindow(0,0);
			glutReshapeWindow(windowWidth/2, windowHeight/2);
		} else {
			fullscreen=!fullscreen;
			glutFullScreen();
		}
		break;

		// reset transformations
	case 'r':

		break;

	default:
		break;
	}
}

void keyboardUp (unsigned char key, int x, int y) {
	switch ( key )
	{
	case 'w':
		holdingw=false;
		break;
	case 's':
		holdings=false;
		break;
	case 'a':
		holdinga=false;
		break;
	case 'd':
		holdingd=false;
		break;
	case ' ':
		holdingspacebar=false;
		break;
	case 'c':
		holdingc=false;
		break;

	default:
		break;
	}
}
// viewing with mouse by pressing it
void mouse(int btn, int state, int x, int y){
	if (state== GLUT_DOWN){
		mouseState=state;
		mouseButton=btn;
		mouseX=x;
		mouseY=y;
	} else {
		mouseState=0;
	}
}

void motion(int x , int y){
	if(mouseState==GLUT_DOWN){
		if(mouseButton== GLUT_LEFT_BUTTON){
			// dont use deltaTime here
			playerViewAngleHor+=localMouseRotationSpeed*mousesensitivity*(mouseX-x);
			playerViewAngleVer+=localMouseRotationSpeed*mousesensitivity*(mouseY-y);
			// reset mouse
			mouseX=x;
			mouseY=y;

		}
		if(mouseButton== GLUT_RIGHT_BUTTON){
			// dont use deltaTime here
			cameraOffset-=localMovementSpeed*(mouseX-x)*playerRightVector();
			cameraOffset+=localMovementSpeed*(mouseY-y)*playerUp;
			// reset mouse
			mouseX=x;
			mouseY=y;
		}

	}
}


void upDateDeltaTime(){ // compute for consistent speed on all machines
	double currentTime=glutGet(GLUT_ELAPSED_TIME);
	float passedTime= currentTime - lastTime;
	lastTime = currentTime; // dont write float lasttime, that deletes the value
	deltaTime = passedTime/1000;
}

void generalUpDates(){

	// update View Stuff
	upDateViewDirection();

	// update Keyboard interaction
	checkKeyboardButtons();

	//update motion parameters
	upDateDeltaTime();
	//	cout << "\r";
	//		cout << deltaTime*1000 << " / " << (double)lastTime/1000 << " sec / " << (int)(1/deltaTime) << " FPS" << endl;;
	globalRotation += globaRotationSpeed;
	localMovementSpeed = movementSpeed*deltaTime;
	localMouseRotationSpeed = 1*0.01;//mouseRotationSpeed*deltaTime;
	framecount++;

	// update Buffer
	glutSwapBuffers();
}



//=======================================================================
//////////////////////////////////////////////////////////////////////////
///////////****** DISPLAY & MAIN *****************//////////////////
//////////////////////////////////////////////////////////////////////////
//=======================================================================


void display()
{
	// reset light pos due to rotations
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos); 

	// set up view and projection matrix
	if(headtracking) {

		// tell the tracker to perform a track
		Tracker.updateTrackData();
		// store the new track data
		humanPosition(0)=Tracker.getXposition();
		humanPosition(1)=Tracker.getYposition();
		humanPosition(2)= Tracker.getZposition();
		
		// update cameraoffset
		cameraOffset = humanPosition;// - playerPosition;
		
		// rotate the offset to be in frame with the view direction
		Vector3d correction;
		correction = cameraOffset(0)*playerRightVector() + cameraOffset(1)*playerUp - cameraOffset(2)*playerViewDirection;
		cameraOffset = correction;
		
		// load the projection matrix
		virtualDisplaySetUp();

	}
	else {
		// load normal projection Matrix
		standardtDisplaySetUp();
	}

	if(showborder){ 
		// to give a white border around the box to jump over
		drawBorder();
	}

	drawAllProps();

	if(drawboxes) drawRooms();
	
	if(drawfaceposition){
		drawFacePointer();
	}

	//updates
	generalUpDates();
}


int main(int argc, char **argv)
{

	// initialize Glut
	cout << " - glut init " << endl;
	glutInit(&argc, argv); // GLUT initialization
	windowHeight=glutGet((int)GLUT_SCREEN_HEIGHT);// System specific sizes
	windowWidth=glutGet((int)GLUT_SCREEN_WIDTH);// DONT USE THIS COMMAND WITH MULTI MONITORS
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	glutInitWindowSize(windowWidth/2,windowHeight/2);	// set first window size, not important in fullscreen. the initial window takes up a quarter of the screen
	glutCreateWindow(windowTitle);	// create Window

	// register functions for main loop
	glutDisplayFunc(display);						
	glutIdleFunc( display );						
	glutKeyboardFunc( keyboard );					
	glutKeyboardUpFunc(keyboardUp);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);

	// OpenGl settings
	cout << " - OpenGl setup " << endl;
	initializeOpenGl();

	// set up some variables
	cout << " - set up variables " << endl;
	variableSetUp();

	//report camera data:
	Tracker.reportStatus();

	cout << " ======= Controls ====== " << endl;
	cout << " press l to calibrate the camera " << endl;
	cout << " press f to toggle fullscreen " << endl;
	cout << " press v to toggle fixed depth " << endl;
	cout << " press j to toggle draw border " << endl;
	cout << " press k to toggle draw face pointer " << endl;

	cout << " ==== Controls end ===== " << endl;

	// run GLUT mainloop so it uses the registered functions
	cout << " - begin mainloop " << endl;
	glutMainLoop();							

	return 0;
}
