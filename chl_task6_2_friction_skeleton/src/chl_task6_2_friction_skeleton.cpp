//===========================================================================
/*
This file is part of the CHAI 3D visualization and haptics libraries.
Copyright (C) 2003-2009 by CHAI 3D. All rights reserved.

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License("GPL") version 2
as published by the Free Software Foundation.

For using the CHAI 3D libraries with software that can not be combined
with the GNU GPL, and for taking advantage of the additional benefits
of our support services, please contact CHAI 3D about acquiring a
Professional Edition License.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   2.0.0 $Rev: 269 $
*/
//===========================================================================

//===========================================================================
/*edited for the Computational Haptics Laboratory SS 2010, 
Lehrstuhl fuer Medientechnik, Technische Universitaet Muenchen, Germany
last revision: 07.06.2010*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <time.h>
#include "bass.h"
#include "chai3d.h" 
#include <ctime>
//#include <CMesh.h>

//---------------------------------------------------------------------------
#include "ch_generic3dofPointer.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//							DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

using namespace std;

//---------------------------------------------------------------------------
//							DECLARED VARIABLES
//---------------------------------------------------------------------------
bool b_timing = false;

bool ch_useForceShading = true;
// a world that contains all objects of the virtual environment
cWorld* world;

// a camera that renders the world in a window display
cCamera* camera;

// a light source to illuminate the objects in the virtual scene
cLight *light;

// a little "chai3d" bitmap logo at the bottom of the screen
cBitmap* logo;

// width and height of the current window display
int displayW  = 0;
int displayH  = 0;

//UI
cLabel CounterI;
cLabel CounterC;
cLabel CounterR;
cLabel Time_d2;
_Longlong inc=0;
_Longlong cor=0;
_Longlong tim_d;
string Wrongs = "  ";
string Rights = "  ";
string Logic1 = " ";
string Logic0 = " ";
string ctr11 = " "; 

void redrawUI();
// a haptic device handler
cHapticDeviceHandler* handler;

// information about the current haptic device
cHapticDeviceInfo info;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a virtual DObject mesh
vector<cMesh*> Objects;

// temp variable to store positions and orientations
// of DObject and drill
cVector3d lastPosObject;
cMatrix3d lastRotObject;
cVector3d lastPosDevice;
cMatrix3d lastRotDevice;
cVector3d lastDeviceObjectPos;
cMatrix3d lastDeviceObjectRot;

// status of the main simulation haptics loop
bool simulationRunning = false;
bool checkBoxColl=false;

// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool simulationFinished = false;

//////////////////////////////////////////////////////////////////////////////////////
//									our variables									//
//////////////////////////////////////////////////////////////////////////////////////
// bool
bool fileload;
bool newButPush = false;
bool highlight_box = false;
bool nr_input_el = true;
bool use_max_force = false;
bool handle_visible = true;
bool restart_counter = false;
bool act_t = true;
bool f_first = false;
bool b_vc = true;

// int
int sound_ctr = 0;
int timing_ctr = 0;
int audio_f = 0;
int act_obj = 0;
int multChoice[3];
int realModel;
int selectedModel;
int LastID;
int inp_nr = 5;
int nr_of_games = 5;
int current_state;
int freq = 0;
int max_force = 5;
int time_kk = 0;

static const int NumObj = 12; //Set number of Objects
static const int numStreams = 5;

// double
double distToObj[3];
double transparencyLevel = 0.3; // transparency level
double abs_force = 0;
double tan_veloc = 0;
double cos_veloc_angle = 0;
double veloc_mul[NumObj] = {1,1,1,1,1,1,1,1,1,1,1,1};

long double t_val;

// virtual drill mesh
cMesh* drill;
string texture, t_ctr, t_path, f_obj;
clock_t t_begin, t_end, tim_b, tim_e;

// haptics loop
cMesh* ContObject;
cGenericObject* GObject; 

// cVector3d
cVector3d device_veloc;
cVector3d force;
cVector3d norm_force;
float c_time = 20;

// ch lab
double static_coeff;
double dynamic_coeff;
bool useFriction = true;
double stiffnessMax;

//---------------------------------------------------------------------------
//							DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//---------------------------------------------------------------------------
//							DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

// function called before exiting the application
void close(void);

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// set static and dynamic friction coefficients for the object, propagating 
// them to the children
void load_object(cMesh *DObject, string f_obj, string texture, double stat_fric, double dyn_fric, double stiffness,cVector3d cvd = cVector3d(1.0, 0.0, 0.0));

// modify sound according to user action
void ChangeSound(int ID);

// game logic
void gameLogic(void);

// highlight colission box
void highlight_col_box(void);

// handles space bar
void space_bar_func(void);



//===========================================================================
/*
DEMO:    DObject.cpp

This demonstration shows how to attach a 3D mesh file to a virtual
tool. A second mesh (DObject) is loaded in the scene. By pressing the user
switch of the haptic device it is possible to translate or oriente
the DObject object accordingly.

In the main haptics loop function  "updateHaptics()" , the position
of the haptic device is retrieved at each simulation iteration.
The interaction forces are then computed and sent to the device.
*/
//===========================================================================

int main(int argc, char* argv[]){

	//-----------------------------------------------------------------------
	//								INITIALIZATION
	//-----------------------------------------------------------------------

	printf ("\n");
	printf ("Computational Haptics Lab SOSE 2015 - Final Project \n");
	printf ("###################################################################### \n");
	printf ("###################################################################### \n");
	printf ("########################### Haptic Memory ############################ \n");
	printf ("###################################################################### \n");
	printf ("###################################################################### \n");
	printf ("by Marc Dreiser and Julian Eiler\n\n");
	printf ("\n\n");
	printf ("Gameplay:\n\n");
	printf ("[I] Init Phase --> get familiar the objects \n");
	printf ("\t [Q] Show Last Model \n");
	printf ("\t [W] Show Next Model \n\n");
	printf ("[R] Load New Object --> explore the object \n\n");
	printf ("[SPACE] Display possible solutions \n");
	printf ("\t Touch one object  \n");
	printf ("\t --> the appropriate colission box will turn blue \n\n");
	printf ("[SPACE] Check if your choice is correct \n");
	printf ("\t the colission box of the correct object will turn green \n\n");
	printf ("Go to next object with [SPACE] \n");
	printf ("\n\n");
	printf ("[F] Switches force shading on/off (default = on) \n");
	printf ("[S] Switches full volume on/off (default = off) \n");
	printf ("[V] Switches handle visibility on/off (default = on) \n");
	printf ("[T] Switches timing on/off (default = on) \n");
	printf ("[Z] Switches voice coil on/off (default = on) \n");
	printf ("\n\n");
	printf ("Choose the number of rounds you want to guess and confirm with [ENTER] \n");
	printf ("[1] play 3 rounds \n");
	printf ("[2] play 5 rounds (default) \n");
	printf ("[3] play 10 rounds \n\n\n");

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

	srand (time(NULL));

	//-----------------------------------------------------------------------
	//							3D - SCENEGRAPH
	//-----------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	// the color is defined by its (R,G,B) components.
	world->setBackgroundColor(0.0, 0.0, 0.0);

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and oriente the camera
	camera->set( cVector3d (3.0, 0.0, 0.0),    // camera position (eye)
		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

	// set the near and far clipping planes of the camera
	// anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// enable high quality rendering when DObject becomes transparent
	camera->enableMultipassTransparency(true);

	 // create a font
    cFont *font;   
	// a label to display the rate [Hz] at which the simulation is running
	cLabel* labelHapticRate;

	// create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel();
	camera->m_front_2Dscene.addChild(labelHapticRate);
    //camera->m_frontLayer->addChild(labelHapticRate);
    //labelHapticRate->m_fontColor.setBlack();
	labelHapticRate->m_fontColor=cColorf(0,0,0,1);

	

	// create a light source and attach it to the camera
	
	light = new cLight(world);
	camera->addChild(light);                   // attach light to camera
	light->setEnabled(true);                   // enable light source
	light->setPos(cVector3d( 0.0, 0.0, 20.0));  // position the light source
	light->setDir(cVector3d(0.0, 0.0, 0.0));  // define the direction of the light beam
	//light->set
	light->setShowBox(true);

	//-----------------------------------------------------------------------
	//								2D - WIDGETS
	//-----------------------------------------------------------------------

	// create a 2D bitmap logo
	logo = new cBitmap();

	// add logo to the front plane
	camera->m_front_2Dscene.addChild(logo);

	// load a "chai3d" bitmap image file
	fileload = logo->m_image.loadFromFile(RESOURCE_PATH("resources/images/chai3d.bmp"));

	// position the logo at the bottom left of the screen (pixel coordinates)
	logo->setPos(10, 10, 0);

	// scale the logo along its horizontal and vertical axis
	logo->setZoomHV(0.4, 0.4);

	// here we replace all black pixels (0,0,0) of the logo bitmap
	// with transparent black pixels (0, 0, 0, 0). This allows us to make
	// the background of the logo look transparent.
	logo->m_image.replace(
		cColorb(0, 0, 0),      // original RGB color
		cColorb(0, 0, 0, 0)    // new RGBA color
		);

	// enable transparency
	logo->enableTransparency(true);

	camera->m_front_2Dscene.addChild(&CounterI);
	camera->m_front_2Dscene.addChild(&CounterC);
	camera->m_front_2Dscene.addChild(&CounterR);
	camera->m_front_2Dscene.addChild(&Time_d2);

	CounterI.m_fontColor = cColorf(1,0,0,1);
	CounterC.m_fontColor = cColorf(0,1,0,1);
	CounterR.m_fontColor = cColorf(1,1,1,1);
	Time_d2.m_fontColor = cColorf(1,1,1,1);

	CounterI.setPos(500,590,0);	
	CounterC.setPos(400,590,0);	
	CounterR.setPos(200,200,0);	
	Time_d2.setPos(400,570,0);
	
	redrawUI();
	
	//-----------------------------------------------------------------------
	//						HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	cGenericHapticDevice* hapticDevice;
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	if (hapticDevice){info = hapticDevice->getSpecifications();}

	// create a 3D tool and add it to the world
	tool = new cGeneric3dofPointer(world);
	//tool = new ch_generic3dofPointer(world);
	world->addChild(tool);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// initialize tool by connecting to haptic device
	tool->start();

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// define a radius for the tool (graphical display)
	tool->setRadius(0.01);

	// hide the device sphere. only show proxy.
	tool->m_deviceSphere->setShowEnabled(false);

	// set the physical radius of the proxy to be equal to the radius
	// of the tip of the mesh drill (see drill in the virtual scene section)
	proxyRadius = 0.03;
	tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);

	// informe the finger-proxy force renderer to only check one side of triangles
	tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

	// the environmeny is static, you can set this parameter to "false"
	tool->m_proxyPointForceModel->m_useDynamicProxy = false;

	tool->m_proxyPointForceModel->m_useForceShading = false;

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// define a maximum stiffness that can be handled by the current
	// haptic device. The value is scaled to take into account the
	// workspace scale factor
	stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;

	tool->m_proxyPointForceModel->setEpsilonBaseValue(0.01);


	// Initialize sound device and create audio stream
	BASS_Init(1,44100,0,0,NULL);

	//-----------------------------------------------------------------------
	//					 COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	//Materials
	cMaterial mat, mat_tooth;
	mat.m_ambient.set(1, 1, 1);
	mat.m_diffuse.set(1, 1, 1);
	mat.m_specular.set(1.0, 1.0, 1.0);

	cout << "Loading objects ";
	// switch chooses the correct object and sets the attributes of the object
	for(int it=0;it<NumObj;it++){
		cMesh *DObject = new cMesh(world);

		// add object to world
		world->addChild(DObject);

		// set the position and orientation of the object 
		DObject->setPos(0.0, 0.0, 0.0);
		DObject->setTag(it);

		cout << ".";

		switch(it){ // set object specific properties

		case 0: // tooth
			// load soundfiles, dyn_fric = 0.2 stat_fric = 0.15, stiff = 0.8s
			load_object(DObject, "tooth/tooth.3ds", "amp_files/ceramik_amp", 0.15, 0.2, 0.8, cVector3d(0.0, 0.0, 0.0)); //  
			
			mat_tooth.m_ambient.set(1.0, 1.0, 1.0);
			mat_tooth.m_diffuse.set(1.0, 1.0, 1.0);
			mat_tooth.m_specular.set(1.0, 1.0, 1.0);
			 
			mat_tooth.setDynamicFriction(0.08);
			mat_tooth.setStaticFriction(0.1);
			mat_tooth.setStiffness(0.8 * stiffnessMax);

			DObject->setMaterial(mat_tooth, true);
			veloc_mul[it] = 1.0;
			break;

		case 1: // bunny
			// load soundfiles, dyn_fric = 0.3, stat_fric = 0.4, stiff = 0.2
			load_object(DObject, "bunny/bunny.obj", "amp_files/carpet_amp", 0.3, 0.4, 0.2,  cVector3d(1.0, 0.0, 0.0));
			veloc_mul[it] = 0.2;
			break;

		case 2:	// rock, granite
			// dyn_fric = 0.3, stat_fric = 0.43, stiff = 0.8
			load_object(DObject, "Stone/Rock1.obj", "stone_tile", 0.3, 0.43, 0.95, cVector3d(0.0, 0.0, 0.8));	
			DObject->getChild(0)->m_material.m_diffuse=cColorf(1,0.5,0.3,1);
			veloc_mul[it] = 1.0;
			break;

		case 3:	// sponge
			// upper part: dyn_fric = 0.2, stat_fric = 0.25, stiff = 0.1
			// power part: dyn_fric = 0.3, stat_fric = 0.4, stiff = 0.2
			load_object(DObject, "Schwamm/sponge.obj", "amp_files/foam_medium_amp", 0.2, 0.25, 0.1,  cVector3d(0.2, 0.2, 0.8));
			veloc_mul[it] = 1.0;
			break;

		case 4:	// rock, sandstone	
			// dyn_fric = 0.4, stat_fric = 0.51, stiff = 0.6
			load_object(DObject, "Stone/sand_stone.obj", "stone_tile", 0.4, 0.51, 0.7,  cVector3d(0.0, 0.0, 0.8));
			veloc_mul[it] = 1.0;
			break;

		 case 5: // Cork
			 // dyn_fric = 0.5, stat_fric = 0.5, stiff = 0.5
			load_object(DObject, "Cork/cork.obj", "amp_files/Cork_amp", 0.5, 0.5, 0.4,  cVector3d(0.0, 0.0, 1.0));
			veloc_mul[it] = 0.8;
			break;	 	
			
		case 6:	// paper Cup
			// dyn_fric = 0.2, stat_fric = 0.25, stiff = 0.4
			load_object(DObject, "paperCup/paper_cup_final.obj", "amp_files/paper_amp", 0.5, 0.5, 0.4,  cVector3d(0.0, 0.0, 0.0));
			veloc_mul[it] = 0.8;
			break;

		case 7:	// ice
			// dyn_fric = 0.02, stat_fric = 0.03, stiff = 0.8
			load_object(DObject, "iceCube2/ice.obj", "amp_files/foil_amp", 0.01, 0.01, 0.95,  cVector3d(1.0, 0.2, 0.0));
			DObject->setTransparencyLevel(0.2);
			DObject->setUseTransparency(true, true);
			veloc_mul[it] = 0.2;
			break;

		//case 8:	// Teddy
		//	// dyn_fric = 0.2, stat_fric = 0.25, stiff = 0.2
		//	load_object(DObject, "teddy_bear/teddy.obj", "amp_files/Cashmere_amp", 0.5, 0.5, 0.2,  cVector3d(0.0, 0.0, 0.0));
		//	veloc_mul[it] = 0.6;
		//	break;

		case 8:	// eraser
			// dyn_fric = 0.15, stat_fric = 0.19, stiff = 0.8
			load_object(DObject, "PinkandInk/eraser.obj", "amp_files/rubber_amp", 0.7, 0.7, 0.5,  cVector3d(0.2, 1.0, 0));
			veloc_mul[it] = 0.6;
			break;	


		case 9:	// (shot) glass
			// dyn_fric = 0.15, stat_fric = 0.19, stiff = 0.8
			load_object(DObject, "Shot_glass/shot_glass.obj", "amp_files/Glass_amp", 0.01, 0.015, 0.9, cVector3d(0.0, 0.0, 0.0));
			DObject->setTransparencyLevel(0.2);
			DObject->setUseTransparency(true, true);
			veloc_mul[it] = 0.8;
			break;	 

		case 10: // plastic bottle
			// dyn_fric = 0.15, stat_fric = 0.19, stiff = 0.5
			load_object(DObject, "plastic_bottle/plastic_bottle.obj", "amp_files/Glass_amp", 0.5, 0.5, 0.4,  cVector3d(0.0, 0.0, 0.0));
			veloc_mul[it] = 0.8;
			break;	 	

		case 11: // box
			// load soundfiles, dyn_fric = 0.2 stat_fric = 0.15, stiff = 0.8s
			load_object(DObject, "Wooden Box/box_or.obj", "amp_files/beech_amp", 0.5, 0.6, 0.9, cVector3d(1.0, 0.0, 0.0)); // 
			veloc_mul[it] = 0.8;
			break;

		} // end of case statement
		
		// 5) compute boundary box
		// 6) create collision detector
		// 8) intialize stream

		// get dimensions of object
		double size = cSub(DObject->getBoundaryMax(), DObject->getBoundaryMin()).length();

		// resize object to screen
		if (size > 0) DObject->scale( 2.0 * tool->getWorkspaceRadius() / size);

		DObject->setUseTexture(true, true);
		
		// compute a boundary box
		DObject->computeBoundaryBox(true);
		// compute collision detection algorithm		
		DObject->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
		DObject->setShowEnabled(false, true);

		Objects.push_back(DObject);	

	}
	cout << endl;
	
	//for(int NP=0;NP<4;NP++) cout<< "loading objects" << Objects[NP]->m_tag <<endl;
	// create a new mesh.
	drill = new cMesh(world);

	// load a drill like mesh and attach it to the tool
	fileload = drill->loadFromFile(RESOURCE_PATH("resources/models/drill/drill.3ds"));

	// resize tool mesh model
	drill->scale(0.004);

	// remove the collision detector. we do not want to compute any
	// force feedback rendering on the object itself.
	drill->deleteCollisionDetector(true);

	// define a material property for the mesh

	drill->setMaterial(mat, true);
	drill->computeAllNormals(true);

	// attach drill to tool
	tool->m_proxyMesh->addChild(drill);


	//-----------------------------------------------------------------------
	//					OPEN GL - WINDOW DISPLAY
	//-----------------------------------------------------------------------

	// initialize GLUT
	glutInit(&argc, argv);

	// retrieve the resolution of the computer display and estimate the position
	// of the GLUT window so that it is located at the center of the screen
	int screenW = glutGet(GLUT_SCREEN_WIDTH);
	int screenH = glutGet(GLUT_SCREEN_HEIGHT);
	int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
	int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

	// initialize the OpenGL GLUT window
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(argv[0]);
	glutDisplayFunc(updateGraphics);
	glutKeyboardFunc(keySelect);
	//glutKeyboardUpFunc(keyUp);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("CHAI 3D");

	// create a mouse menu (right button)
	glutCreateMenu(menuSelect);
	glutAddMenuEntry("full screen", OPTION_FULLSCREEN);
	glutAddMenuEntry("window display", OPTION_WINDOWDISPLAY);
	glutAttachMenu(GLUT_RIGHT_BUTTON);


	//-----------------------------------------------------------------------
	//						START SIMULATION
	//-----------------------------------------------------------------------

	// simulation in now running
	simulationRunning = true;

	// create a thread which starts the main haptics rendering loop
	cThread* hapticsThread = new cThread();
	hapticsThread->set(updateHaptics, CHAI_THREAD_PRIORITY_HAPTICS);

	// start the main graphics rendering loop
	glutMainLoop();

	// close everything
	close();

	// exit
	return (0);
}

//---------------------------------------------------------------------------

void resizeWindow(int w, int h){
	// update the size of the viewport
	displayW = w;
	displayH = h;
	glViewport(0, 0, displayW, displayH);
			
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y){
	
	switch(key){
		case 27:
		case 'x':
			close();
			exit(0);
			break;
						
		case '1':	
			if (current_state == 0)	inp_nr = 3;
			break;
			
		case '2':
			if (current_state == 0) inp_nr = 5;
			break;

		case '3':
			if (current_state == 0) inp_nr = 10;
			break;
			
		case 32: // space bar
			space_bar_func();
			if (act_t == true)	tim_d = 19;
			break;

		case 's':
			use_max_force = !use_max_force;
			if (use_max_force == true) cout << "Voice coil actuator always plays on full volume." << endl;
			else cout << "Voice coil actuator volume depends on penetration depth." << endl;
			break;	

		case 'r':
			if (act_t == true)	tim_d = 19;
			current_state = 1;
			gameLogic();
			nr_input_el = false;
			break;	

		case 'i': // reset --> go to beginning of the game
			current_state = 0;
			gameLogic();
			nr_input_el = false;
			break;

		case 'q':
			if((current_state == 0) && (act_obj > 0)) act_obj--;
			gameLogic();
			break;

		case 'w':
			if((current_state == 0) && (act_obj < (NumObj - 1))) act_obj++;
			gameLogic();
			break;

		case 't':
			act_t = !act_t; 
			if (act_t == true) cout << "Time is active." << endl;
			else cout << "Time is NOT active." << endl;
			break;

		case 'z':
			b_vc = !b_vc; 
			if (b_vc == true) cout << "Voice Coil is active." << endl;
			else cout << "Voice Coil is NOT active." << endl;
			break;
			

		case 'f':
			ch_useForceShading = !ch_useForceShading; 
			if (ch_useForceShading == true) cout << "Force shading is active." << endl;
			else cout << "Force shading is NOT active." << endl;
			break;

		case 'v':
			if (current_state == 1){
				handle_visible = !handle_visible; 
				if (handle_visible == true){
					cout << "handle is visible." << endl;
					drill->setTransparencyLevel(1); // lässt stab und kugel erscheinen
					tool->m_proxySphere->setShowEnabled(true);
					tool->getChild(0)->setTransparencyLevel(1);
				}
				else{
					cout << "handle is NOT visible." << endl;
					drill->setTransparencyLevel(0); // lässt stab und kugel verschwinden
					tool->m_proxySphere->setShowEnabled(false);
					tool->getChild(0)->setTransparencyLevel(0);
				}
			}
			break;

		case 13: // enter
			if(nr_input_el == true){
				nr_of_games = inp_nr;
				cout << "number of games: \t" << nr_of_games << endl;
			}
			break;
	}
	redrawUI();
	restart_counter = false;
}

//---------------------------------------------------------------------------

void menuSelect(int value){

	switch (value){
		// enable full screen display
	case OPTION_FULLSCREEN:
		glutFullScreen();
		CounterI.setPos(1530,990,0);	
		CounterC.setPos(1430,990,0);
		CounterR.setPos(700,300,0);
		Time_d2.setPos(1480,970,0);
		break;

		// reshape window to original size
	case OPTION_WINDOWDISPLAY:
		glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
		CounterI.setPos(500,590,0);	
		CounterC.setPos(400,590,0);
		CounterR.setPos(200,200,0);
		Time_d2.setPos(400,570,0);
		break;
	}
}

//---------------------------------------------------------------------------

void close(void){
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	tool->stop();
}

//---------------------------------------------------------------------------

void updateGraphics(void){
	// render world
	camera->renderView(displayW, displayH);

	// Swap buffers
	glutSwapBuffers();

	if (highlight_box == true) highlight_col_box(); // highlight colission box, if user is close to object


	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	// inform the GLUT window to call updateGraphics again (next frame)
	if (simulationRunning)	glutPostRedisplay();
	if (act_t == true){
		if (current_state == 1){
			if (f_first == 1)	time_kk = c_time - 1;
			f_first = false;
			tim_e = clock();
			tim_d = ((float)(tim_e - tim_b))/CLOCKS_PER_SEC;
			tim_d = c_time - tim_d;
			if(tim_d < time_kk){
				time_kk = time_kk - 1;
				redrawUI();
			}
			//cout << "tim_d: " << tim_d << endl;
			if (tim_d < 1.0){
				space_bar_func();
			}
		}
	}
}

//---------------------------------------------------------------------------

void updateHaptics(void){
	
	tool->setShowEnabled(true, true);

	// turn off friction (the original Chai3d friction implementation!)
	tool->m_proxyPointForceModel->m_useFriction = false;

	// main haptic simulation loop
	while(simulationRunning){
		if(b_timing){ if(timing_ctr == 0){ t_begin = clock();}}
		timing_ctr++;

		// update position and orientation of tool
		tool->updatePose();			
		// compute interaction forces
		//if(useFriction) 

		if (ch_useForceShading == true) ((ch_generic3dofPointer*)tool)->computeInteractionForcesD();
		else ((ch_generic3dofPointer*)tool)->computeInteractionForces(); // D

		//((ch_generic3dofPointer*)tool)->computeInteractionForcesD();
		//tool->computeInteractionForces();
		//else tool->computeInteractionForces();

		//Sound
		// check if the tool is touching an object
		ContObject = (cMesh*)tool->m_proxyPointForceModel->m_contactPoint0->m_object;
		GObject = tool->m_proxyPointForceModel->m_contactPoint0->m_object; 
		
		//TODO Change to for Loop if Hstream member keeps empty
		sound_ctr++;
		if (sound_ctr == 100){ // only change sound every 100 iterations
			if(ContObject != NULL){
				if (tool->isInContact(ContObject)){		
					LastID = GObject->getParent()->m_tag;
					//cout << "LastID: " << LastID << endl;
					//cout << "LastID: " <<  << endl;
					if(LastID >= 0) ChangeSound(LastID);	
				}
			}
			else{
				if(LastID >= 0) BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
			}
			sound_ctr = 0;
		}
		
		// send forces to device
	
		tool->applyForces();
		tool->m_proxyPointForceModel->getForce();
		
		// compute global reference frames for each object
		world->computeGlobalPositions(true);

		if(b_timing){
			if (timing_ctr == 1000){
				t_end = clock();
				cout << "Execution time in seconds: \t" << ((float)(t_end-t_begin))/CLOCKS_PER_SEC << std::endl;
				timing_ctr = 0;
			}
		}
		cSleepMs(0.001);
	}
	// exit haptics thread
	simulationFinished = true;
}

//---------------------------------------------------------------------------

void redrawUI(){
	
	if (current_state == 0){
		//cout << "current_state = 0" << endl;
		Wrongs = "  ";
		Rights = "  ";
		Logic1 = " ";
		ctr11 = " "; 
		CounterI.m_string = Wrongs;
		CounterC.m_string = Rights;
		CounterR.m_string = Logic1;
		Time_d2.m_string = ctr11;
	}
	else{
		if (act_t == true){
			if (current_state == 1)	ctr11 = "Remaining Time: " + to_string(tim_d);
			else ctr11 = " ";
		}
		else ctr11 = " ";
		//cout << "current_state = 1" << endl;
		Wrongs = "INCORRECT: " + to_string(inc);
		Rights = "CORRECT: " + to_string(cor);
		Logic0 = "Game Done! Restart with [SPACE] ";
		Logic1 = " ";
		Time_d2.m_string = ctr11;
		CounterI.m_string = Wrongs;
		CounterC.m_string = Rights;
		if (restart_counter == true){
			CounterR.m_string = Logic0;
			inc = 0;
			cor = 0;
		}
		else CounterR.m_string = Logic1;
	}

}

//---------------------------------------------------------------------------

void ChangeSound(int ID){
	// doku: http://www.bass.radio42.com/help/html/f00d6245-b20b-f37d-7982-8cc6549f4ae3.htm
	// http://www.bass.radio42.com/help/html/937729d8-fb7a-497d-a1d5-951f42873d58.htm
	
	force = tool->m_lastComputedGlobalForce;
	device_veloc = tool->m_deviceGlobalVel;
	norm_force = tool->m_proxyPointForceModel->getTangentialForce();
	
	if(use_max_force) abs_force = max_force;
	else abs_force = force.length();

	abs_force = abs_force * veloc_mul[ID];
	if (abs_force > max_force) abs_force = max_force;
	                                
	cos_veloc_angle = (norm_force.dot(device_veloc))/(device_veloc.length()*norm_force.length());
	tan_veloc = (cos_veloc_angle * device_veloc).length();

	//cout << "Objects[ID]: \t" << Objects[ID] << endl;
	//cout << "ID: " << ID << "\t volume: \t" << abs_force/max_force << endl;
	if (b_vc == false) abs_force = 0;

	if (tan_veloc > 1.5){
		if (audio_f != 4) BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
		BASS_ChannelSetAttribute(Objects[ID]->finalStream[4], BASS_ATTRIB_VOL, abs_force/max_force);
		BASS_ChannelPlay(Objects[ID]->finalStream[4],FALSE);
		audio_f = 4;
	}
	else if (tan_veloc > 0.75){
		if (audio_f != 3) BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
		BASS_ChannelSetAttribute(Objects[ID]->finalStream[3], BASS_ATTRIB_VOL, abs_force/max_force);
		BASS_ChannelPlay(Objects[ID]->finalStream[3],FALSE);
		audio_f = 3;
	}
	else if (tan_veloc > 0.5){
		if (audio_f != 2) BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
		BASS_ChannelSetAttribute(Objects[ID]->finalStream[2], BASS_ATTRIB_VOL, abs_force/max_force);
		BASS_ChannelPlay(Objects[ID]->finalStream[2],FALSE);
		audio_f = 2;
	}
	else if (tan_veloc > 0.25){
		if (audio_f != 1) BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
		BASS_ChannelSetAttribute(Objects[ID]->finalStream[1], BASS_ATTRIB_VOL, abs_force/max_force);
		BASS_ChannelPlay(Objects[ID]->finalStream[1],FALSE);
		audio_f = 1;
	}
	else if (tan_veloc > 0.1){
		if (audio_f != 0) BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
		BASS_ChannelSetAttribute(Objects[ID]->finalStream[0], BASS_ATTRIB_VOL, abs_force/max_force);
		BASS_ChannelPlay(Objects[ID]->finalStream[0],FALSE);
		audio_f = 0;
	}
	else BASS_ChannelStop(Objects[LastID]->finalStream[audio_f]);
}


//---------------------------------------------------------------------------
/*
*	Loads the object into workspace
*	if there are multiple children function need to be called for every children with the appropriate children number
*/
void load_object(cMesh *DObject, string f_obj, string i_texture, double i_dyn_fric, double i_stat_fric, double i_stiffness, cVector3d cvd){
	// 1) load object
	// 2) set material (friction, stiffness, ...)
	// 3) load sound files
	// 4) scale and rotate the object
	int numChild;
	t_path = "resources/models/" + f_obj;
	fileload = DObject->loadFromFile(RESOURCE_PATH(t_path));
	for(int sf=0;sf<5;sf++){
		t_val = sf + 1;
		t_ctr = std::to_string(t_val);
		t_path = "resources/sounds/oneSec/" + i_texture + "/f" + t_ctr + ".wav";
		DObject->finalStream[sf] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH(t_path), 0, 0, 0);
	}
	DObject->rotate(cvd, cDegToRad(90));
	numChild = DObject->getNumChildren();

	for(int it=0;it <numChild;it++){
		DObject->getChild(it)->m_material.setDynamicFriction(i_dyn_fric);
		DObject->getChild(it)->m_material.setStaticFriction(i_stat_fric);
		DObject->getChild(it)->m_material.setStiffness(i_stiffness * stiffnessMax);	
		DObject->getChild(it)->m_material.m_ambient = cColorf(1, 1, 1);
	}
}

//---------------------------------------------------------------------------
/*
*	controls the game logic:
*	phase 0: called by pressing [I]
*		user can feel and see the object and switch between them ([Q]: last object, [W]: nex object)
*	phase 2: called by pressing [R]
*		user can feel the object but not see it
*	phase 3: called by pressing [SPACE]
*		three solutions are suggestes --> user needs to choose one
*		touch the object until it turns blue
*	phase 4: called by pressing [SPACE]
*		the box will turn green, if the correct one was chosen
*		the box will turn red and the correct one will turn green, if the wrong one was chosen
*/
void gameLogic(void){
	switch(current_state){
		case 0: // get to know the game phase
			if (handle_visible == false){
					drill->setTransparencyLevel(1); // lässt stab und kugel erscheinen
					tool->m_proxySphere->setShowEnabled(true);
					tool->getChild(0)->setTransparencyLevel(1);
			}
			checkBoxColl = false;
			tool->setWorkspaceRadius(1.0);
			for(int s = 0; s < Objects.size(); s++){
				Objects[s]->setShowBox(false);
				Objects[s]->setPos(0,0,0);
				Objects[s]->setShowEnabled(false,true);
			}
			camera->set( cVector3d (3.0, 0.0, 0.0),	cVector3d (0.0, 0.0, 0.0), cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

			if ((act_obj == 7) || (act_obj == 9)) Objects[act_obj]->setTransparencyLevel(0.2);
			else Objects[act_obj]->setTransparencyLevel(1);

			Objects[act_obj]->setShowEnabled(true, true);
			Objects[act_obj]->setHapticEnabled(true, true);
			break;
			
		case 1: // explore the object
			tim_b = clock();
			f_first = true;
			if (handle_visible == false){
					drill->setTransparencyLevel(0); // lässt stab und kugel verschwinden
					tool->m_proxySphere->setShowEnabled(false);
					tool->getChild(0)->setTransparencyLevel(0);
			}
			checkBoxColl = false;
			tool->setWorkspaceRadius(1.0);
			for(int s = 0; s < Objects.size(); s++){
				Objects[s]->setShowBox(false);
				Objects[s]->setPos(0,0,0);
				Objects[s]->setShowEnabled(false, true); 
			}
			camera->set( cVector3d (3.0, 0.0, 0.0), cVector3d (0.0, 0.0, 0.0), 	cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
			
			realModel = rand() % NumObj; // choose random object
			Objects[realModel]->setShowEnabled(true, true);
			Objects[realModel]->setHapticEnabled(true,true);
			Objects[realModel]->setTransparencyLevel(0);
			break;

		case 2 : // choose the solution
			if (handle_visible == false){
					drill->setTransparencyLevel(1); // lässt stab und kugel erscheinen
					tool->m_proxySphere->setShowEnabled(true);
					tool->getChild(0)->setTransparencyLevel(1);
			}
			if(newButPush == 1 && checkBoxColl == false){ // 
				vector <int> list;
				//fill list
				for(int l = 0; l < Objects.size(); l++){
					Objects[l]->setShowEnabled(false,true);		
					list.push_back(l);
				}
				list.erase(list.begin() + realModel);
				//Choose Random elements from List
				int el;
				for(int i = 0; i < 2; i++){
					el = rand() % list.size();
					multChoice[i] = list.at(el);
					list.erase(list.begin() + el);
				}
				multChoice[2] = realModel;
				//Shuffle Multiple Choice Elements
				int buf1;
				int swap;
				for(int w = 0; w < 2; w++){	
					swap = rand() % 2;
					buf1 = multChoice[2];
					multChoice[2] = multChoice[swap];
					multChoice[swap] = buf1;
				}
				// position the elements
				Objects[multChoice[0]]->setPos(0.0, -2, 0.0);
				Objects[multChoice[1]]->setPos(0.0, 0.0, 0.0);
				Objects[multChoice[2]]->setPos(0.0, 2, 0.0);	

				for(int k = 0; k < 3; k++){
					Objects[multChoice[k]]->setShowEnabled(true,true);
					if ((act_obj == 7) || (act_obj == 9)) Objects[multChoice[k]]->setTransparencyLevel(0.2);
					else Objects[multChoice[k]]->setTransparencyLevel(1);
					//Objects[multChoice[k]]->setTransparencyLevel(1);

					//Objects[multChoice[k]]->setUseTransparency(false, true);
					Objects[multChoice[k]]->setShowBox(true);
					Objects[multChoice[k]]->setBoxColor(cColorf(1,0,0,1));
				
					distToObj[k] = Objects[multChoice[k]]->getBoundaryMax().distance(Objects[multChoice[k]]->getBoundaryCenter());
					//Objects[multChoice[k]]->setAsGhost(true);
					Objects[multChoice[k]]->setHapticEnabled(false,true);
					/////////////////////neu //////////////////////////////
					Objects[multChoice[k]]->setShowEnabled(true, true);
				}
				/*Objects[7]->setTransparencyLevel(0.2);
				Objects[9]->setTransparencyLevel(0.2);*/
				
				camera->set( cVector3d (9.0, 0.0, 0.0), cVector3d (0.0, 0.0, 0.0),cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
				tool->setWorkspaceRadius(5.0);
				checkBoxColl = true;
				newButPush = false;
			}
			break;

		case 3: // check result
			if (handle_visible == false){
					drill->setTransparencyLevel(1); // lässt stab und kugel erscheinen
					tool->m_proxySphere->setShowEnabled(true);
					tool->getChild(0)->setTransparencyLevel(1);
			}
			if(checkBoxColl == true){
				if(selectedModel == realModel){ // user chose the correct object
					for(int k = 0; k < 3; k++){ // loop through all 3 objects
						if (multChoice[k] == selectedModel) {
							Objects[multChoice[k]]->setShowBox(true);
							Objects[selectedModel]->setBoxColor(cColorf(0,1,0,1)); // green
						}
						else{
							Objects[multChoice[k]]->setShowBox(false);
						}
					}		
					cor++;
					redrawUI();
				}
				else{
					inc++;
					redrawUI();
					Objects[selectedModel]->setShowBox(true);
					Objects[realModel]->setShowBox(true);
					Objects[selectedModel]->setBoxColor(cColorf(1,0,0,1)); // red
					Objects[realModel]->setBoxColor(cColorf(0,1,0,1)); // green
				}
				checkBoxColl = false;
			}
			if ((inc + cor) == nr_of_games){
				cout <<"######################################################################" << endl;
				cout <<"############## CONGRATULATIONS YOU FINISHED THE GAME #################" << endl;
				cout <<"######################################################################" << endl;
				cout <<"#\t \t number of false answers:" << inc << endl;
				cout <<"#\t \t number of correct answers:" << cor <<  endl;
				cout <<"######################################################################" << endl << endl ;
				// cout <<"Press [R] for new Game"<<endl;
				restart_counter = true;
				redrawUI();
			}
			break;
		}
}

//---------------------------------------------------------------------------

void highlight_col_box(void){
	for (int j = 0; j < 3; j++){
		if (tool->getDeviceGlobalPos().distance(Objects[multChoice[j]]->getGlobalPos()) < distToObj[j]){
			Objects[multChoice[j]]->setBoxColor(cColorf(0.24, 0.47, 0.85, 1));
		}
		else Objects[multChoice[j]]->setBoxColor(cColorf(1, 1, 1, 1));
	}
}

//---------------------------------------------------------------------------

void space_bar_func(void){
	/*if (restart_counter == true){
		inc = 0;
		cor = 0;
		redrawUI();
		restart_counter = false;
	}*/
	redrawUI();
	if (current_state == 1){
		current_state = 2;
		highlight_box = true;
	}
	else if (current_state == 2){
		for (int j = 0; j < 3; j++){
			if (tool->getDeviceGlobalPos().distance(Objects[multChoice[j]]->getGlobalPos()) < distToObj[j]){ // check if decive is next to object
				//Objects[multChoice[j]]->setBoxColor(cColorf(0, 1, 0, 1)); // change color to green
				for (int s = 0; s < 3; s++){ Objects[multChoice[s]]->setShowBox(false); }
				selectedModel = multChoice[j];
				current_state = 3;
				highlight_box = false;
				}
				//else Objects[multChoice[j]]->setBoxColor(cColorf(1, 0, 0, 1));
			}
		}
	else if (current_state == 3){
		current_state = 1;
		//gameLogic();
		nr_input_el = false;
	}
	newButPush = true;
	gameLogic();
	redrawUI();
}

//---------------------------------------------------------------------------

// random bullsch!

//void InitStream(){
//	//for(int i=0; i<=5;i++){
//	//	this->stream_length[i]= BASS_ChannelGetLength(this->file_stream[i], 0);
//	//	BASS_ChannelGetInfo(this->file_stream[i], &this->infoBass[i]);
//	//	this->data[i] = new char[(unsigned int)this->stream_length[i]];
//	//	BASS_ChannelGetData(this->file_stream[i], this->data[i], (unsigned int)this->stream_length[i]);		
//	//	//stream[i] = BASS_StreamCreate(infoBass[i].freq, infoBass[i].chans, 0, &MyStreamWriter, 0);
//	//	stream[i] = BASS_StreamCreate(infoBass[i].freq, infoBass[i].chans, 0,0, 0);
//	//}
//	
//		stream_length[0]=BASS_ChannelGetLength(file_stream[0], 0);
//		//this->stream_length[0]= BASS_ChannelGetLength(this->file_stream[0], 0);
//		BASS_ChannelGetInfo(file_stream[0], &infoBass[0]);
//		//BASS_ChannelGetInfo(this->file_stream[0], &this->infoBass[0]);
//		data[0] = new char[(unsigned int)stream_length[0]];
//		//this->data[0] = new char[(unsigned int)this->stream_length[0]];
//		BASS_ChannelGetData(file_stream[0], data[0], (unsigned int)stream_length[0]);	
//		//BASS_ChannelGetData(this->file_stream[0], this->data[0], (unsigned int)this->stream_length[0]);	
//
//		
//		stream[0] = BASS_StreamCreate(infoBass[0].freq, infoBass[0].chans, 0,(STREAMPROC*)MyStreamWriter, 0);
//		//stream[0] = BASS_StreamCreate(infoBass[0].freq, infoBass[0].chans, 0, &cMesh::MyStreamWriter, 0);
//		
//		//stream[0] = BASS_StreamCreate(infoBass[0].freq, infoBass[0].chans, 0,0, 0);
//		
//	
//
//}

//DWORD MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user)
//{
//	
//
//	//int tb_mulop = pos;
//    // Cast the buffer to a character array
//	char *d=(char*)buf;
//
//    // Loop the file when it reaches the beginning or end
//    if ((pos >= stream_length[0]) && (record_direction == 1))
//		    pos = 0;
//	if ((pos <= 0) && (record_direction == -1))
//		    pos = (unsigned int)stream_length[0];
//
//	if (pos == 0){
//		
//	}
//
//	// If record is spinning in positive direction, write requested
//	// amount of data from current position forwards
//	if (record_direction == 1)
//	{
//		int up = len + pos;
//		if (up > stream_length[0])
//			up = (unsigned int)stream_length[0];
//
//    for (int i=pos; i<up; i+=1)
//			d[(i-pos)] = data[0][i];
//
//		int amt = (up-pos);
//		pos += amt;
//		return amt;
//	 }
//
//    // If record is spinning in negative direction, write requested
//	// amount of data from current position backwards
//	if (record_direction == -1)
//	{
//		int up = pos - len;
//
//		if (up < 0)
//			up = 0;
//
//	    int cnt = 0;
//        for (int i=pos; i>up; i-=1)
//                d[cnt++] = data[0][i];
//
//		int amt = cnt;
//		pos -= amt;
//
//		return amt;
//	 }
//
//	 return 0;
//}


// sound bullsch
//Load Sound Files
	//// Load the data from the specified file
	//   HSTREAM file_stream = 1;
	//   file_stream = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,BASS_STREAM_DECODE);

	//HSTREAM file_stream_mat[NBR_TEXTURES][NBR_VELOCITIES] = {1,1,1,1,1};
	//file_stream_mat[0][0] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/horse.wav"),0,0,BASS_STREAM_DECODE);
	//file_stream_mat[0][1] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/lion.wav"),0,0,BASS_STREAM_DECODE);
	//file_stream_mat[0][2] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/bird.wav"),0,0,BASS_STREAM_DECODE);
	//file_stream_mat[0][3] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/cow.wav"),0,0,BASS_STREAM_DECODE);
	//file_stream_mat[0][4] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/duck.wav"),0,0,BASS_STREAM_DECODE);

	//if (file_stream_mat[0][0] == 0)
	//{
	//	#if defined(_msvc)
	//	file_stream_mat[0][0] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/horse.wav"),0,0,bass_stream_decode);
	//	file_stream_mat[0][1] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/lion.wav"),0,0,bass_stream_decode);
	//	file_stream_mat[0][2] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/bird.wav"),0,0,bass_stream_decode);
	//	file_stream_mat[0][3] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/cow.wav"),0,0,bass_stream_decode);
	//	file_stream_mat[0][4] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/duck.wav"),0,0,bass_stream_decode);
	//	//file_stream = bass_streamcreatefile(false,"../../../bin/resources/sounds/classic.mp3",0,0,bass_stream_decode);
	//	#endif
	//}
	//if (!fileload)
	//{
	//       printf("error - mp3 audio file failed to load correctly.\n");
	//       close();
	//       return (-1);
	//   }


	/////////////////////////////////////////////////////////////////////////////////////
	//								bla bla bla										   //
	/////////////////////////////////////////////////////////////////////////////////////
	//for( int text = 0; text < NBR_TEXTURES; text = text + 1 ){	
	//	for( int veloc = 0; veloc < NBR_VELOCITIES; veloc = veloc + 1 ){
	//		stream_length_mat[text][veloc] = BASS_ChannelGetLength(file_stream_mat[text][veloc], 0);
	//		BASS_ChannelGetInfo(file_stream_mat[text][veloc], &infoBass_mat[text][veloc]);
	//		data_mat[text][veloc] = new char[(unsigned int)stream_length_mat[text][veloc]];
	//		BASS_ChannelGetData(file_stream_mat[text][veloc], data_mat[text][veloc], (unsigned int)stream_length_mat[text][veloc]);
	//		
	//		/*data = data_mat[text][veloc];
	//		stream_length = stream_length_mat[text][veloc];*/

	//		stream_mat[text][veloc] = BASS_StreamCreate(infoBass_mat[text][veloc].freq, infoBass_mat[text][veloc].chans, 0, &MyStreamWriter, 0);
	//		std::cout << "stream: " << stream_mat[text][veloc] << std::endl;
	//		std::cout << "data_mat:  " << sizeof(*data_mat[text][veloc]) <<"|| stream_length:  " << stream_length_mat[text][veloc] << std::endl;
	//	}
	//}

		
		//DObject->InitStream();
		//InitStream();
		//DObject->finalStream[0]=stream[0];
		//BASS_ChannelPlay(DObject->finalStream[0],FALSE);

// material bullsch


	//cMaterial tooth_mat;
	//tooth_mat.m_ambient.set(0.5, 0.5, 0.5);		
	//tooth_mat.m_specular.set(1.0, 1.0, 1.0);
	//tooth_mat.setStiffness(0.8*stiffnessMax); //MaxStiffness:74, Max Force:4;
	//tooth_mat.setDynamicFriction(0.2);
	//tooth_mat.setStaticFriction(0.15);


	//cMaterial rock_mat;
	//rock_mat.m_ambient.set(0.5, 0.5, 0.5);
	////rock_mat.m_diffuse.set(0.1, 0.8, 0.8);
	//rock_mat.m_specular.set(1.0, 1.0, 1.0);
	//rock_mat.setStiffness(0.8*stiffnessMax); //MaxStiffness:74, Max Force:4;
	//rock_mat.setDynamicFriction(0.45);
	//rock_mat.setStaticFriction(0.4);


// bullsch function

//---------------------------------------------------------------------------

//void StartPlayback(cMesh* Obj){	
//
//
//	//BASS_ChannelSetAttribute(Obj->stream[0], BASS_ATTRIB_FREQ, 500);
//  
//	//cout<< Obj->finalStream[0];
//	//BASS_ChannelPlay(Obj->finalStream[0],FALSE);
//	//cout << BASS_ChannelPlay(file_stream[0],FALSE);
//	//BASS_ChannelPlay(Obj->getParent()->,FALSE);
//	
//	cout << "warsch" << BASS_ErrorGetCode();
//}

// bullsch nach cases
		
		//// make the outside of the DObject rendered in wireframe
		//((cMesh*)(DObject->getChild(1)))->setWireMode(true);

		//// make the outside of the DObject rendered in semi-transparent
		//((cMesh*)(DObject->getChild(1)))->setUseTransparency(false);
		//((cMesh*)(DObject->getChild(1)))->setTransparencyLevel(transparencyLevel);

		// resize DObject to screen

// anderer bullsch

//void load_soundfiles(cMesh *DObject ,string texture){
//	for(int sf=0;sf<5;sf++){
//		t_val = sf + 1;
//		t_ctr = std::to_string(t_val);
//		t_path = "resources/sounds/oneSec/" + texture + "/f" + t_ctr + ".wav";
//		DObject->finalStream[sf]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH(t_path),0,0,0);
//	}
//}

// alter bullsch

// ch lab
// set static and dynamic friction coefficients for the object, propagating 
// them to the children
//void ch_setFrictionCoefficients(cGenericObject* obj, const double& static_coeff, const double& dynamic_coeff){
//	//obj->m_material.setDynamicFriction(dynamic_coeff);
//	//obj->m_material.setStaticFriction(static_coeff);
//
//	//for(unsigned int i = 0; i < obj->getNumChildren(); i++)
//	//{
//	//	ch_setFrictionCoefficients(obj->getChild(i), static_coeff, dynamic_coeff);		
//	//}	
//}

// anderer bullsch
//	if (!fileload)
//	{
//#if defined(_MSVC)
//		fileload = drill->loadFromFile("../../../bin/resources/models/drill/drill.3ds");
//#endif
//	}
//	if (!fileload)
//	{
//		printf("Error - 3D Model failed to load correctly.\n");
//		close();
//		return (-1);
//	}

// alte game logic
//---------------------------------------------------------------------------

//void startGame(void){
//	for(int s = 0; s < 3; s++){
//		Objects[multChoice[s]]->setShowBox(false);
//	}
//	curState=0;
//	checkBoxColl=false;
//	tool->setWorkspaceRadius(1.0);
//	std::cout << "# of objects: \t" << Objects.size() << std::endl;
//	for(int s = 0; s < Objects.size(); s++){
//		Objects[s]->setShowBox(false);
//		Objects[s]->setPos(0,0,0);
//	}
//
//	camera->set( cVector3d (3.0, 0.0, 0.0),    // camera position (eye)
//		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
//		cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
//
//	realModel=rand() % NumObj;
//	cout<<"Model "<<realModel<<endl;
//	for(int l = 0; l < Objects.size(); l++){Objects[l]->setShowEnabled(false,true);}
//	Objects[realModel]->setShowEnabled(true, true);
//	Objects[realModel]->setHapticEnabled(true,true);
//
//	Objects[realModel]->setTransparencyLevel(0);
//}

//---------------------------------------------------------------------------

//void selectSolution(void){
//	if(newButPush==1 && checkBoxColl==false){ // button==1 && 
//		vector <int> list;
//		//fill list
//		for(int l=0;l<Objects.size();l++){
//			Objects[l]->setShowEnabled(false,true);		
//			list.push_back(l);
//		}
//		list.erase(list.begin()+realModel);
//		//Choose Random elements from List
//		int el;
//		for(int i=0;i<2;i++){
//			el=rand() % list.size();
//
//			multChoice[i] = list.at(el);
//			//std::cout <<"El:" << multChoice[i];
//			list.erase(list.begin() + el);
//		}
//		multChoice[2] = realModel;
//		//Shuffle Multiple Choice Elements
//		int buf1;
//		int swap;
//		for(int w = 0; w < 2; w++)
//		{	
//			swap=rand()%2;
//			buf1 = multChoice[2];
//			multChoice[2]=multChoice[swap];
//			multChoice[swap]=buf1;
//		}
//
//		Objects[multChoice[0]]->setPos(0.0, -2, 0.0);
//		Objects[multChoice[1]]->setPos(0.0, 0.0, 0.0);
//		Objects[multChoice[2]]->setPos(0.0, 2, 0.0);	
//
//		for(int k=0;k<3;k++){
//			Objects[multChoice[k]]->setShowEnabled(true,true);	
//			Objects[multChoice[k]]->setUseTransparency(false,true);
//			Objects[multChoice[k]]->setShowBox(true);
//			Objects[multChoice[k]]->setBoxColor(cColorf(1,0,0,1));
//			distToObj[k] = Objects[multChoice[k]]->getBoundaryMax().distance(Objects[multChoice[k]]->getBoundaryCenter());
//		}
//		camera->set( cVector3d (9.0, 0.0, 0.0),    // camera position (eye)
//			cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
//			cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
//		tool->setWorkspaceRadius(5.0);
//		checkBoxColl = true;
//		newButPush = false;
//	}
//
//	else if(checkBoxColl == true){
//
//		//tool->m_proxyPointForceModel->m_contactPoint0->m_object->m_tag;
//
//		for(int j = 0; j < 3; j++){
//			//distToObj[j] = Objects[multChoice[j]]->getBoundaryMax().distance(Objects[multChoice[j]]->getBoundaryCenter());
//			//if(tool->isInContact(Objects[multChoice[0]])){
//			if(tool->getDeviceGlobalPos().distance(Objects[multChoice[j]]->getGlobalPos()) < distToObj[j] ){
//				Objects[multChoice[j]]->setBoxColor(cColorf(0,1,0,1));					
//				//Objects[multChoice[j]]->m_material.m_diffuse.set(0,0,1);
//				if(newButPush==1){		// && button==1	
//					for(int s = 0; s < 3; s++){
//						Objects[multChoice[s]]->setShowBox(false);
//					}
//					selectedModel = multChoice[j];
//					newResult = true;
//					curState = 1;
//				}
//			}
//			else Objects[multChoice[j]]->setBoxColor(cColorf(1,0,0,1));	
//		}
//	}
//}

//---------------------------------------------------------------------------

//void nextState(void){
//
//// was soll das???
//}

//---------------------------------------------------------------------------

//void checkSolution(void){		
//	if(newResult){
//		if(selectedModel == realModel)
//		{
//			//Objects[selectedModel]->setShowBox(true);
//			Objects[selectedModel]->setBoxColor(cColorf(0,1,0,1));
//			cout<<"RIGHT"<<endl;			
//			cor++;
//			redrawUI();
//			cout <<"Press n for new Game"<<endl;
//			
//		}
//		else{
//			cout<<"WRONG"<<endl;
//			cout <<"Press n for new Game"<<endl;
//			inc++;
//			redrawUI();
//			Objects[selectedModel]->setShowBox(true);
//			//Objects[realModel]->setShowBox(true);
//			Objects[selectedModel]->setBoxColor(cColorf(1,0,0,1));
//			Objects[realModel]->setBoxColor(cColorf(0,1,0,1));
//		}
//		for(int s = 0; s < 3; s++){
//			Objects[multChoice[s]]->setShowBox(false);
//		}
//		newResult=false;
//	}
//	/*close();
//	exit(0);*/
//}

//start game logic
//void startGame(void);
//void selectSolution(void);
//void nextState(void);
//void checkSolution(void);

//// start playback
//void StartPlayback(cMesh* Obj);

/*case '4':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[4]->setShowEnabled(true,true);
			break;*/

		/*case '5':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[5]->setShowEnabled(true,true);
			break;*/

		/*case '6':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[6]->setShowEnabled(true,true);
			break;
		
		case '7':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[7]->setShowEnabled(true,true);
			break;*/

		//case '-':
		//	//	// decrease transparency level
		//	//	transparencyLevel = transparencyLevel - 0.1;
		//	//	if (transparencyLevel < 0.0) { transparencyLevel = 0.0; }

		//	//	// apply changes to DObject
		//	//	((cMesh*)(DObject->getChild(1)))->setTransparencyLevel(transparencyLevel);
		//	//	((cMesh*)(DObject->getChild(1)))->setUseTransparency(true);

		//	//	((cMesh*)(DObject->getChild(0)))->setTransparencyLevel(transparencyLevel);
		//	//	((cMesh*)(DObject->getChild(0)))->setUseTransparency(true);

		//	//	// if object is almost transparent, make it invisible
		//	//	if (transparencyLevel < 0.1){
		//	//		//((cMesh*)(DObject->getChild(1)))->setShowEnabled(false, true);
		//	//	}
		//	break;

		//case '+':
		//	//	// increase transparency level
		//	//	transparencyLevel = transparencyLevel + 0.1;
		//	//	if (transparencyLevel > 1.0) { transparencyLevel = 1.0; }

		//	//	// apply changes to DObject
		//	//	((cMesh*)(DObject->getChild(1)))->setTransparencyLevel(transparencyLevel);

		//	//	// make object visible
		//	//	if (transparencyLevel >= 0.1)
		//	//	{
		//	//		((cMesh*)(DObject->getChild(1)))->setShowEnabled(true, true);
		//	//	}

		//	//	// disable transparency is transparency level is set to 1.0
		//	//	if (transparencyLevel == 1.0)
		//	//	{
		//	//		((cMesh*)(DObject->getChild(1)))->setUseTransparency(false);
		//	//	}
		//	break;