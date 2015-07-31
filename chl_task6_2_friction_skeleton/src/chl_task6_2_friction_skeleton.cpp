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
//#include <CMesh.h>

//---------------------------------------------------------------------------
#include "ch_generic3dofPointer.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;

using namespace std;

//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

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
_Longlong inc=0;
_Longlong cor=0;

void redrawUI();
// a haptic device handler
cHapticDeviceHandler* handler;

// information about the current haptic device
cHapticDeviceInfo info;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;
//ch_generic3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a virtual DObject mesh
//cMesh* DObject;
vector<cMesh*> Objects;
//cMesh* DObject;

int aktObj=0;

//Set number of Objects
int NumObj=6;


int multChoice[3];
static int button;
int realModel;
int selectedModel;
bool newResult;

double distToObj[3];
bool newButPush=false;

// transparency level
double transparencyLevel = 0.3;
int curState=0;
// virtual drill mesh
cMesh* drill;

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


/////////////////////////////////////////////////////////////////////////////////////
//								AUDIO								   //
/////////////////////////////////////////////////////////////////////////////////////

//DWORD MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user);
//void InitStream();


// Global variables for the audio stream
static const int numStreams=5;
//HSTREAM file_stream[numStreams];
//HSTREAM stream[numStreams];
//QWORD stream_length[numStreams];
//BASS_CHANNELINFO infoBass[numStreams];
//char *data[numStreams];


int LastID;
// Write the requested data from the loaded buffer to the sound card
//DWORD CALLBACK MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user);
void StartPlayback(cMesh* Obj);





//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
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

//start game logic
void startGame(void);
void selectSolution(void);
void nextState(void);
void checkSolution(void);

// ch lab
double static_coeff;
double dynamic_coeff;
bool useFriction = true;

// set static and dynamic friction coefficients for the object, propagating 
// them to the children
void ch_setFrictionCoefficients(cGenericObject* obj, const double& static_coeff, const double& dynamic_coeff);

// modify sound according to user action
void ChangeSound(int ID);

// load all five sound files of specific texture
void load_soundfiles(std::string texture);

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
	// INITIALIZATION
	//-----------------------------------------------------------------------

	printf ("\n");
	printf ("-----------------------------------\n");
	printf ("CHAI 3D\n");
	printf ("Demo: CHL friction rendering\n");
	printf ("Copyright 2003-2009\n");
	printf ("-----------------------------------\n");
	printf ("\n\n");
	printf ("Instructions:\n\n");
	printf ("- Use haptic device and user switch to feel the shape \n");
	printf ("  and guess the object. \n");
	printf ("\n\n");
	printf ("Keyboard Options:\n\n");
	printf ("[1-5] - Model 1-N \n");	
	printf ("[x] - Exit application\n");
	printf ("[N] -New Game \n");
	printf ("\n\n");

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);



	srand (time(NULL));

	//-----------------------------------------------------------------------
	// 3D - SCENEGRAPH
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

	// create a light source and attach it to the camera
	light = new cLight(world);
	camera->addChild(light);                   // attach light to camera
	light->setEnabled(true);                   // enable light source
	light->setPos(cVector3d( 2.0, 0.5, 1.0));  // position the light source
	light->setDir(cVector3d(-2.0, 0.5, 1.0));  // define the direction of the light beam


	//-----------------------------------------------------------------------
	// 2D - WIDGETS
	//-----------------------------------------------------------------------

	// create a 2D bitmap logo
	logo = new cBitmap();

	// add logo to the front plane
	camera->m_front_2Dscene.addChild(logo);

	// load a "chai3d" bitmap image file
	bool fileload;
	fileload = logo->m_image.loadFromFile(RESOURCE_PATH("resources/images/chai3d.bmp"));
	if (!fileload)
	{
#if defined(_MSVC)
		fileload = logo->m_image.loadFromFile("../../../bin/resources/images/chai3d.bmp");
#endif
	}

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

	CounterI.m_fontColor=cColorf(1,0,0,1);
	CounterC.m_fontColor=cColorf(0,1,0,1);

	CounterI.setPos(500,590,0);	
	CounterC.setPos(400,590,0);		
	
	redrawUI();
	

	//-----------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	cGenericHapticDevice* hapticDevice;
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	if (hapticDevice)
	{
		info = hapticDevice->getSpecifications();
	}

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
	double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;


	// Initialize sound device and create audio stream
	BASS_Init(1,44100,0,0,NULL);

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






	//-----------------------------------------------------------------------
	// COMPOSE THE VIRTUAL SCENE
	//-----------------------------------------------------------------------

	// create a virtual mesh

	//Objects = new cMesh(world)[5];

	//Materials
	cMaterial mat;
	mat.m_ambient.set(0.5, 0.5, 0.5);
	mat.m_diffuse.set(0.8, 0.8, 0.8);
	mat.m_specular.set(1.0, 1.0, 1.0);


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




	// switch chooses the correct object and sets the attributes of the object
	// 1) load object
	// 2) set material (friction, stiffness, ...)
	// 3) scale and rotate the object
	// 4) activate texture
	// 5) load sound files
	for(int it=0;it<NumObj;it++){
		cMesh *DObject = new cMesh(world);
		//DObject=new cMesh(world);
		// add object to world
		world->addChild(DObject);

		// set the position and orientation of the object 
		DObject->setPos(0.0, 0.0, 0.0);
		//DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(-10));
		//DObject->rotate(cVector3d(0.0, 1.0, 0.0), cDegToRad(10));
		DObject->setTag(it);
		fileload = false;
		// load an object file
		switch(it){

		case 0: // tooth			
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/tooth/tooth.3ds"));

			// dyn_fric = 0.2 stat_fric = 0.15, stiff = 0.8
			DObject->getChild(0)->m_material.setDynamicFriction(0.2);
			DObject->getChild(0)->m_material.setStaticFriction(0.15);
			DObject->getChild(0)->m_material.setStiffness(0.8*stiffnessMax);		
			
			texture = Glass;
			for(int sf=0;sf<5;sf++){DObject->finalStream[sf]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/oneSec/" + texture + "/f" + (sf+1)),0,0,0);}
			/*DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/oneSec/Glass/f1"),0,0,0);
			DObject->finalStream[1]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/oneSec/Glass/f2"),0,0,0);
			DObject->finalStream[2]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/oneSec/Glass/f3"),0,0,0);
			DObject->finalStream[3]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/oneSec/Glass/f4"),0,0,0);
			DObject->finalStream[4]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/oneSec/Glass/f5"),0,0,0);*/
			break;

		case 1: // bunny
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/bunny/bunny.obj"));

			// dyn_fric = 0.3, stat_fric = 0.4, stiff = 0.2
			DObject->getChild(0)->m_material.setDynamicFriction(0.3);
			DObject->getChild(0)->m_material.setStaticFriction(0.4);
			DObject->getChild(0)->m_material.setStiffness(0.2*stiffnessMax);

			//DObject->setMaterial(tooth_mat, true);
			// scaling & rotating
			//DObject->scale(0.8);
			DObject->rotate(cVector3d(1.0, 0.0, 0.0), cDegToRad(90));

					
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/Kalimba.mp3"),0,0,0);
			break;

		case 2:	// rock,	granite
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/Stone/Rock1.obj"));

			// dyn_fric = 0.3, stat_fric = 0.43, stiff = 0.6
			DObject->getChild(0)->m_material.setDynamicFriction(0.3);
			DObject->getChild(0)->m_material.setStaticFriction(0.43);
			DObject->getChild(0)->m_material.setStiffness(0.6*stiffnessMax);

			// scaling & rotating
			//DObject->scale(0.3);
			DObject->rotate(cVector3d(1.0, 0.0, 0.0), cDegToRad(90));		
		
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/Maid with the Flaxen Hair.mp3"),0,0,0);
			break;

		case 3:	// sponge
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/Schwamm/sponge.obj"));

			// upper part: dyn_fric = 0.2, stat_fric = 0.25, stiff = 0.1		
			DObject->getChild(1)->m_material.setDynamicFriction(0.2);
			DObject->getChild(1)->m_material.setStaticFriction(0.25);
			DObject->getChild(1)->m_material.setStiffness(0.1*stiffnessMax);
			// power part: dyn_fric = 0.3, stat_fric = 0.4, stiff = 0.2
			DObject->getChild(0)->m_material.setDynamicFriction(0.3);
			DObject->getChild(0)->m_material.setStaticFriction(0.4);
			DObject->getChild(0)->m_material.setStiffness(0.2*stiffnessMax);

			//DObject->setMaterial(sponge_mat, true);
			// scaling & rotating
			//DObject->scale(0.1);
			DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(90));			
			
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/Sleep Away.mp3"),0,0,0);
			break;

		case 4:	// rock, sandstone		
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/Stone/Rock_rough.obj"));
			
			// dyn_fric = 0.4, stat_fric = 0.51, stiff = 0.6
			/*DObject->getChild(0)->m_material.setDynamicFriction(0.4);
			DObject->getChild(0)->m_material.setStaticFriction(0.51);
			DObject->getChild(0)->m_material.setStiffness(0.6*stiffnessMax);*/


			DObject->m_material.setDynamicFriction(0.4);
			DObject->m_material.setStaticFriction(0.51);
			DObject->m_material.setStiffness(0.6*stiffnessMax);
			

			// DObject->setMaterial(rock_mat, true);
			// scaling & rotating
			
			DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(90));
			
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);
			break;

			
			//Cork doesn't work
		 case 5:	 // Cork
			
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/Cork/cork.obj"));
			
			// dyn_fric = 0.5, stat_fric = 0.5, stiff = ??
			DObject->getChild(0)->m_material.setDynamicFriction(0.5);
			DObject->getChild(0)->m_material.setStaticFriction(0.5);
			DObject->getChild(0)->m_material.setStiffness(0.2*stiffnessMax);

			// scaling & rotating
			DObject->scale(0.003);
			DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(90));

			DObject->setUseTexture(true);					
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);	
			break;	 	
			
			/* case 6:	// bottle
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/cork/cork.obj"));

			// dyn_fric = 0.2, stat_fric = 0.25, stiff = 0.2
			DObject->getChild(0)->m_material.setDynamicFriction(0.2);
			DObject->getChild(0)->m_material.setStaticFriction(0.25);
			DObject->getChild(0)->m_material.setStiffness(0.2*stiffnessMax);

			// scaling & rotating
			DObject->scale(0.003);
			DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(90));

			DObject->setUseTexture(true);
			// DObject->file_stream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,BASS_STREAM_DECODE);	
			file_stream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);		
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);	
			break;	 */	
			
			/* case 7:	// ice
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/cork/cork.obj"));

			// dyn_fric = 0.02, stat_fric = 0.03, stiff = 0.8
			DObject->getChild(0)->m_material.setDynamicFriction(0.02);
			DObject->getChild(0)->m_material.setStaticFriction(0.03);
			DObject->getChild(0)->m_material.setStiffness(0.8*stiffnessMax);

			// scaling & rotating
			DObject->scale(0.003);
			DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(90));

			DObject->setUseTexture(true);
			// DObject->file_stream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,BASS_STREAM_DECODE);	
			file_stream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);		
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);	
			break;	 */	
			
			
		/* case 8:	// (shot) glass
			fileload = DObject->loadFromFile(RESOURCE_PATH("resources/models/cork/cork.obj"));

			// dyn_fric = 0.15, stat_fric = 0.19, stiff = 0.8
			DObject->getChild(0)->m_material.setDynamicFriction(0.15);
			DObject->getChild(0)->m_material.setStaticFriction(0.19);
			DObject->getChild(0)->m_material.setStiffness(0.8*stiffnessMax);

			// scaling & rotating
			DObject->scale(0.003);
			DObject->rotate(cVector3d(0.0, 0.0, 1.0), cDegToRad(90));

			DObject->setUseTexture(true);
			// DObject->file_stream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,BASS_STREAM_DECODE);	
			file_stream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);		
			DObject->finalStream[0]=BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,0);	
			break;	 */	
		} // end of case statement
		
		//// make the outside of the DObject rendered in wireframe
		//((cMesh*)(DObject->getChild(1)))->setWireMode(true);

		//// make the outside of the DObject rendered in semi-transparent
		//((cMesh*)(DObject->getChild(1)))->setUseTransparency(false);
		//((cMesh*)(DObject->getChild(1)))->setTransparencyLevel(transparencyLevel);

		// resize DObject to screen

		
		// 5) compute boundary box
		// 6) create collision detector
		// 8) intialize stream
			// get dimensions of object
		double size = cSub(DObject->getBoundaryMax(), DObject->getBoundaryMin()).length();

		// resize object to screen
		if (size > 0)
		{
			DObject->scale( 2.0 * tool->getWorkspaceRadius() / size);
		}

		DObject->setUseTexture(true);
		
		//// compute a boundary box
		DObject->computeBoundaryBox(true);
		// compute collision detection algorithm		
		DObject->createAABBCollisionDetector(1.01 * proxyRadius, true, false);
		DObject->setShowEnabled(false, true);
	
		
		//DObject->InitStream();
		//InitStream();
		//DObject->finalStream[0]=stream[0];
		//BASS_ChannelPlay(DObject->finalStream[0],FALSE);


		Objects.push_back(DObject);	

		//TODO Ghost objects

	}
	for(int NP=0;NP<4;NP++){
		cout << Objects[NP]->m_tag <<endl;
	}
	// create a new mesh.
	drill = new cMesh(world);

	// load a drill like mesh and attach it to the tool
	fileload = drill->loadFromFile(RESOURCE_PATH("resources/models/drill/drill.3ds"));
	if (!fileload)
	{
#if defined(_MSVC)
		fileload = drill->loadFromFile("../../../bin/resources/models/drill/drill.3ds");
#endif
	}
	if (!fileload)
	{
		printf("Error - 3D Model failed to load correctly.\n");
		close();
		return (-1);
	}

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
	// OPEN GL - WINDOW DISPLAY
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
	// START SIMULATION
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
			
		case '0':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[0]->setShowEnabled(true,true);
			break;
			
		case '1':	
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[1]->setShowEnabled(true,true);
			break;
			
		case '2':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[2]->setShowEnabled(true,true);
			break;

		case '3':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[3]->setShowEnabled(true,true);			
			break;

		case '4':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[4]->setShowEnabled(true,true);
			break;

		case '5':
			for(int l=0;l<Objects.size();l++){
				Objects[l]->setShowEnabled(false,true);
			}
			Objects[5]->setShowEnabled(true,true);
			break;

		case '-':
			//	// decrease transparency level
			//	transparencyLevel = transparencyLevel - 0.1;
			//	if (transparencyLevel < 0.0) { transparencyLevel = 0.0; }

			//	// apply changes to DObject
			//	((cMesh*)(DObject->getChild(1)))->setTransparencyLevel(transparencyLevel);
			//	((cMesh*)(DObject->getChild(1)))->setUseTransparency(true);

			//	((cMesh*)(DObject->getChild(0)))->setTransparencyLevel(transparencyLevel);
			//	((cMesh*)(DObject->getChild(0)))->setUseTransparency(true);

			//	// if object is almost transparent, make it invisible
			//	if (transparencyLevel < 0.1){
			//		//((cMesh*)(DObject->getChild(1)))->setShowEnabled(false, true);
			//	}
			break;

		case '+':
			//	// increase transparency level
			//	transparencyLevel = transparencyLevel + 0.1;
			//	if (transparencyLevel > 1.0) { transparencyLevel = 1.0; }

			//	// apply changes to DObject
			//	((cMesh*)(DObject->getChild(1)))->setTransparencyLevel(transparencyLevel);

			//	// make object visible
			//	if (transparencyLevel >= 0.1)
			//	{
			//		((cMesh*)(DObject->getChild(1)))->setShowEnabled(true, true);
			//	}

			//	// disable transparency is transparency level is set to 1.0
			//	if (transparencyLevel == 1.0)
			//	{
			//		((cMesh*)(DObject->getChild(1)))->setUseTransparency(false);
			//	}
			break;
			
		case 'n':
			startGame();
			break;			
	}	
	
}

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------

void menuSelect(int value)
{

	switch (value)
	{
		// enable full screen display
	case OPTION_FULLSCREEN:
		glutFullScreen();
		CounterI.setPos(1530,990,0);	
		CounterC.setPos(1430,990,0);
		break;

		// reshape window to original size
	case OPTION_WINDOWDISPLAY:
		glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
		CounterI.setPos(500,590,0);	
		CounterC.setPos(400,590,0);
		break;
	}
}

//---------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	tool->stop();
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
	// render world
	camera->renderView(displayW, displayH);

	// Swap buffers
	glutSwapBuffers();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	// inform the GLUT window to call updateGraphics again (next frame)
	if (simulationRunning){	glutPostRedisplay();}
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
	// CH lab
	tool->setShowEnabled(true, true);





	// turn off friction (the original Chai3d friction implementation!)
	tool->m_proxyPointForceModel->m_useFriction = false;

	// main haptic simulation loop
	while(simulationRunning)
	{
		
		// update position and orientation of tool
		tool->updatePose();			
		// compute interaction forces
		if(useFriction)
			((ch_generic3dofPointer*)tool)->computeInteractionForcesD();
		else
			tool->computeInteractionForces();



		//Sound
		// check if the tool is touching an object
		cMesh* ContObject = (cMesh*)tool->m_proxyPointForceModel->m_contactPoint0->m_object;
		cGenericObject* GObject = tool->m_proxyPointForceModel->m_contactPoint0->m_object; 
		
		//TODO Change to for Loop if Hstream member keeps empty
		if(ContObject!=NULL){
			if (tool->isInContact(ContObject)){		
				LastID=GObject->getParent()->m_tag;					
				//BASS_ChannelPlay(Objects[LastID]->finalStream[0],FALSE==0);
				if(LastID>=0) ChangeSound(LastID);
				cout << GObject->getParent()->m_tag <<endl;
				
				
			}
		}
		else{
			if(LastID>=0) BASS_ChannelStop(Objects[LastID]->finalStream[0]);
		}
		button = tool->getUserSwitch(0);
		if(button==0)	newButPush=true;
		//std::cout<<"Dist"<<tool->getDeviceGlobalPos()<<std::endl ;

		/*	 if (button == 1)
		{
		nextState();
		}*/
		switch(curState)
		{
		case 0:
			selectSolution();
			break;
		case 1 :
			checkSolution();
			break;
		}


		
		// send forces to device
		tool->applyForces();
		tool->m_proxyPointForceModel->getForce();
		
		// compute global reference frames for each object
		world->computeGlobalPositions(true);
	}

	// exit haptics thread
	simulationFinished = true;
}

//---------------------------------------------------------------------------


// ch lab
// set static and dynamic friction coefficients for the object, propagating 
// them to the children
void ch_setFrictionCoefficients(cGenericObject* obj, const double& static_coeff, const double& dynamic_coeff){
	//obj->m_material.setDynamicFriction(dynamic_coeff);
	//obj->m_material.setStaticFriction(static_coeff);

	//for(unsigned int i = 0; i < obj->getNumChildren(); i++)
	//{
	//	ch_setFrictionCoefficients(obj->getChild(i), static_coeff, dynamic_coeff);		
	//}	
}


void redrawUI(){
	
	string Wrongs="INCORRECT: " + to_string(inc);
	string Rights="CORRECT: " + to_string(cor);
	CounterI.m_string=Wrongs;
	CounterC.m_string=Rights;
}

//---------------------------------------------------------------------------

void startGame(void){
	curState=0;
	checkBoxColl=false;
	tool->setWorkspaceRadius(1.0);
	for(int s=0;s<Objects.size();s++){
		Objects[s]->setShowBox(false);
		Objects[s]->setPos(0,0,0);

	}

	camera->set( cVector3d (3.0, 0.0, 0.0),    // camera position (eye)
		cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

	realModel=rand() % NumObj;
	cout<<"Model "<<realModel<<endl;
	for(int l=0;l<Objects.size();l++){
		Objects[l]->setShowEnabled(false,true);
	}
	Objects[realModel]->setShowEnabled(true,true);
	//Objects[realModel]->setHapticEnabled(true,true);

	Objects[realModel]->setTransparencyLevel(0);
}

//---------------------------------------------------------------------------

void selectSolution(void){
	if(button==1 && newButPush==1 && checkBoxColl==false){
		vector <int> list;
		//fill list
		for(int l=0;l<Objects.size();l++){
			Objects[l]->setShowEnabled(false,true);		
			list.push_back(l);
		}
		list.erase(list.begin()+realModel);
		//Choose Random elements from List
		int el;
		for(int i=0;i<2;i++){
			el=rand() % list.size();

			multChoice[i]=list.at(el);
			//std::cout <<"El:" << multChoice[i];
			list.erase(list.begin()+el);
		}
		multChoice[2]=realModel;
		//Shuffle Multiple Choice Elements
		int buf1;
		int swap;
		for(int w=0;w<2;w++)
		{	
			swap=rand()%2;
			buf1 = multChoice[2];
			multChoice[2]=multChoice[swap];
			multChoice[swap]=buf1;
		}

		Objects[multChoice[0]]->setPos(0.0, -2, 0.0);
		Objects[multChoice[1]]->setPos(0.0, 0.0, 0.0);
		Objects[multChoice[2]]->setPos(0.0, 2, 0.0);	

		for(int k=0;k<3;k++){
			Objects[multChoice[k]]->setShowEnabled(true,true);	
			Objects[multChoice[k]]->setUseTransparency(false,true);
			Objects[multChoice[k]]->setShowBox(true);
			Objects[multChoice[k]]->setBoxColor(cColorf(1,0,0,1));
			distToObj[k] = Objects[multChoice[k]]->getBoundaryMax().distance(Objects[multChoice[k]]->getBoundaryCenter());
		}
		camera->set( cVector3d (9.0, 0.0, 0.0),    // camera position (eye)
			cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
			cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector
		tool->setWorkspaceRadius(5.0);
		checkBoxColl=true;
		newButPush=false;
	}

	else if(checkBoxColl==true){

		//tool->m_proxyPointForceModel->m_contactPoint0->m_object->m_tag;

		for(int j=0;j<3;j++){
			//distToObj[j] = Objects[multChoice[j]]->getBoundaryMax().distance(Objects[multChoice[j]]->getBoundaryCenter());
			//if(tool->isInContact(Objects[multChoice[0]])){
			if(tool->getDeviceGlobalPos().distance(Objects[multChoice[j]]->getGlobalPos()) < distToObj[j] ){
				Objects[multChoice[j]]->setBoxColor(cColorf(0,1,0,1));					
				//Objects[multChoice[j]]->m_material.m_diffuse.set(0,0,1);
				if(newButPush==1 && button==1){			
					for(int s=0;s<3;s++){
						Objects[multChoice[s]]->setShowBox(false);
					}
					selectedModel=multChoice[j];
					newResult=true;
					curState=1;
				}

			}
			else Objects[multChoice[j]]->setBoxColor(cColorf(1,0,0,1));	
		}
	}
}

//---------------------------------------------------------------------------

void nextState(void){

// was soll das???
}

//---------------------------------------------------------------------------

void checkSolution(void){		
	if(newResult){
		if(selectedModel==realModel)
		{
			Objects[selectedModel]->setShowBox(true);
			Objects[selectedModel]->setBoxColor(cColorf(0,1,0,1));
			cout<<"RIGHT"<<endl;			
			cor++;
			redrawUI();

			cout <<"Press n for new Game"<<endl;			
		}
		else{
			cout<<"WRONG"<<endl;
			cout <<"Press n for new Game"<<endl;
			inc++;
			redrawUI();
			Objects[selectedModel]->setShowBox(true);
			Objects[realModel]->setShowBox(true);
			Objects[selectedModel]->setBoxColor(cColorf(1,0,0,1));
			Objects[realModel]->setBoxColor(cColorf(0,1,0,1));
		}
		newResult=false;
	}
	/*close();
	exit(0);*/
}

//---------------------------------------------------------------------------

void StartPlayback(cMesh* Obj){	


	//BASS_ChannelSetAttribute(Obj->stream[0], BASS_ATTRIB_FREQ, 500);
  
	//cout<< Obj->finalStream[0];
	//BASS_ChannelPlay(Obj->finalStream[0],FALSE);
	//cout << BASS_ChannelPlay(file_stream[0],FALSE);
	//BASS_ChannelPlay(Obj->getParent()->,FALSE);
	
	cout <<BASS_ErrorGetCode();
}

//---------------------------------------------------------------------------

void ChangeSound(int ID){
	// doku: http://www.bass.radio42.com/help/html/f00d6245-b20b-f37d-7982-8cc6549f4ae3.htm
	// http://www.bass.radio42.com/help/html/937729d8-fb7a-497d-a1d5-951f42873d58.htm
	
	//define maximum depth and maximum volume for material
	/*static const depth_max = 10;
	static const max_vol_material;*/
	int freq; //0-4
	cVector3d frequency;
	cVector3d force;

	force= tool->m_lastComputedGlobalForce;
	//cout <<"Force: " <<force;
	//// calculate output volume
	//if (depth < depth_max) {volume = (depth/depth_max) * max_vol_material;}
	//else {colume = max_vol_material;}
	// apply volume
	//BASS_ChannelSetAttribute(Obj->stream[0], BASS_ATTRIB_MUSIC_VOL_CHAN, volume); // range 0 - 1
	//BASS_ChannelSetAttribute(Obj->stream[freq], BBASS_ATTRIB_MUSIC_VOL_GLOBAL, volume);
	
	frequency = tool->m_deviceGlobalVel;
	/*if(frequency<=5) freq=0;
	else freq=1;*/
	freq=0;

	//BASS_ChannelSetAttribute(Objects[ID]->finalStream[freq], BASS_ATTRIB_FREQ, frequency);
	BASS_ChannelPlay(Objects[ID]->finalStream[freq],FALSE);
}

//---------------------------------------------------------------------------

//void load_soundfiles(std::string texture){
//	
//}

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
