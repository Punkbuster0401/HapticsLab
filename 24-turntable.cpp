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

    \author:    <http://www.chai3d.org>
    \author:    Federico Barbagli
    \author:    Chris Sewell
    \author:    Francois Conti
    \version    #CHAI_VERSION#
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bass.h"
//---------------------------------------------------------------------------
#include "chai3d.h"
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

// a haptic device handler
cHapticDeviceHandler* handler;

// a virtual tool representing the haptic device in the scene
cGeneric3dofPointer* tool;

// radius of the tool proxy
double proxyRadius;

// a virtual turntble object
cMesh* turntable;

// a virtual vinyl
cMesh* record;

// rotational velocity of the record
double rotVel;

// rotational position of the record
double rotPos;

// status of the main simulation haptics loop
bool simulationRunning = false;

// simulation clock
cPrecisionClock simClock;

// BASS library
const int MAX_VAL_SLIDERS_P = 100000;
const int MAX_VAL_SLIDERS_I = 10000000;
const int FREQ = 1000;
const int MAX_TIME = 50;
const double chartClock = 50;
const double MESH_SCALE_SIZE = 0.35;

// Global variables for the audio stream
HSTREAM stream;
BASS_CHANNELINFO infoBass;
QWORD stream_length;
char *data;
int record_direction = 1;
unsigned int pos = 0;


/////////////////////////////////////////////////////////////////////////////////////
//								bla bla bla										   //
/////////////////////////////////////////////////////////////////////////////////////
// size of sound data
const int NBR_TEXTURES = 1;
const int NBR_VELOCITIES = 5;
int ctr_chg_sound = 0;
HSTREAM stream_mat[NBR_TEXTURES][NBR_VELOCITIES];
BASS_CHANNELINFO infoBass_mat[NBR_TEXTURES][NBR_VELOCITIES];
QWORD stream_length_mat[NBR_TEXTURES][NBR_VELOCITIES];
char *data_mat[NBR_TEXTURES][NBR_VELOCITIES];
int soundf = 0;



// root resource path
string resourceRoot;

// has exited haptics simulation thread
bool simulationFinished = false;

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

// build record player
void createRecord(cMesh *a_mesh, double radius);

// Write the requested data from the loaded buffer to the sound card
DWORD CALLBACK MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user);


//===========================================================================
/*
    DEMO:    turntable.cpp

    This example demonstrates the use of friction, animation, and
    sound effects.  Enable haptcs, and you can spin the record
    back and forth and at different speeds.
*/
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    printf ("\n");
    printf ("-----------------------------------\n");
    printf ("CHAI 3D\n");
    printf ("Demo: 24-turntable\n");
    printf ("Copyright 2003-2009\n");
    printf ("-----------------------------------\n");
    printf ("\n\n");
    printf ("Instructions:\n\n");
    printf ("- Use haptic device to spin record \n");
    printf ("\n\n");
    printf ("Keyboard Options:\n\n");
    printf ("[x] - Exit application\n");
    printf ("\n\n");

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


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
    camera->set( cVector3d (2.5, 0.1, 1.2),    // camera position (eye)
                 cVector3d (0.0, 0.1, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the "up" vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // create a light source and attach it to the camera
    light = new cLight(world);
    light->setEnabled(true);                   // enable light source
    light->setPos(cVector3d( 0.5, 0.0, 1.0));  // position the light source
    light->setDir(cVector3d(-0.5, 0.0, -1.0));  // define the direction of the light beam

    // adjust light source components
    light->m_ambient.set(0.9, 0.9, 0.9);
    light->m_diffuse.set(0.9, 0.9, 0.9);
    light->m_specular.set(1.0, 1.0, 1.0);


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


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device
    cGenericHapticDevice* hapticDevice;
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo info;
    if (hapticDevice)
    {
        info = hapticDevice->getSpecifications();
    }

    // create a 3D tool and add it to the world
    tool = new cGeneric3dofPointer(world);
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

    // set the physical readius of the proxy.
    proxyRadius = 0.0;
    tool->m_proxyPointForceModel->setProxyRadius(proxyRadius);

    tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->m_proxyPointForceModel->m_useDynamicProxy = true;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    double stiffnessMax = info.m_maxForceStiffness / workspaceScaleFactor;


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a virtual mesh
    turntable = new cMesh(world);

    // add object to world
    world->addChild(turntable);

    // set the position of the object at the center of the world
    turntable->setPos(0.0, 0.0, 0.0);
    turntable->rotate(cVector3d(0,0,1), cDegToRad(90));

    // load an object file
    fileload = turntable->loadFromFile(RESOURCE_PATH("resources/models/turntable/turntable.obj"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = turntable->loadFromFile("../../../bin/resources/models/turntable/turntable.obj");
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // compute a boundary box
    turntable->computeBoundaryBox(true);

    // get dimensions of object
    double size = cSub(turntable->getBoundaryMax(), turntable->getBoundaryMin()).length();

    // resize object to screen
    if (size > 0)
    {
        turntable->scale( 2.0 * tool->getWorkspaceRadius() / size);
    }

    // compute collision detection algorithm
    turntable->createAABBCollisionDetector(1.01 * proxyRadius, true, false);

    // define a default stiffness for the object
    turntable->setStiffness(stiffnessMax, true);

    // Initialize sound device and create audio stream
    BASS_Init(1,44100,0,0,NULL);

	//// Load the data from the specified file
    HSTREAM file_stream = 1;
    file_stream = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/classic.mp3"),0,0,BASS_STREAM_DECODE);

	HSTREAM file_stream_mat[NBR_TEXTURES][NBR_VELOCITIES] = {1,1,1,1,1};
	file_stream_mat[0][0] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/horse.wav"),0,0,BASS_STREAM_DECODE);
	file_stream_mat[0][1] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/lion.wav"),0,0,BASS_STREAM_DECODE);
	file_stream_mat[0][2] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/bird.wav"),0,0,BASS_STREAM_DECODE);
	file_stream_mat[0][3] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/cow.wav"),0,0,BASS_STREAM_DECODE);
	file_stream_mat[0][4] = BASS_StreamCreateFile(FALSE,RESOURCE_PATH("resources/sounds/s1/duck.wav"),0,0,BASS_STREAM_DECODE);

	if (file_stream_mat[0][0] == 0)
	{
		#if defined(_msvc)
		file_stream_mat[0][0] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/horse.wav"),0,0,bass_stream_decode);
		file_stream_mat[0][1] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/lion.wav"),0,0,bass_stream_decode);
		file_stream_mat[0][2] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/bird.wav"),0,0,bass_stream_decode);
		file_stream_mat[0][3] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/cow.wav"),0,0,bass_stream_decode);
		file_stream_mat[0][4] =  bass_streamcreatefile(false,"../../../bin/resources/sounds/s1/duck.wav"),0,0,bass_stream_decode);
		//file_stream = bass_streamcreatefile(false,"../../../bin/resources/sounds/classic.mp3",0,0,bass_stream_decode);
		#endif
	}
	if (!fileload)
	{
        printf("error - mp3 audio file failed to load correctly.\n");
        close();
        return (-1);
    }


	/////////////////////////////////////////////////////////////////////////////////////
	//								bla bla bla										   //
	/////////////////////////////////////////////////////////////////////////////////////
	for( int text = 0; text < NBR_TEXTURES; text = text + 1 ){	
		for( int veloc = 0; veloc < NBR_VELOCITIES; veloc = veloc + 1 ){
			stream_length_mat[text][veloc] = BASS_ChannelGetLength(file_stream_mat[text][veloc], 0);
			BASS_ChannelGetInfo(file_stream_mat[text][veloc], &infoBass_mat[text][veloc]);
			data_mat[text][veloc] = new char[(unsigned int)stream_length_mat[text][veloc]];
			BASS_ChannelGetData(file_stream_mat[text][veloc], data_mat[text][veloc], (unsigned int)stream_length_mat[text][veloc]);
			
			/*data = data_mat[text][veloc];
			stream_length = stream_length_mat[text][veloc];*/

			stream_mat[text][veloc] = BASS_StreamCreate(infoBass_mat[text][veloc].freq, infoBass_mat[text][veloc].chans, 0, &MyStreamWriter, 0);
			std::cout << "stream: " << stream_mat[text][veloc] << std::endl;
			std::cout << "data_mat:  " << sizeof(*data_mat[text][veloc]) <<"|| stream_length:  " << stream_length_mat[text][veloc] << std::endl;
		}
	}

 //   // Get the length and header info from the loaded file
 //   //stream_length = BASS_StreamGetLength(file_stream);
 //   stream_length = BASS_ChannelGetLength(file_stream, 0);

 //   BASS_ChannelGetInfo(file_stream, &infoBass);

 //   // Get the audio samples from the loaded file
 //   data = new char[(unsigned int)stream_length];
 //   BASS_ChannelGetData(file_stream, data, (unsigned int)stream_length);

 //   // Set playing to begin at the beginning of the loaded data
 //   stream = BASS_StreamCreate(infoBass.freq, infoBass.chans, 0, &MyStreamWriter, 0);
	//stream = stream_mat[0][0];

    // create new record
    record = new cMesh(world);
    createRecord(record, 0.48);
    cTexture2D* recordImage = new cTexture2D();

    fileload = recordImage->loadFromFile(RESOURCE_PATH("resources/images/record.bmp"));
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = recordImage->loadFromFile("../../../bin/resources/images/record.bmp");
        #endif
    }
    if (!fileload)
    {
        printf("Error - Texture image failed to load correctly.\n");
        close();
        return (-1);
    }

    record->setTexture(recordImage);
    record->setUseTexture(true, true);
    record->createAABBCollisionDetector(proxyRadius, true, false);

    // add object to world and translate
    world->addChild(record);
    record->translate(-0.0, 0, 0.02);

    // set stiffness properties of record
    record->setStiffness(stiffnessMax, true);

    // set static and dynamic friction
    double staticFriction = (double)100 / 100.0;
    double dynamicFriction = (double)100 / 100.0;
    record->setFriction(staticFriction, dynamicFriction, true);
    

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;

    // initialize the OpenGL GLUT window
    glutInit(&argc, argv);
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
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

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    displayW = w;
    displayH = h;
    glViewport(0, 0, displayW, displayH);
}

//---------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }
	else if (key == '0'){
		pos = 0;
		soundf = 0;
		std::cout << "-------------soundf_key: " << soundf << std::endl; // prints Output sentence on screen
	}
	else if (key == '1'){
		pos = 0;
		soundf = 1;
		std::cout << "-------------soundf_key: " << soundf << std::endl; // prints Output sentence on screen
	}
	else if (key == '2'){
		pos = 0;
		std::cout << "-------------soundf_key: " << soundf << std::endl; // prints Output sentence on screen
		soundf = 2;
	}
	else if (key == '3'){
		pos = 0;
		std::cout << "-------------soundf_key: " << soundf << std::endl; // prints Output sentence on screen
		soundf = 3;
	}
	else if (key == '4'){
		pos = 0;
		std::cout << "-------------soundf_key: " << soundf << std::endl; // prints Output sentence on screen
		soundf = 4;
	}
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
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
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // reset clock
    simClock.reset();

    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updatePose();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to device
        tool->applyForces();

        // stop the simulation clock
        simClock.stop();

        // read the time increment in seconds
        double timeInterval =simClock.getCurrentTimeSeconds();

        // restart the simulation clock
        simClock.reset();
        simClock.start();

        // init
        cVector3d torque(0,0,0);

        // figure out if we're touching the record
        if (tool->isInContact(record))
        {
            // get position of cursor in global coordinates
            cVector3d toolPos = tool->m_deviceGlobalPos;

            // get position of object in global coordinates
            cVector3d objectPos = record->getGlobalPos();

            // compute a vector from the center of mass of the object (point of rotation) to the tool
            cVector3d vObjectCMToTool = cSub(toolPos, objectPos);

            if (vObjectCMToTool.length() > 0)
            {
                // get the last force applied to the cursor in global coordinates
                // we negate the result to obtain the opposite force that is applied on the
                // object
                cVector3d toolForce = cNegate(tool->m_lastComputedGlobalForce);

                // compute effective force to take into account the fact the object
                // can only rotate arround a its center mass and not translate
                cVector3d effectiveForce = toolForce - cProject(toolForce, vObjectCMToTool);

                // compute the resulting torque
                torque = cMul(vObjectCMToTool.length(), cCross( cNormalize(vObjectCMToTool), effectiveForce));
            }
        }

        // update rotational acceleration
        const double OBJECT_INERTIA = 0.02;
        double rotAcc = (1.0 / OBJECT_INERTIA) * torque.z;

        // update rotattional velocity
        rotVel = rotVel + timeInterval * rotAcc;

        // set a threshold on the rotational velocity term
        const double ROT_VEL_MAX = 20.0;

        if (rotVel > ROT_VEL_MAX)
        {
            rotVel = ROT_VEL_MAX;
        }
        else if (rotVel < -ROT_VEL_MAX)
        {
            rotVel = -ROT_VEL_MAX;
        }

        // compute the next rotation of the torus
        rotPos = rotPos + timeInterval * rotVel;

        // update position of record
        cMatrix3d rot;
        rot.identity();
        rot.rotate(cVector3d(0,0,1), rotPos);
        record->setRot(rot);

        // update audio
        // set audio direction and frequency based on rotational velocity
        if ((fabs(rotVel)) > 0.0)
        {
            if (rotVel < 0.0) record_direction = 1;
            else record_direction = -1;

		/////////////////////////////////////////////////////////////////////////////////////
		//								bla bla bla										   //
		/////////////////////////////////////////////////////////////////////////////////////
			
			std::cout << "soundf: " << soundf << std::endl; // prints Output sentence on screen

			std::cout << "rotVel: " << rotVel << std::endl; // prints Output sentence on screen
			ctr_chg_sound++;
			if (ctr_chg_sound == 100){
				ctr_chg_sound = 0;
				if (soundf == 4){

					BASS_ChannelStop(stream_mat[0][0]);
					BASS_ChannelStop(stream_mat[0][1]);
					BASS_ChannelStop(stream_mat[0][2]);
					BASS_ChannelStop(stream_mat[0][3]);

					data = data_mat[0][4];
					stream_length = stream_length_mat[0][4];
					// maybe all sound files must have same length (len in callback)

					BASS_ChannelSetAttribute(stream_mat[0][4], BASS_ATTRIB_FREQ, (int)(infoBass_mat[0][4].freq));
					BASS_ChannelPlay(stream_mat[0][4],FALSE);
					std::cout << "sound: DUCK" << std::endl; // prints Output sentence on screen

				}
				else if (soundf == 3){
					BASS_ChannelStop(stream_mat[0][0]);
					BASS_ChannelStop(stream_mat[0][1]);
					BASS_ChannelStop(stream_mat[0][2]);
					BASS_ChannelStop(stream_mat[0][4]);

					data = data_mat[0][3];
					stream_length = stream_length_mat[0][3];

					BASS_ChannelSetAttribute(stream_mat[0][3], BASS_ATTRIB_FREQ, (int)(infoBass_mat[0][3].freq));
					BASS_ChannelPlay(stream_mat[0][3],FALSE);
					std::cout << "sound: COW" << std::endl; // prints Output sentence on screen
				}
				else if (soundf == 2){
					BASS_ChannelStop(stream_mat[0][0]);
					BASS_ChannelStop(stream_mat[0][1]);
					BASS_ChannelStop(stream_mat[0][3]);
					BASS_ChannelStop(stream_mat[0][4]);

					data = data_mat[0][2];
					stream_length = stream_length_mat[0][2];
			
					BASS_ChannelSetAttribute(stream_mat[0][2], BASS_ATTRIB_FREQ, (int)(infoBass_mat[0][2].freq));
					BASS_ChannelPlay(stream_mat[0][2],FALSE);
					std::cout << "sound: BIRD" << std::endl; // prints Output sentence on screen
				}
				else if (soundf == 1){
					BASS_ChannelStop(stream_mat[0][0]);
					BASS_ChannelStop(stream_mat[0][2]);
					BASS_ChannelStop(stream_mat[0][3]);
					BASS_ChannelStop(stream_mat[0][4]);

					data = data_mat[0][1];
					stream_length = stream_length_mat[0][1];
					
					BASS_ChannelSetAttribute(stream_mat[0][1], BASS_ATTRIB_FREQ, (int)(infoBass_mat[0][1].freq));
					BASS_ChannelPlay(stream_mat[0][1],FALSE);
					std::cout << "sound: LION" << std::endl; // prints Output sentence on screen
				}
				else{
					BASS_ChannelStop(stream_mat[0][1]);
					BASS_ChannelStop(stream_mat[0][2]);
					BASS_ChannelStop(stream_mat[0][3]);
					BASS_ChannelStop(stream_mat[0][4]);

					data = data_mat[0][0];
					stream_length = stream_length_mat[0][0];
					
					BASS_ChannelSetAttribute(stream_mat[0][0], BASS_ATTRIB_FREQ, (int)(infoBass_mat[0][0].freq));
					BASS_ChannelPlay(stream_mat[0][0],FALSE);
					std::cout << "sound: HORSE" << std::endl; // prints Output sentence on screen
				}
			}

            /*if (!(BASS_ChannelPlay(stream_mat[0][0],FALSE)))
            {

            }*/
        }
        else
        {
            BASS_ChannelStop(stream_mat[0][0]);
			BASS_ChannelStop(stream_mat[0][1]);
			BASS_ChannelStop(stream_mat[0][2]);
			BASS_ChannelStop(stream_mat[0][3]);
			BASS_ChannelStop(stream_mat[0][4]);
        }


          /*  BASS_ChannelSetAttribute(stream, BASS_ATTRIB_FREQ, (int)(infoBass.freq*(0.05+fabs(rotVel))/6.5));

            if (!(BASS_ChannelPlay(stream,FALSE)))
            {

            }
        }
        else
        {
            BASS_ChannelStop(stream);
        }*/
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

DWORD CALLBACK MyStreamWriter(HSTREAM handle, void *buf, DWORD len, void *user)
{
	std::cout << "====len====" << len << "stream_length: " << stream_length << std::endl; // prints Output sentence on screen

	

	int tb_mulop = pos;
    // Cast the buffer to a character array
	char *d=(char*)buf;

    // Loop the file when it reaches the beginning or end
    if ((pos >= stream_length) && (record_direction == 1))
		    pos = 0;
	if ((pos <= 0) && (record_direction == -1))
		    pos = (unsigned int)stream_length;

	if (pos == 0){
		std::cout << "pos ist gay!!!" << std::endl;
	}

	// If record is spinning in positive direction, write requested
	// amount of data from current position forwards
	if (record_direction == 1)
	{
		int up = len + pos;
		if (up > stream_length)
			up = (unsigned int)stream_length;

    for (int i=pos; i<up; i+=1)
			d[(i-pos)] = data[i];

		int amt = (up-pos);
		pos += amt;
		return amt;
	 }

    // If record is spinning in negative direction, write requested
	// amount of data from current position backwards
	if (record_direction == -1)
	{
		int up = pos - len;

		if (up < 0)
			up = 0;

	    int cnt = 0;
        for (int i=pos; i>up; i-=1)
                d[cnt++] = data[i];

		int amt = cnt;
		pos -= amt;

		return amt;
	 }

	 return 0;
}

//---------------------------------------------------------------------------

void createRecord(cMesh *a_mesh, double radius)
{
    const double HEIGHT = 0.04;
    double vectorY1, originY, vectorX1, originX;
    vectorY1 = 0; originY = 0;
    vectorX1 = 0; originX = 0;
    cVector3d vector1b, vector2b, originb;
    cVector3d vector1t, vector2t, origint;
    originb.set(originX, originY, 0);
    origint.set(originX, originY, HEIGHT);

    vector2b.set(originX, originY, 0);
    vector2t.set(originX, originY, HEIGHT);

    int num = 28;
    double divisore = num / (2*CHAI_PI);

    for(int i=0; i<=num; i++)
    {
        double angle=(float)(((double)i)/divisore);
        vector1b.set(originX+(radius*(float)sin((double)angle)), originY+(radius*(float)cos((double)angle)), 0);
        vector1t.set(originX+(radius*(float)sin((double)angle)), originY+(radius*(float)cos((double)angle)), HEIGHT);
        a_mesh->newTriangle(origint, vector1t, vector2t);
        a_mesh->newTriangle(originb, vector2b, vector1b);

        a_mesh->newTriangle(vector1b, vector2b, vector2t);
        a_mesh->newTriangle(vector1b, vector2t, vector1t);

        vector2b = vector1b;
        vector2t = vector1t;
    }

    for(unsigned int n=0; n<a_mesh->getNumVertices(); n++)
    {
        cVertex* curVertex = a_mesh->getVertex(n);
        curVertex->setTexCoord(
            (curVertex->getPos().x + radius) / (2.0 * radius),
            (curVertex->getPos().y + radius) / (2.0 * radius)
        );
        curVertex->setNormal(0,0,1);
    }

    // compute normals
    a_mesh->computeAllNormals(true);

    // compute boudary box
    a_mesh->computeBoundaryBox(true);
}

