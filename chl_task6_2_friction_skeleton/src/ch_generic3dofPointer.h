#pragma once

#include "tools\CGeneric3dofPointer.h"
#include "ch_proxyPointForceAlgo.h"

class ch_generic3dofPointer : public cGeneric3dofPointer
{
public:
	ch_generic3dofPointer(cWorld* a_world):cGeneric3dofPointer(a_world) {};
	~ch_generic3dofPointer(void) {};

	//! Finger-proxy algorithm model to handle interactions with mesh objects.
    ch_proxyPointForceAlgo* ch_m_proxyPointForceModel;

	//! Compute interaction forces with environment.
	// overriden from class cGeneric3dofPointer
    void computeInteractionForcesD();
};
