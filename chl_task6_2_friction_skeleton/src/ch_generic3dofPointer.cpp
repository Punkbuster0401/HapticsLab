#include "ch_generic3dofPointer.h"


void ch_generic3dofPointer::computeInteractionForcesD()
{
    // temporary variable to store forces
    cVector3d force;
    force.zero();
	
	ch_m_proxyPointForceModel = (ch_proxyPointForceAlgo*)m_proxyPointForceModel;

    // compute forces in world coordinates for the point force algorithm    
	force.add(ch_m_proxyPointForceModel->computeForcesD(m_deviceGlobalPos, m_deviceGlobalVel));

    // copy result
    m_lastComputedGlobalForce.copyfrom(force);
}