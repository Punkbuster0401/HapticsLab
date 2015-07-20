#ifndef CH_PROXY_POINT_FORCE_ALGO
#define CH_PROXY_POINT_FORCE_ALGO

//#include "forces\CProxyPointForceAlgo.h"
#include "chai3d.h"

#define SMALL_VEL 0.001

class ch_proxyPointForceAlgo : public cProxyPointForceAlgo
{
public:
	ch_proxyPointForceAlgo(void) {};
	~ch_proxyPointForceAlgo(void) {};

	cVector3d computeForcesD(const cVector3d& a_toolPos, const cVector3d& a_toolVel);

	void computeNextBestProxyPosition(const cVector3d& a_goal, const cVector3d& a_toolVel);	

	void testFrictionAndMoveProxy(const cVector3d& a_goal, const cVector3d& a_proxy, cVector3d& a_normal, cGenericObject* a_parent, const cVector3d& a_toolVel);


protected:
	bool computeNextProxyPositionWithContraints00(const cVector3d& a_goalGlobalPos, const cVector3d& a_toolVel);

	bool computeNextProxyPositionWithContraints11(const cVector3d& a_goalGlobalPos, const cVector3d& a_toolVel);

	bool computeNextProxyPositionWithContraints22(const cVector3d& a_goalGlobalPos, const cVector3d& a_toolVel);
};

#endif