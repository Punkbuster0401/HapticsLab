#include "ch_proxyPointForceAlgo.h"
#include "scenegraph/CWorld.h"

// overriden from class cProxyPointForceAlgo	
cVector3d ch_proxyPointForceAlgo::computeForcesD(const cVector3d& a_toolPos, const cVector3d& a_toolVel)
{
    // update device position
    m_deviceGlobalPos = a_toolPos;

    // check if world has been defined; if so, compute forces
    if (m_world != NULL)
    {
        // compute next best position of proxy
        computeNextBestProxyPosition(m_deviceGlobalPos, a_toolVel);

        // update proxy to next best position
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;

        // compute force vector applied to device
        updateForce();

        // return result
        return (m_lastGlobalForce);
    }

    // if no world has been defined in which algorithm operates, there is no force
    else
    {
        return (cVector3d(0.0, 0.0, 0.0));
    }
}



void ch_proxyPointForceAlgo::computeNextBestProxyPosition(const cVector3d& a_goal, const cVector3d& a_toolVel)
{
    if (m_useDynamicProxy)
    {
        bool hit0, hit1, hit2;
        hit0 = computeNextProxyPositionWithContraints00(a_goal, a_toolVel);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit0) { return; }

        hit1 = computeNextProxyPositionWithContraints11(a_goal, a_toolVel);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
        if (!hit1) { return; }

        hit2 = computeNextProxyPositionWithContraints22(a_goal, a_toolVel);
        m_proxyGlobalPos = m_nextBestProxyGlobalPos;
    }
    else
    {
        // In order to keep the finger-proxy algorithm running fast, we only
        // compute collision with one constraint at the time. The next time
        // the algorithm is called, it searches for the second or
        // third obstacle.

        switch(m_algoCounter)
        {
            case 0:
                computeNextProxyPositionWithContraints00(a_goal, a_toolVel);
                break;

            case 1:
                computeNextProxyPositionWithContraints11(a_goal, a_toolVel);
                break;

            case 2:
                computeNextProxyPositionWithContraints22(a_goal, a_toolVel);
                break;
        }
    }
}


bool ch_proxyPointForceAlgo::computeNextProxyPositionWithContraints00(const cVector3d& a_goalGlobalPos, const cVector3d& a_toolVel)
{
    // We define the goal position of the proxy.
    cVector3d goalGlobalPos = a_goalGlobalPos;

    // To address numerical errors of the computer, we make sure to keep the proxy
    // slightly above any triangle and not directly on it. If we are using a radius of
    // zero, we need to define a default small value for epsilon
    m_epsilonInitialValue = cAbs(0.0001 * m_radius);
    if (m_epsilonInitialValue < m_epsilonBaseValue)
    {
        m_epsilonInitialValue = m_epsilonBaseValue;
    }

    // The epsilon value is dynamic (can be reduced). We set it to its initial
    // value if the proxy is not touching any triangle.
    if (m_numContacts == 0)
    {
        m_epsilon = m_epsilonInitialValue;
        m_slipping = true;
    }

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (!m_useDynamicProxy)
    {
        if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
        {
            m_nextBestProxyGlobalPos = m_proxyGlobalPos;
            m_algoCounter = 0;
            return (false);
        }
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position

    // compute the distance between the proxy and the goal positions
    double distanceProxyGoal = cDistance(m_proxyGlobalPos, goalGlobalPos);

    // A vector from the proxy to the goal
    cVector3d vProxyToGoal;
    cVector3d vProxyToGoalNormalized;
    bool proxyAndDeviceEqual;

    if (distanceProxyGoal > m_epsilon)
    {
		// proxy and goal are sufficiently distant from each other
        goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);
        vProxyToGoal.normalizer(vProxyToGoalNormalized);
        proxyAndDeviceEqual = false;
    }
    else
    {
		// proxy and goal are very close to each other
        vProxyToGoal.zero();
        vProxyToGoalNormalized.zero();
        proxyAndDeviceEqual = true;
    }

    // Test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos;
    if (m_useDynamicProxy)
    {
        targetPos = goalGlobalPos;
    }
    else
    {
        targetPos = goalGlobalPos +
                    cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);
    }

    // setup collision detector
	// m_radius is the radius of the proxy
    m_collisionSettings.m_collisionRadius = m_radius;

    // Search for a collision between the first segment (proxy-device)
    // and the environment.
    m_collisionSettings.m_adjustObjectMotion = m_useDynamicProxy;
    m_collisionRecorderConstraint0.clear();
	bool hit = m_world->computeCollisionDetection(m_proxyGlobalPos,
                                                  targetPos,
                                                  m_collisionRecorderConstraint0,
                                                  m_collisionSettings);


    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint0.m_nearestCollision.m_squareDistance);
        if (m_useDynamicProxy)
        {
            // retrieve new position of proxy
            cVector3d posLocal = m_collisionRecorderConstraint0.m_nearestCollision.m_adjustedSegmentAPoint;
            cGenericObject* obj = m_collisionRecorderConstraint0.m_nearestCollision.m_object;
            cVector3d posGlobal = cAdd(obj->getGlobalPos(), cMul( obj->getGlobalRot(), posLocal ));
            m_proxyGlobalPos = posGlobal;

            distanceProxyGoal = cDistance(m_proxyGlobalPos, goalGlobalPos);
            goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);
            vProxyToGoal.normalizer(vProxyToGoalNormalized);
        }


        if (collisionDistance > (distanceProxyGoal + CHAI_SMALL))
        {
			// just to make sure that the collision point lies on the proxy-goal segment and not outside of it
            hit = false;
        }


        if (hit)
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, then we move the proxy to its goal, and we're done
    if (!hit)
    {
        m_numContacts = 0;
        m_algoCounter = 0;
        m_slipping = true;
        m_nextBestProxyGlobalPos = goalGlobalPos;
        return (false);
    }

    // a first collision has occurred
    m_algoCounter = 1;

    //-----------------------------------------------------------------------
    // FIRST COLLISION OCCURES:
    //-----------------------------------------------------------------------

    // We want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // So we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);

    // Now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // If only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / cAbs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // We compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // A vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_contactPoint0->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_contactPoint0->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;


    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numContacts = 1;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//---------------------------------------------------------------------------

bool ch_proxyPointForceAlgo::computeNextProxyPositionWithContraints11(const cVector3d& a_goalGlobalPos, const cVector3d& a_toolVel)
{
    // The proxy is now constrained on a plane; we now calculate the nearest
    // point to the original goal (device position) on this plane; this point
    // is computed by projecting the ideal goal onto the plane defined by the
    // intersected triangle
    cVector3d goalGlobalPos = cProjectPointOnPlane(a_goalGlobalPos,
              m_proxyGlobalPos,
              m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal);

    // A vector from the proxy to the goal (ch lab -projection of the original goal onto the collided triangle)
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

	// ch lab - If the distance between the proxy and the projected goal position is
    // very small then we can be considered done.
    // original comment - If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numContacts = 1;
        return (false);
    }

	// ch lab - compute the normalized form of the vector going from the
    // current proxy position to the projected goal position
    // original comment - compute the normalized form of the vector going from the
    // current proxy position to the desired goal position
    cVector3d vProxyToGoalNormalized;
    vProxyToGoal.normalizer(vProxyToGoalNormalized);

    // Test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos = goalGlobalPos +
                          cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for collision
    m_collisionSettings.m_adjustObjectMotion = false;
    m_collisionRecorderConstraint1.clear();
    bool hit = m_world->computeCollisionDetection( m_proxyGlobalPos,
                                                   targetPos,
                                                   m_collisionRecorderConstraint1,
                                                   m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint1.m_nearestCollision.m_squareDistance);
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + CHAI_SMALL))
        {
            hit = false;
        }
        else
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, we move the proxy to its goal, unless
    // friction prevents us from doing so.
    if (!hit)
    {
		m_numContacts = 1;
        testFrictionAndMoveProxy(goalGlobalPos,
                                 m_proxyGlobalPos,
                                 m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal,
                                 m_collisionRecorderConstraint0.m_nearestCollision.m_object, a_toolVel);

        
        m_algoCounter = 0;

        return (false);
    }

    // a second collision has occurred
    m_algoCounter = 2;

    //-----------------------------------------------------------------------
    // SECOND COLLISION OCCURES:
    //-----------------------------------------------------------------------
    // We want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // So we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal);

    // Now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // If only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / cAbs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // We compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // A vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_contactPoint1->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_contactPoint1->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(goalGlobalPos, nextProxyPos))
    {
        m_numContacts = 2;
        m_algoCounter = 0;
        return (true);
    }

    return (true);
}

//---------------------------------------------------------------------------

bool ch_proxyPointForceAlgo::computeNextProxyPositionWithContraints22(const cVector3d& a_goalGlobalPos, const cVector3d& a_toolVel)
{
    // The proxy is now constrained by two triangles and can only move along
    // a virtual line; we now calculate the nearest point to the original
    // goal (device position) along this line by projecting the ideal
    // goal onto the line.
    //
    // The line is expressed by the cross product of both surface normals,
    // which have both been oriented to point away from the device
    cVector3d line;
    m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal.crossr(m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal, line);

    // check result.
    if (line.equals(cVector3d(0,0,0)))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numContacts = 2;
        return (false);
    }

    line.normalize();

    // Compute the projection of the device position (goal) onto the line; this
    // gives us the new goal position.
    cVector3d goalGlobalPos = cProjectPointOnLine(a_goalGlobalPos, m_proxyGlobalPos, line);

    // A vector from the proxy to the goal
    cVector3d vProxyToGoal;
    goalGlobalPos.subr(m_proxyGlobalPos, vProxyToGoal);

    // If the distance between the proxy and the goal position (device) is
    // very small then we can be considered done.
    if (goalAchieved(m_proxyGlobalPos, goalGlobalPos))
    {
        m_nextBestProxyGlobalPos = m_proxyGlobalPos;
        m_algoCounter = 0;
        m_numContacts = 2;
        return (false);
    }

    // compute the normalized form of the vector going from the
    // current proxy position to the desired goal position
    cVector3d vProxyToGoalNormalized;
    vProxyToGoal.normalizer(vProxyToGoalNormalized);

    // Test whether the path from the proxy to the goal is obstructed.
    // For this we create a segment that goes from the proxy position to
    // the goal position plus a little extra to take into account the
    // physical radius of the proxy.
    cVector3d targetPos = goalGlobalPos +
                          cMul(m_epsilonCollisionDetection, vProxyToGoalNormalized);

    // setup collision detector
    m_collisionSettings.m_collisionRadius = m_radius;

    // search for collision
    m_collisionSettings.m_adjustObjectMotion = false;
    m_collisionRecorderConstraint2.clear();
    bool hit = m_world->computeCollisionDetection( m_proxyGlobalPos,
                                                   targetPos,
                                                   m_collisionRecorderConstraint2,
                                                   m_collisionSettings);

    // check if collision occurred between proxy and goal positions.
    double collisionDistance;
    if (hit)
    {
        collisionDistance = sqrt(m_collisionRecorderConstraint2.m_nearestCollision.m_squareDistance);
        if (collisionDistance > (cDistance(m_proxyGlobalPos, goalGlobalPos) + CHAI_SMALL))
        {
            hit = false;
        }
        else
        {
            // a collision has occurred and we check if the distance from the
            // proxy to the collision is smaller than epsilon. If yes, then
            // we reduce the epsilon term in order to avoid possible "pop through"
            // effect if we suddenly push the proxy "up" again.
            if (collisionDistance < m_epsilon)
            {
                m_epsilon = collisionDistance;
                if (m_epsilon < m_epsilonMinimalValue)
                {
                    m_epsilon = m_epsilonMinimalValue;
                }
            }
        }
    }

    // If no collision occurs, we move the proxy to its goal, unless
    // friction prevents us from doing so
    if (!hit)
    {
		cVector3d normal = cMul(0.5,cAdd(m_collisionRecorderConstraint0.m_nearestCollision.m_globalNormal,
										 m_collisionRecorderConstraint1.m_nearestCollision.m_globalNormal));
						 
        testFrictionAndMoveProxy(goalGlobalPos, 
								 m_proxyGlobalPos,
								 normal,
								m_collisionRecorderConstraint1.m_nearestCollision.m_triangle->getParent(), a_toolVel);
        m_numContacts = 2;
        m_algoCounter = 0;

        return (false);
    }

    //-----------------------------------------------------------------------
    // THIRD COLLISION OCCURES:
    //-----------------------------------------------------------------------
   // We want the center of the proxy to move as far toward the triangle as it can,
    // but we want it to stop when the _sphere_ representing the proxy hits the
    // triangle.  We want to compute how far the proxy center will have to
    // be pushed _away_ from the collision point - along the vector from the proxy
    // to the goal - to keep a distance m_radius between the proxy center and the
    // triangle.
    //
    // So we compute the cosine of the angle between the normal and proxy-goal vector...
    double cosAngle = vProxyToGoalNormalized.dot(m_collisionRecorderConstraint2.m_nearestCollision.m_globalNormal);

    // Now we compute how far away from the collision point - _backwards_
    // along vProxyGoal - we have to put the proxy to keep it from penetrating
    // the triangle.
    //
    // If only ASCII art were a little more expressive...
    double distanceTriangleProxy = m_epsilon / cAbs(cosAngle);
    if (distanceTriangleProxy > collisionDistance) { distanceTriangleProxy = cMax(collisionDistance, m_epsilon); }

    // We compute the projection of the vector between the proxy and the collision
    // point onto the normal of the triangle.  This is the direction in which
    // we'll move the _goal_ to "push it away" from the triangle (to account for
    // the radius of the proxy).

    // A vector from the most recent collision point to the proxy
    cVector3d vCollisionToProxy;
    m_proxyGlobalPos.subr(m_contactPoint2->m_globalPos, vCollisionToProxy);

    // Move the proxy to the collision point, minus the distance along the
    // movement vector that we computed above.
    //
    // Note that we're adjusting the 'proxy' variable, which is just a local
    // copy of the proxy position.  We still might decide not to move the
    // 'real' proxy due to friction.
    cVector3d vColNextGoal;
    vProxyToGoalNormalized.mulr(-distanceTriangleProxy, vColNextGoal);
    cVector3d nextProxyPos;
    m_contactPoint2->m_globalPos.addr(vColNextGoal, nextProxyPos);

    // we can now set the next position of the proxy
    m_nextBestProxyGlobalPos = nextProxyPos;
    m_algoCounter = 0;
    m_numContacts = 3;

    // TODO: There actually should be a third friction test to see if we
    // can make it to our new goal position, but this is generally such a
    // small movement in one iteration that it's irrelevant...

    return (true);
}


// ch lab ---to be filled in by the students---
// Overriden from class cProxyPointForceAlgo	
// Here, a_goal has been computed by collision detection above as the constrained goal towards which
// the proxy should be moved. But before moving it freely to that location, let us check if friction allows
// us to do so. we answer the question: to what extent along the proxy-goal segment can we forward the proxy?
void ch_proxyPointForceAlgo::testFrictionAndMoveProxy(const cVector3d& a_goal, const cVector3d& a_proxy, cVector3d& a_normal, cGenericObject* a_parent, const cVector3d& a_toolVel)
{	

	// In this method, our aim is to calculate the the next best position for the proxy (m_nextBestProxyGlobalPos), considering friction.
	// Among other things, we are given the goal position (a_goal), the current proxy position (a_proxy), the parent object (a_parent),
	// the tool velocity (a_toolVel)


	// We will use this variable to determine if we should use the static or the dynamic
	// friction coeff to compute current frictional force.
	static double last_device_vel;

	
	
	// friction coefficients assigned to object surface	 
    cMesh* parent_mesh = dynamic_cast<cMesh*>(a_parent);

    // Right now we can only work with cMesh's
    if (parent_mesh == NULL)
    {
        m_nextBestProxyGlobalPos = a_goal;
        return;
    }

	// read friction coefficients here
	// -----------------------your code here------------------------------------
	double dynamic_coeff = parent_mesh->m_material.getDynamicFriction();		
     double static_coeff = parent_mesh->m_material.getStaticFriction();



	// find the penetration depth of the actual device position from the nominal object surface
	double pen_depth = cDistance(a_goal, m_deviceGlobalPos);

	// shall we use the static or the dynamic friction coeff. for the cone radius calculation?
	double cone_radius;
	if(last_device_vel < SMALL_VEL)		
	{
		//// the radius of the friction cone
		//-----------------------your code here------------------------------------
		cone_radius = static_coeff*pen_depth;	
	}
	else
	{
		//// the radius of the friction cone
		//-----------------------your code here------------------------------------
		 cone_radius = dynamic_coeff*pen_depth;		
	}

	// vector from the current proxy position to the new sub-goal
	cVector3d a_proxyGoal, a_proxyGoalNormalized;
	a_goal.subr(a_proxy, a_proxyGoal);
		
	// normalize the proxy goal vector
	a_proxyGoal.normalizer(a_proxyGoalNormalized);
	
	double a_proxyGoalLength = a_proxyGoal.length();

	if(a_proxyGoalLength < cone_radius)		
		// The proxy is inside the friction cone already.
		return;
	else
	{		
		// The proxy is outside the friction cone, we should advance it towards the cone circumference,
		// along the vector from the current proxy position to the current goal position

		//-----------------------your code here------------------------------------
		// calculate a value for m_nextBestProxyGlobalPos
		m_nextBestProxyGlobalPos=a_proxy+(a_proxyGoalLength-cone_radius)*a_proxyGoalNormalized;
	}

	// record last velocity in order to decide if static or dynamic friction is to be applied during the
	// next iteration
	last_device_vel = a_toolVel.length();
}



