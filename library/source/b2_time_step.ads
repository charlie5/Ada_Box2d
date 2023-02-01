#ifndef B2_TIME_STEP_H
#define B2_TIME_STEP_H

#include "b2_api.h"
#include "b2_math.h"

/// Profiling data. Times are in milliseconds.
struct B2_API b2Profile
{
	float step;
	float collide;
	float solve;
	float solveInit;
	float solveVelocity;
	float solvePosition;
	float broadphase;
	float solveTOI;
};

/// This is an internal structure.
struct B2_API b2TimeStep
{
	float dt;			// time step
	float inv_dt;		// inverse time step (0 if dt == 0).
	float dtRatio;	// dt * inv_dt0
	int32 velocityIterations;
	int32 positionIterations;
	bool warmStarting;
};

/// This is an internal structure.
struct B2_API b2Position
{
	b2Vec2 c;
	float a;
};

/// This is an internal structure.
struct B2_API b2Velocity
{
	b2Vec2 v;
	float w;
};

/// Solver Data
struct B2_API b2SolverData
{
	b2TimeStep step;
	b2Position* positions;
	b2Velocity* velocities;
};

#endif
