// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "benchmarks.h"
#include "overflow_color.h"
#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/constants.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

// This is a simple example of building and running a simulation
// using Box3D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box3D is meant to be used
// with your rendering engine in your game engine.
int HelloWorld( void )
{
	// Construct a world object, which will hold and simulate the rigid bodies.
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	// Define the ground body.
	b3BodyDef groundBodyDef = b3DefaultBodyDef();
	groundBodyDef.position = (b3Pos){ 0.0f, -10.0f, 0.0f };

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	b3BodyId groundId = b3CreateBody( worldId, &groundBodyDef );
	ENSURE( b3Body_IsValid( groundId ) );

	// Define the ground box shape. The extents are the half-widths of the box.
	b3BoxHull groundBox = b3MakeBoxHull( 50.0f, 10.0f, 50.0f );

	// Add the box shape to the ground body.
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Define the dynamic body. We set its position and call the body factory.
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 4.0f, 0.0f };

	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// Define another box shape for our dynamic body.
	b3BoxHull dynamicBox = b3MakeCubeHull( 1.0f );

	// Define the dynamic body shape
	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Set the box density to be non-zero, so it will be dynamic.
	shapeDef.density = 1.0f;

	// Override the default friction.
	shapeDef.baseMaterial.friction = 0.3f;

	// Add the shape to the body.
	b3CreateHullShape( bodyId, &shapeDef, &dynamicBox.base );

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 4 sub-steps. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	b3Pos position = b3Body_GetPosition( bodyId );
	b3Quat rotation = b3Body_GetRotation( bodyId );

	// This is our little game loop.
	for ( int i = 0; i < 90; ++i )
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b3World_Step( worldId, timeStep, subStepCount );

		// Now print the position and angle of the body.
		position = b3Body_GetPosition( bodyId );
		rotation = b3Body_GetRotation( bodyId );

		// printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b3Rot_GetAngle(rotation));
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned ids, so be careful about your world management.
	b3DestroyWorld( worldId );

	ENSURE_SMALL( position.y - 1.00f, 0.01f );
	ENSURE_SMALL( rotation.v.x, 0.01f );
	ENSURE_SMALL( rotation.v.z, 0.01f );

	return 0;
}

int EmptyWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) == true );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 1;

	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );

	return 0;
}

#define BODY_COUNT 10
int DestroyAllBodiesWorld( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) == true );

	int count = 0;
	bool creating = true;

	b3BodyId bodyIds[BODY_COUNT];
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BoxHull cube = b3MakeCubeHull( 0.5f );

	for ( int i = 0; i < 2 * BODY_COUNT + 10; ++i )
	{
		if ( creating )
		{
			if ( count < BODY_COUNT )
			{
				bodyIds[count] = b3CreateBody( worldId, &bodyDef );

				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3CreateHullShape( bodyIds[count], &shapeDef, &cube.base );
				count += 1;
			}
			else
			{
				creating = false;
			}
		}
		else if ( count > 0 )
		{
			b3DestroyBody( bodyIds[count - 1] );
			bodyIds[count - 1] = b3_nullBodyId;
			count -= 1;
		}

		b3World_Step( worldId, 1.0f / 60.0f, 3 );
	}

	b3Counters counters = b3World_GetCounters( worldId );
	ENSURE( counters.bodyCount == 0 );

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );

	return 0;
}

static int TestIsValid( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3BodyDef bodyDef = b3DefaultBodyDef();

	b3BodyId bodyId1 = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId1 ) == true );

	b3BodyId bodyId2 = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsValid( bodyId2 ) == true );

	b3DestroyBody( bodyId1 );
	ENSURE( b3Body_IsValid( bodyId1 ) == false );

	b3DestroyBody( bodyId2 );
	ENSURE( b3Body_IsValid( bodyId2 ) == false );

	b3DestroyWorld( worldId );

	ENSURE( b3World_IsValid( worldId ) == false );
	ENSURE( b3Body_IsValid( bodyId2 ) == false );
	ENSURE( b3Body_IsValid( bodyId1 ) == false );

	return 0;
}

#define WORLD_COUNT ( B3_MAX_WORLDS / 2 )

int TestWorldRecycle( void )
{
	_Static_assert( WORLD_COUNT > 0, "world count" );

	int count = 100;

	b3WorldId worldIds[WORLD_COUNT];

	for ( int i = 0; i < count; ++i )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			worldIds[j] = b3CreateWorld( &worldDef );
			ENSURE( b3World_IsValid( worldIds[j] ) == true );

			b3BodyDef bodyDef = b3DefaultBodyDef();
			b3CreateBody( worldIds[j], &bodyDef );
		}

		for ( int j = 0; j < WORLD_COUNT; ++j )
		{
			float timeStep = 1.0f / 60.0f;
			int subStepCount = 1;

			for ( int k = 0; k < 10; ++k )
			{
				b3World_Step( worldIds[j], timeStep, subStepCount );
			}
		}

		for ( int j = WORLD_COUNT - 1; j >= 0; --j )
		{
			b3DestroyWorld( worldIds[j] );
			ENSURE( b3World_IsValid( worldIds[j] ) == false );
			worldIds[j] = b3_nullWorldId;
		}
	}

	return 0;
}

static bool CustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	ENSURE( context == NULL );
	return true;
}

typedef struct PreSolveCapture
{
	b3ShapeId groundShapeId;
	b3ShapeId sphereShapeId;
	int callCount;
	bool shapeIdsValid;
} PreSolveCapture;

static bool RejectPreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	ENSURE( data != NULL );

	PreSolveCapture* capture = context;
	if ( capture != NULL )
	{
		capture->callCount += 1;
		capture->shapeIdsValid = capture->shapeIdsValid && data->phase == b3_preSolveDiscrete && data->manifoldCount > 0;

		capture->shapeIdsValid = capture->shapeIdsValid && B3_ID_EQUALS( shapeIdA, capture->groundShapeId ) &&
								 B3_ID_EQUALS( shapeIdB, capture->sphereShapeId );
	}

	return false;
}

// This test is here to ensure all API functions link correctly.
int TestWorldCoverage( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();

	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );

	b3World_EnableSleeping( worldId, true );
	b3World_EnableSleeping( worldId, false );
	bool flag = b3World_IsSleepingEnabled( worldId );
	ENSURE( flag == false );

	b3World_EnableContinuous( worldId, false );
	b3World_EnableContinuous( worldId, true );
	flag = b3World_IsContinuousEnabled( worldId );
	ENSURE( flag == true );

	b3World_SetRestitutionThreshold( worldId, 0.0f );
	b3World_SetRestitutionThreshold( worldId, 2.0f );
	float value = b3World_GetRestitutionThreshold( worldId );
	ENSURE( value == 2.0f );

	b3World_SetHitEventThreshold( worldId, 0.0f );
	b3World_SetHitEventThreshold( worldId, 100.0f );
	value = b3World_GetHitEventThreshold( worldId );
	ENSURE( value == 100.0f );

	b3World_SetCustomFilterCallback( worldId, CustomFilter, NULL );
	b3World_SetPreSolveCallback( worldId, RejectPreSolve, NULL );

	b3Vec3 g = { 1.0f, 2.0f };
	b3World_SetGravity( worldId, g );
	b3Vec3 v = b3World_GetGravity( worldId );
	ENSURE( v.x == g.x );
	ENSURE( v.y == g.y );

	b3ExplosionDef explosionDef = b3DefaultExplosionDef();
	b3World_Explode( worldId, &explosionDef );

	b3World_SetContactTuning( worldId, 10.0f, 2.0f, 4.0f );

	b3World_SetMaximumLinearSpeed( worldId, 10.0f );
	value = b3World_GetMaximumLinearSpeed( worldId );
	ENSURE( value == 10.0f );

	b3World_EnableWarmStarting( worldId, true );
	flag = b3World_IsWarmStartingEnabled( worldId );
	ENSURE( flag == true );

	int count = b3World_GetAwakeBodyCount( worldId );
	ENSURE( count == 0 );

	b3World_SetUserData( worldId, &value );
	void* userData = b3World_GetUserData( worldId );
	ENSURE( userData == &value );

	b3World_Step( worldId, 1.0f, 1 );

	b3DestroyWorld( worldId );

	return 0;
}

static int TestPreSolveRejectsContact( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull groundBox = b3MakeBoxHull( 5.0f, 0.5f, 5.0f );
	b3ShapeId groundShapeId = b3CreateHullShape( groundBodyId, &shapeDef, &groundBox.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 2.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );

	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	PreSolveCapture capture = {
		.groundShapeId = groundShapeId,
		.sphereShapeId = sphereShapeId,
		.shapeIdsValid = true,
	};
	b3World_SetPreSolveCallback( worldId, RejectPreSolve, &capture );

	for ( int i = 0; i < 90; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3Pos spherePosition = b3Body_GetPosition( sphereBodyId );
	ENSURE( capture.callCount > 0 );
	ENSURE( capture.shapeIdsValid );
	ENSURE( spherePosition.y < -1.0f );
	ENSURE( b3Body_GetContactCapacity( sphereBodyId ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

typedef struct DisableAllPointsCapture
{
	int callCount;
	int pointCount;
} DisableAllPointsCapture;

static bool DisableAllPreSolvePoints( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;

	DisableAllPointsCapture* capture = context;
	ENSURE( data->phase == b3_preSolveDiscrete );
	capture->callCount += 1;

	for ( int manifoldIndex = 0; manifoldIndex < data->manifoldCount; ++manifoldIndex )
	{
		b3Manifold* manifold = data->manifolds + manifoldIndex;
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			manifold->points[pointIndex].enabled = false;
			capture->pointCount += 1;
		}
	}

	return true;
}

static int TestPreSolveDisablesAllPoints( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull ground = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.9f, 0.0f };
	b3BodyId boxBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3CreateHullShape( boxBodyId, &shapeDef, &box.base );

	DisableAllPointsCapture capture = { 0 };
	b3World_SetPreSolveCallback( worldId, DisableAllPreSolvePoints, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	ENSURE( capture.callCount == 1 );
	ENSURE( capture.pointCount > 0 );
	b3ContactData contactData[1];
	ENSURE( b3Body_GetContactData( boxBodyId, contactData, ARRAY_COUNT( contactData ) ) == 0 );

	b3DestroyWorld( worldId );
	return 0;
}

typedef struct MutablePreSolveCapture
{
	int callCount;
	int sourcePointCount;
	int enabledPointCount;
	bool pointRoundTrip;
} MutablePreSolveCapture;

static bool ModifyPreSolveContact( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;

	MutablePreSolveCapture* capture = context;
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	capture->callCount += 1;
	capture->sourcePointCount = 0;
	capture->enabledPointCount = 0;
	capture->pointRoundTrip = true;

	for ( int manifoldIndex = 0; manifoldIndex < data->manifoldCount; ++manifoldIndex )
	{
		b3Manifold* manifold = data->manifolds + manifoldIndex;
		manifold->normal = b3Normalize( manifold->normal );

		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			b3ManifoldPoint* point = manifold->points + pointIndex;
			capture->sourcePointCount += 1;

			b3Pos worldPoint = b3PreSolve_GetPoint( data, manifoldIndex, pointIndex );
			b3PreSolve_SetPoint( data, manifoldIndex, pointIndex, worldPoint );
			b3Pos roundTrip = b3PreSolve_GetPoint( data, manifoldIndex, pointIndex );
			capture->pointRoundTrip = capture->pointRoundTrip && b3LengthSquared( b3SubPos( roundTrip, worldPoint ) ) < 1.0e-10f;

			point->separation = -0.01f;
			point->friction = 0.25f;
			point->restitution = 0.75f;
			point->maxNormalImpulse = 0.02f;

			if ( pointIndex == 0 && manifold->pointCount > 1 )
			{
				point->enabled = false;
			}
			else
			{
				capture->enabledPointCount += 1;
			}
		}
	}

	return true;
}

static int TestPreSolveMutableContact( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull ground = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.9f, 0.0f };
	b3BodyId boxBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3CreateHullShape( boxBodyId, &shapeDef, &box.base );

	MutablePreSolveCapture capture = { 0 };
	b3World_SetPreSolveCallback( worldId, ModifyPreSolveContact, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	ENSURE( capture.callCount == 1 );
	ENSURE( capture.sourcePointCount > 1 );
	ENSURE( capture.enabledPointCount == capture.sourcePointCount - 1 );
	ENSURE( capture.pointRoundTrip );

	b3ContactData contacts[4];
	int contactCount = b3Body_GetContactData( boxBodyId, contacts, ARRAY_COUNT( contacts ) );
	ENSURE( contactCount == 1 );

	int pointCount = 0;
	for ( int manifoldIndex = 0; manifoldIndex < contacts[0].manifoldCount; ++manifoldIndex )
	{
		const b3Manifold* manifold = contacts[0].manifolds + manifoldIndex;
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			const b3ManifoldPoint* point = manifold->points + pointIndex;
			pointCount += 1;
			ENSURE_SMALL( point->separation + 0.01f, 1.0e-6f );
			ENSURE_SMALL( point->friction - 0.25f, 1.0e-6f );
			ENSURE_SMALL( point->restitution - 0.75f, 1.0e-6f );
			ENSURE_SMALL( point->maxNormalImpulse - 0.02f, 1.0e-6f );
			ENSURE( point->normalImpulse <= point->maxNormalImpulse + 1.0e-6f );
			ENSURE( point->enabled );
		}
	}
	ENSURE( pointCount == capture.enabledPointCount );

	b3DestroyWorld( worldId );
	return 0;
}

typedef struct SolverPointOverride
{
	float friction;
	float restitution;
	float maxNormalImpulse;
	int callCount;
} SolverPointOverride;

static bool OverrideSolverPointProperties( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;

	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	SolverPointOverride* override = context;
	override->callCount += 1;
	for ( int manifoldIndex = 0; manifoldIndex < data->manifoldCount; ++manifoldIndex )
	{
		b3Manifold* manifold = data->manifolds + manifoldIndex;
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			b3ManifoldPoint* point = manifold->points + pointIndex;
			point->friction = override->friction;
			point->restitution = override->restitution;
			point->maxNormalImpulse = override->maxNormalImpulse;
		}
	}

	return true;
}

static int CheckCappedContact( b3BodyId bodyId, float maxNormalImpulse )
{
	b3ContactData contacts[4];
	int contactCount = b3Body_GetContactData( bodyId, contacts, ARRAY_COUNT( contacts ) );
	ENSURE( contactCount > 0 );

	int pointCount = 0;
	bool reachedCap = false;
	for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
	{
		const b3ContactData* contact = contacts + contactIndex;
		for ( int manifoldIndex = 0; manifoldIndex < contact->manifoldCount; ++manifoldIndex )
		{
			const b3Manifold* manifold = contact->manifolds + manifoldIndex;
			for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
			{
				const b3ManifoldPoint* point = manifold->points + pointIndex;
				pointCount += 1;
				ENSURE_SMALL( point->maxNormalImpulse - maxNormalImpulse, 1.0e-6f );
				ENSURE( point->normalImpulse <= maxNormalImpulse + 1.0e-6f );
				reachedCap = reachedCap || point->normalImpulse >= maxNormalImpulse - 1.0e-6f;
			}
		}
	}

	ENSURE( pointCount > 0 );
	ENSURE( reachedCap );
	return 0;
}

// Equality checks alone do not reject NaNs. Check both body state and the stored solver caches.
static int CheckFiniteContactBody( b3BodyId bodyId )
{
	ENSURE( b3IsValidWorldTransform( b3Body_GetTransform( bodyId ) ) );
	ENSURE( b3IsValidVec3( b3Body_GetLinearVelocity( bodyId ) ) );
	ENSURE( b3IsValidVec3( b3Body_GetAngularVelocity( bodyId ) ) );
	b3ContactData contacts[64];
	ENSURE( b3Body_GetContactCapacity( bodyId ) <= ARRAY_COUNT( contacts ) );
	int count = b3Body_GetContactData( bodyId, contacts, ARRAY_COUNT( contacts ) );
	for ( int i = 0; i < count; ++i )
	{
		for ( int j = 0; j < contacts[i].manifoldCount; ++j )
		{
			const b3Manifold* manifold = contacts[i].manifolds + j;
			ENSURE( b3IsValidVec3( manifold->normal ) );
			ENSURE( b3IsValidVec3( manifold->frictionImpulse ) );
			ENSURE( b3IsValidVec3( manifold->rollingImpulse ) );
			ENSURE( b3IsValidFloat( manifold->twistImpulse ) );
			ENSURE( b3IsValidFloat( manifold->maxPushSpeed ) );
			ENSURE( b3IsValidFloat( manifold->contactDampingRatio ) );
			for ( int k = 0; k < manifold->pointCount; ++k )
			{
				const b3ManifoldPoint* point = manifold->points + k;
				ENSURE( b3IsValidVec3( point->anchorA ) && b3IsValidVec3( point->anchorB ) );
				ENSURE( b3IsValidFloat( point->separation ) );
				ENSURE( b3IsValidFloat( point->normalImpulse ) );
				ENSURE( b3IsValidFloat( point->totalNormalImpulse ) );
				ENSURE( b3IsValidFloat( point->normalVelocity ) );
			}
		}
	}
	return 0;
}

typedef struct PushSpeedOverride
{
	b3ShapeId shapeId;
	float defaultSpeed;
	float overrideSpeed;
	int callCount;
	bool overrideEnabled;
	bool defaultsValid;
} PushSpeedOverride;

static bool OverrideManifoldPushSpeed( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	PushSpeedOverride* override = context;
	override->callCount += 1;
	bool selected = B3_ID_EQUALS( shapeIdA, override->shapeId ) || B3_ID_EQUALS( shapeIdB, override->shapeId );
	for ( int i = 0; i < data->manifoldCount; ++i )
	{
		b3Manifold* manifold = data->manifolds + i;
		override->defaultsValid = override->defaultsValid && manifold->maxPushSpeed == override->defaultSpeed;
		if ( selected && override->overrideEnabled )
		{
			manifold->maxPushSpeed = override->overrideSpeed;
		}
	}
	return true;
}

static int TestPreSolveMaxPushSpeed( void )
{
	const float speeds[] = { 0.0f, 6.0f, FLT_MAX };
	for ( int meshIndex = 0; meshIndex < 2; ++meshIndex )
	{
		b3MeshData* mesh = meshIndex != 0 ? b3CreateGridMesh( 4, 4, 2.0f, 0, true ) : NULL;
		ENSURE( meshIndex == 0 || mesh != NULL );
		float firstStepY[ARRAY_COUNT( speeds )];

		for ( int speedIndex = 0; speedIndex < ARRAY_COUNT( speeds ); ++speedIndex )
		{
			b3WorldId worlds[2];
			b3BodyId spheres[2][3];
			PushSpeedOverride overrides[2] = { 0 };
			b3WorldDef worldDef = b3DefaultWorldDef();
			worldDef.gravity = b3Vec3_zero;
			worldDef.workerCount = 1;
			worldDef.enableContinuous = false;
			worldDef.enableSleep = false;

			for ( int worldIndex = 0; worldIndex < 2; ++worldIndex )
			{
				// Compare a local override against the same cap applied to the whole world.
				worldDef.contactSpeed = worldIndex == 0 ? 3.0f : speeds[speedIndex];
				b3WorldId worldId = b3CreateWorld( &worldDef );
				worlds[worldIndex] = worldId;
				b3World_SetContactRecycleDistance( worldId, 1.0f );
				PushSpeedOverride* override = overrides + worldIndex;
				override->defaultSpeed = worldDef.contactSpeed;
				override->overrideSpeed = speeds[speedIndex];
				override->overrideEnabled = worldIndex == 0;
				override->defaultsValid = true;
				b3World_SetPreSolveCallback( worldId, OverrideManifoldPushSpeed, override );

				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.position.y = mesh == NULL ? -0.5f : 0.0f;
				b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				shapeDef.baseMaterial.friction = 0.0f;
				if ( mesh != NULL )
				{
					b3CreateMeshShape( groundId, &shapeDef, mesh, b3Vec3_one );
				}
				else
				{
					b3BoxHull ground = b3MakeBoxHull( 6.0f, 0.5f, 6.0f );
					b3CreateHullShape( groundId, &shapeDef, &ground.base );
				}

				for ( int bodyIndex = 0; bodyIndex < 3; ++bodyIndex )
				{
					bodyDef = b3DefaultBodyDef();
					bodyDef.type = b3_dynamicBody;
					bodyDef.position = (b3Pos){ -2.0f + 2.0f * bodyIndex, 0.1f, 0.0f };
					bodyDef.linearVelocity = (b3Vec3){ 1.0f, 0.0f, 0.0f };
					bodyDef.motionLocks.angularX = true;
					bodyDef.motionLocks.angularY = true;
					bodyDef.motionLocks.angularZ = true;
					b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );
					spheres[worldIndex][bodyIndex] = bodyId;
					b3Body_EnableContactRecycling( bodyId, true );
					shapeDef = b3DefaultShapeDef();
					shapeDef.density = 1.0f;
					shapeDef.baseMaterial.friction = 0.0f;
					// Body 0 is selected, body 1 is ordinary, and body 2 gets a no-op callback.
					shapeDef.enablePreSolveEvents = bodyIndex != 1;
					b3Sphere sphere = { b3Vec3_zero, 0.5f };
					b3ShapeId shapeId = b3CreateSphereShape( bodyId, &shapeDef, &sphere );
					if ( bodyIndex == 0 )
					{
						override->shapeId = shapeId;
					}
				}
				b3World_Step( worldId, 1.0f / 60.0f, 4 );
				ENSURE( override->callCount >= 2 );
				ENSURE( override->defaultsValid );
				for ( int bodyIndex = 0; bodyIndex < 3; ++bodyIndex )
				{
					ENSURE( CheckFiniteContactBody( spheres[worldIndex][bodyIndex] ) == 0 );
				}
			}

			float localY = b3Body_GetPosition( spheres[0][0] ).y;
			float globalY = b3Body_GetPosition( spheres[1][0] ).y;
			float ordinaryY = b3Body_GetPosition( spheres[0][1] ).y;
			firstStepY[speedIndex] = localY;
			ENSURE_SMALL( localY - globalY, 1.0e-5f );
			ENSURE_SMALL( ordinaryY - b3Body_GetPosition( spheres[0][2] ).y, 1.0e-5f );
			ENSURE( ordinaryY > 0.14f && ordinaryY < 0.16f );
			if ( speeds[speedIndex] == 0.0f )
			{
				ENSURE_SMALL( localY - 0.1f, 1.0e-6f );
			}
			else if ( speeds[speedIndex] == 6.0f )
			{
				// This cap binds: recovery is below 6 * dt, but clearly above the ordinary 3 m/s cap.
				ENSURE( localY > ordinaryY + 0.035f );
				ENSURE( localY > 0.18f && localY <= 0.20001f );
			}
			else
			{
				ENSURE( localY > ordinaryY + 0.05f );
			}
			ENSURE_SMALL( b3Body_GetLinearVelocity( spheres[0][0] ).x - 1.0f, 1.0e-6f );

			// The override must reset on the next update. Ordinary recycled contacts must use the new world cap.
			overrides[0].overrideEnabled = false;
			overrides[1].defaultSpeed = 3.0f;
			b3World_SetContactTuning( worlds[1], worldDef.contactHertz, worldDef.contactDampingRatio, 3.0f );
			for ( int worldIndex = 0; worldIndex < 2; ++worldIndex )
			{
				overrides[worldIndex].callCount = 0;
				b3World_Step( worlds[worldIndex], 1.0f / 60.0f, 4 );
				ENSURE( overrides[worldIndex].callCount >= 2 );
				ENSURE( overrides[worldIndex].defaultsValid );
				ENSURE( b3World_GetCounters( worlds[worldIndex] ).recycledContactCount > 0 );
				for ( int bodyIndex = 0; bodyIndex < 3; ++bodyIndex )
				{
					ENSURE( CheckFiniteContactBody( spheres[worldIndex][bodyIndex] ) == 0 );
				}
				ENSURE_SMALL( b3Body_GetPosition( spheres[worldIndex][1] ).y -
							  b3Body_GetPosition( spheres[worldIndex][2] ).y, 1.0e-5f );
			}
			ENSURE_SMALL( b3Body_GetPosition( spheres[0][0] ).y - b3Body_GetPosition( spheres[1][0] ).y, 1.0e-5f );

			b3DestroyWorld( worlds[0] );
			b3DestroyWorld( worlds[1] );
		}
		// Independent of local/global agreement: ignoring the positive override cannot pass this check.
		ENSURE( firstStepY[2] > firstStepY[1] + 0.02f );
		if ( mesh != NULL )
		{
			b3DestroyMesh( mesh );
		}
	}
	return 0;
}

typedef struct DampingRatioOverride
{
	b3ShapeId shapeId;
	float ratio;
	int callCount;
	bool defaultsValid;
	bool verticalOnly;
} DampingRatioOverride;

static bool OverrideManifoldDampingRatio( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	DampingRatioOverride* override = context;
	override->callCount += 1;
	bool selected = B3_ID_EQUALS( shapeIdA, override->shapeId ) || B3_ID_EQUALS( shapeIdB, override->shapeId );
	for ( int i = 0; i < data->manifoldCount; ++i )
	{
		b3Manifold* manifold = data->manifolds + i;
		override->defaultsValid = override->defaultsValid && manifold->contactDampingRatio == -1.0f;
		if ( selected && ( override->verticalOnly == false || b3AbsFloat( manifold->normal.y ) > 0.99f ) )
		{
			manifold->contactDampingRatio = override->ratio;
		}
	}
	return true;
}

static int TestPreSolveContactDampingRatio( void )
{
	const int subSteps[] = { 1, 4, 8 };
	const float ratios[] = { 0.0f, 1.0f, FLT_MAX };
	// Static convex (SIMD), static mesh (scalar), and dynamic-dynamic convex softness.
	for ( int scene = 0; scene < 3; ++scene )
	{
		b3MeshData* mesh = scene == 1 ? b3CreateGridMesh( 4, 4, 2.0f, 0, true ) : NULL;
		ENSURE( scene != 1 || mesh != NULL );
		for ( int subStepIndex = 0; subStepIndex < ARRAY_COUNT( subSteps ); ++subStepIndex )
		{
			for ( int ratioIndex = 0; ratioIndex < ARRAY_COUNT( ratios ); ++ratioIndex )
			{
				b3WorldId worlds[3];
				b3BodyId spheres[3][3];
				DampingRatioOverride overrides[2] = { 0 };
				b3WorldDef worldDef = b3DefaultWorldDef();
				worldDef.gravity = b3Vec3_zero;
				worldDef.workerCount = 1;
				worldDef.enableContinuous = false;
				worldDef.enableSleep = false;
				worldDef.contactSpeed = FLT_MAX;
				float defaultRatio = worldDef.contactDampingRatio;

				for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
				{
					// Local override, equivalent global tuning, and an untouched default world.
					worldDef.contactDampingRatio = worldIndex == 1 ? ratios[ratioIndex] : defaultRatio;
					b3WorldId worldId = b3CreateWorld( &worldDef );
					worlds[worldIndex] = worldId;
					b3World_SetContactRecycleDistance( worldId, 1.0f );
					if ( worldIndex < 2 )
					{
						overrides[worldIndex].ratio = worldIndex == 0 ? ratios[ratioIndex] : -1.0f;
						overrides[worldIndex].defaultsValid = true;
						b3World_SetPreSolveCallback( worldId, OverrideManifoldDampingRatio, overrides + worldIndex );
					}

					for ( int bodyIndex = 0; bodyIndex < 3; ++bodyIndex )
					{
						b3BodyDef bodyDef = b3DefaultBodyDef();
						bodyDef.type = scene == 2 ? b3_dynamicBody : b3_staticBody;
						bodyDef.position = (b3Pos){ -3.0f + 3.0f * bodyIndex, mesh == NULL ? -0.5f : 0.0f, 0.0f };
						bodyDef.motionLocks.angularX = true;
						bodyDef.motionLocks.angularY = true;
						bodyDef.motionLocks.angularZ = true;
						b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
						b3ShapeDef shapeDef = b3DefaultShapeDef();
						shapeDef.baseMaterial.friction = 0.0f;
						if ( mesh != NULL )
						{
							b3CreateMeshShape( groundId, &shapeDef, mesh, (b3Vec3){ 0.2f, 1.0f, 0.2f } );
						}
						else
						{
							b3BoxHull ground = b3MakeBoxHull( 0.75f, 0.5f, 0.75f );
							b3CreateHullShape( groundId, &shapeDef, &ground.base );
						}

						bodyDef.type = b3_dynamicBody;
						bodyDef.position.y = 0.3f;
						b3BodyId sphereId = b3CreateBody( worldId, &bodyDef );
						spheres[worldIndex][bodyIndex] = sphereId;
						b3Body_EnableContactRecycling( sphereId, true );
						// Selected, ordinary (recyclable), and no-op callback contacts.
						shapeDef.enablePreSolveEvents = bodyIndex != 1;
						b3Sphere sphere = { b3Vec3_zero, 0.5f };
						b3ShapeId shapeId = b3CreateSphereShape( sphereId, &shapeDef, &sphere );
						if ( worldIndex < 2 && bodyIndex == 0 )
						{
							overrides[worldIndex].shapeId = shapeId;
						}
					}
				}

				for ( int step = 0; step < 3; ++step )
				{
					if ( step == 1 )
					{
						// Both selected contacts now use defaults; stale local damping must not survive.
						overrides[0].ratio = -1.0f;
						b3World_SetContactTuning( worlds[1], worldDef.contactHertz, defaultRatio, worldDef.contactSpeed );
					}
					for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
					{
						if ( worldIndex < 2 )
						{
							overrides[worldIndex].callCount = 0;
						}
						b3World_Step( worlds[worldIndex], 1.0f / 60.0f, subSteps[subStepIndex] );
						for ( int bodyIndex = 0; bodyIndex < 3; ++bodyIndex )
						{
							ENSURE( CheckFiniteContactBody( spheres[worldIndex][bodyIndex] ) == 0 );
						}
						if ( worldIndex < 2 )
						{
							ENSURE( overrides[worldIndex].defaultsValid );
							ENSURE( overrides[worldIndex].callCount >= 2 );
						}
					}

					ENSURE_SMALL( b3Body_GetPosition( spheres[0][0] ).y - b3Body_GetPosition( spheres[1][0] ).y, 1.0e-6f );
					ENSURE_SMALL( b3Body_GetLinearVelocity( spheres[0][0] ).y -
								  b3Body_GetLinearVelocity( spheres[1][0] ).y, 1.0e-6f );
					if ( step == 0 )
					{
						if ( ratios[ratioIndex] == FLT_MAX )
						{
							ENSURE( b3Body_GetPosition( spheres[0][0] ).y < b3Body_GetPosition( spheres[2][0] ).y - 0.001f );
						}
						else
						{
							ENSURE( b3Body_GetPosition( spheres[0][0] ).y > b3Body_GetPosition( spheres[2][0] ).y + 0.01f );
						}
					}
					else
					{
						ENSURE( b3World_GetCounters( worlds[0] ).recycledContactCount > 0 );
					}
					for ( int bodyIndex = 1; bodyIndex < 3; ++bodyIndex )
					{
						b3Pos localPosition = b3Body_GetPosition( spheres[0][bodyIndex] );
						b3Pos defaultPosition = b3Body_GetPosition( spheres[2][bodyIndex] );
						b3Vec3 localVelocity = b3Body_GetLinearVelocity( spheres[0][bodyIndex] );
						b3Vec3 defaultVelocity = b3Body_GetLinearVelocity( spheres[2][bodyIndex] );
						ENSURE( memcmp( &localPosition, &defaultPosition, sizeof( localPosition ) ) == 0 );
						ENSURE( memcmp( &localVelocity, &defaultVelocity, sizeof( localVelocity ) ) == 0 );
					}
				}

				for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
				{
					b3DestroyWorld( worlds[worldIndex] );
				}
			}
		}
		if ( mesh != NULL )
		{
			b3DestroyMesh( mesh );
		}
	}
	return 0;
}

static int TestPreSolveDampingManifoldIsolation( void )
{
	b3MeshData* mesh = b3CreateHollowBoxMesh( b3Vec3_zero, b3Vec3_one );
	ENSURE( mesh != NULL );
	b3Pos positions[3];
	for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.gravity = b3Vec3_zero;
		worldDef.workerCount = 1;
		worldDef.enableContinuous = false;
		worldDef.contactSpeed = FLT_MAX;
		if ( worldIndex == 1 )
		{
			worldDef.contactDampingRatio = 1.0f;
		}
		b3WorldId worldId = b3CreateWorld( &worldDef );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId meshId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		shapeDef.baseMaterial.friction = 0.0f;
		b3CreateMeshShape( meshId, &shapeDef, mesh, b3Vec3_one );

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 0.7f, 0.7f, 0.0f };
		bodyDef.motionLocks.angularX = true;
		bodyDef.motionLocks.angularY = true;
		bodyDef.motionLocks.angularZ = true;
		b3BodyId sphereId = b3CreateBody( worldId, &bodyDef );
		shapeDef.enablePreSolveEvents = true;
		b3Sphere sphere = { b3Vec3_zero, 0.5f };
		b3ShapeId shapeId = b3CreateSphereShape( sphereId, &shapeDef, &sphere );
		DampingRatioOverride override = {
			.shapeId = shapeId,
			.ratio = 1.0f,
			.defaultsValid = true,
			.verticalOnly = true,
		};
		if ( worldIndex == 0 )
		{
			b3World_SetPreSolveCallback( worldId, OverrideManifoldDampingRatio, &override );
		}
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
		positions[worldIndex] = b3Body_GetPosition( sphereId );
		ENSURE( CheckFiniteContactBody( sphereId ) == 0 );

		b3ContactData contacts[1];
		ENSURE( b3Body_GetContactData( sphereId, contacts, ARRAY_COUNT( contacts ) ) == 1 );
		ENSURE( contacts[0].manifoldCount == 2 );
		if ( worldIndex == 0 )
		{
			ENSURE( override.callCount == 1 && override.defaultsValid );
			for ( int i = 0; i < contacts[0].manifoldCount; ++i )
			{
				const b3Manifold* manifold = contacts[0].manifolds + i;
				float expected = b3AbsFloat( manifold->normal.y ) > 0.99f ? 1.0f : -1.0f;
				ENSURE( manifold->contactDampingRatio == expected );
			}
		}
		b3DestroyWorld( worldId );
	}
	b3DestroyMesh( mesh );

	// One contact, two perpendicular manifolds: local Y damping must not retune X.
	ENSURE_SMALL( positions[0].y - positions[1].y, 1.0e-6f );
	ENSURE_SMALL( positions[0].x - positions[2].x, 1.0e-6f );
	ENSURE( positions[0].y < positions[2].y - 0.01f );
	ENSURE( positions[1].x < positions[0].x - 0.01f );
	return 0;
}

typedef struct ContactTuningCapture
{
	float defaultSpeed;
	float maxPushSpeed;
	float dampingRatio;
	int callCount;
	int loadedCount;
	bool defaultsValid;
} ContactTuningCapture;

static bool OverrideContactTuning( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}
	ContactTuningCapture* capture = context;
	capture->callCount += 1;
	for ( int i = 0; i < data->manifoldCount; ++i )
	{
		b3Manifold* manifold = data->manifolds + i;
		capture->defaultsValid = capture->defaultsValid && manifold->maxPushSpeed == capture->defaultSpeed &&
								 manifold->contactDampingRatio == -1.0f;
		manifold->maxPushSpeed = capture->maxPushSpeed;
		manifold->contactDampingRatio = capture->dampingRatio;
		for ( int j = 0; j < manifold->pointCount; ++j )
		{
			const b3ManifoldPoint* point = manifold->points + j;
			if ( point->persisted && point->normalImpulse > 0.0f )
			{
				capture->loadedCount += 1;
			}
		}
	}
	return true;
}

static int TestPreSolveLoadedTuningTransitions( void )
{
	const float speeds[] = { 6.0f, 0.0f, FLT_MAX, 3.0f };
	const float ratios[] = { 1.0f, 0.0f, FLT_MAX, -1.0f };
	for ( int meshIndex = 0; meshIndex < 2; ++meshIndex )
	{
		b3MeshData* mesh = meshIndex != 0 ? b3CreateGridMesh( 4, 4, 2.0f, 0, true ) : NULL;
		ENSURE( meshIndex == 0 || mesh != NULL );
		b3WorldId worlds[2];
		b3BodyId spheres[2];
		ContactTuningCapture captures[2] = { 0 };
		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.workerCount = 1;
		worldDef.enableSleep = false;
		worldDef.enableContinuous = false;
		for ( int worldIndex = 0; worldIndex < 2; ++worldIndex )
		{
			b3WorldId worldId = b3CreateWorld( &worldDef );
			worlds[worldIndex] = worldId;
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position.y = mesh == NULL ? -0.5f : 0.0f;
			b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			if ( mesh != NULL )
			{
				b3CreateMeshShape( groundId, &shapeDef, mesh, b3Vec3_one );
			}
			else
			{
				b3BoxHull ground = b3MakeBoxHull( 4.0f, 0.5f, 4.0f );
				b3CreateHullShape( groundId, &shapeDef, &ground.base );
			}
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = (b3Pos){ 0.0f, 0.49f, 0.0f };
			spheres[worldIndex] = b3CreateBody( worldId, &bodyDef );
			shapeDef.enablePreSolveEvents = true;
			b3Sphere sphere = { b3Vec3_zero, 0.5f };
			b3CreateSphereShape( spheres[worldIndex], &shapeDef, &sphere );
			for ( int step = 0; step < 60; ++step )
			{
				b3World_Step( worldId, 1.0f / 60.0f, 4 );
			}
			captures[worldIndex].defaultsValid = true;
			b3World_SetPreSolveCallback( worldId, OverrideContactTuning, captures + worldIndex );
		}

		for ( int phase = 0; phase < ARRAY_COUNT( speeds ); ++phase )
		{
			// No teleport or cache reset: switch a resting, gravity-loaded contact to new tuning and back to defaults.
			for ( int worldIndex = 0; worldIndex < 2; ++worldIndex )
			{
				ContactTuningCapture* capture = captures + worldIndex;
				capture->defaultSpeed = worldIndex == 0 ? worldDef.contactSpeed : speeds[phase];
				capture->maxPushSpeed = speeds[phase];
				capture->dampingRatio = worldIndex == 0 ? ratios[phase] : -1.0f;
				capture->callCount = 0;
				capture->loadedCount = 0;
				float ratio = ratios[phase] < 0.0f ? worldDef.contactDampingRatio : ratios[phase];
				if ( worldIndex == 1 )
				{
					b3World_SetContactTuning( worlds[worldIndex], worldDef.contactHertz, ratio, speeds[phase] );
				}
				b3World_Step( worlds[worldIndex], 1.0f / 60.0f, 4 );
				ENSURE( capture->callCount == 1 && capture->loadedCount > 0 && capture->defaultsValid );
				ENSURE( CheckFiniteContactBody( spheres[worldIndex] ) == 0 );
			}
			b3Vec3 deltaPosition = b3SubPos( b3Body_GetPosition( spheres[0] ), b3Body_GetPosition( spheres[1] ) );
			b3Vec3 deltaVelocity = b3Sub( b3Body_GetLinearVelocity( spheres[0] ), b3Body_GetLinearVelocity( spheres[1] ) );
			ENSURE_SMALL( b3Length( deltaPosition ), 1.0e-6f );
			ENSURE_SMALL( b3Length( deltaVelocity ), 1.0e-6f );
		}
		b3DestroyWorld( worlds[0] );
		b3DestroyWorld( worlds[1] );
		if ( mesh != NULL )
		{
			b3DestroyMesh( mesh );
		}
	}
	return 0;
}

static int TestPreSolveMaxNormalImpulse( void )
{
	const float maxNormalImpulse = 0.05f;
	SolverPointOverride override = {
		.friction = 0.0f,
		.restitution = 1.0f,
		.maxNormalImpulse = maxNormalImpulse,
	};

	// Convex contacts use the SIMD solver.
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetRestitutionThreshold( worldId, 0.0f );
	b3World_SetPreSolveCallback( worldId, OverrideSolverPointProperties, &override );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3BoxHull ground = b3MakeBoxHull( 4.0f, 0.5f, 4.0f );
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.95f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { b3Vec3_zero, 0.5f };
	b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	ENSURE( override.callCount > 0 );
	ENSURE( CheckCappedContact( sphereBodyId, maxNormalImpulse ) == 0 );
	ENSURE( b3Body_GetLinearVelocity( sphereBodyId ).y < -9.0f );
	b3DestroyWorld( worldId );

	// Mesh contacts use the scalar solver, including the overflow implementation.
	override.callCount = 0;
	worldId = b3CreateWorld( &worldDef );
	b3World_SetRestitutionThreshold( worldId, 0.0f );
	b3World_SetPreSolveCallback( worldId, OverrideSolverPointProperties, &override );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3MeshData* mesh = b3CreateGridMesh( 2, 2, 2.0f, 0, true );
	ENSURE( mesh != NULL );
	shapeDef = b3DefaultShapeDef();
	b3CreateMeshShape( groundBodyId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.45f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	sphereBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	ENSURE( override.callCount > 0 );
	ENSURE( CheckCappedContact( sphereBodyId, maxNormalImpulse ) == 0 );
	ENSURE( b3Body_GetLinearVelocity( sphereBodyId ).y < -9.0f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

static int TestPreSolveZeroRestitution( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetRestitutionThreshold( worldId, 0.0f );

	SolverPointOverride override = {
		.friction = 0.0f,
		.restitution = 0.0f,
		.maxNormalImpulse = FLT_MAX,
	};
	b3World_SetPreSolveCallback( worldId, OverrideSolverPointProperties, &override );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.restitution = 1.0f;
	b3BoxHull ground = b3MakeBoxHull( 6.0f, 0.5f, 4.0f );
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	b3Sphere sphere = { b3Vec3_zero, 0.5f };
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -2.0f, 0.95f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -5.0f, 0.0f };
	b3BodyId zeroRestitutionBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.baseMaterial.restitution = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3CreateSphereShape( zeroRestitutionBodyId, &shapeDef, &sphere );

	bodyDef.position.x = 2.0f;
	b3BodyId bounceBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef.enablePreSolveEvents = false;
	b3CreateSphereShape( bounceBodyId, &shapeDef, &sphere );

	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	float zeroRestitutionSpeed = b3Body_GetLinearVelocity( zeroRestitutionBodyId ).y;
	float bounceSpeed = b3Body_GetLinearVelocity( bounceBodyId ).y;
	ENSURE( override.callCount > 0 );
	ENSURE( zeroRestitutionSpeed < 1.0f );
	ENSURE( bounceSpeed > 4.0f );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestPreSolveFrictionAffectsSliding( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	SolverPointOverride override = {
		.friction = 0.0f,
		.restitution = 0.0f,
		.maxNormalImpulse = FLT_MAX,
	};
	b3World_SetPreSolveCallback( worldId, OverrideSolverPointProperties, &override );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 1.0f;
	b3BoxHull ground = b3MakeBoxHull( 20.0f, 0.5f, 4.0f );
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ -5.0f, 1.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 5.0f, 0.0f, 0.0f };
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	b3BodyId zeroFrictionBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.baseMaterial.friction = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3CreateHullShape( zeroFrictionBodyId, &shapeDef, &box.base );

	bodyDef.position.x = 5.0f;
	b3BodyId frictionBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef.enablePreSolveEvents = false;
	b3CreateHullShape( frictionBodyId, &shapeDef, &box.base );

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	float zeroFrictionSpeed = b3Body_GetLinearVelocity( zeroFrictionBodyId ).x;
	float frictionSpeed = b3Body_GetLinearVelocity( frictionBodyId ).x;
	ENSURE( override.callCount > 0 );
	ENSURE( zeroFrictionSpeed > 4.5f );
	ENSURE( frictionSpeed < 1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

typedef struct NullPreSolveContactState
{
	b3Pos position;
	b3Quat rotation;
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
} NullPreSolveContactState;

static int RunPreSolveContact( bool useMesh, bool enablePreSolveEvents, b3PreSolveFcn* preSolveFcn, void* preSolveContext,
							   NullPreSolveContactState* result )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_SetPreSolveCallback( worldId, preSolveFcn, preSolveContext );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.baseMaterial.friction = 0.37f;

	b3MeshData* mesh = NULL;
	if ( useMesh )
	{
		mesh = b3CreateGridMesh( 4, 4, 2.0f, 0, true );
		ENSURE( mesh != NULL );
		b3CreateMeshShape( groundBodyId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	}
	else
	{
		b3BoxHull ground = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
		b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );
	}

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, useMesh ? 0.49f : 0.99f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 3.0f, 0.0f, 0.7f };
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	b3BodyId boxBodyId = b3CreateBody( worldId, &bodyDef );

	// Keep mesh recycling policy identical so this test isolates solver property selection.
	b3Body_EnableContactRecycling( boxBodyId, false );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.baseMaterial.friction = 0.37f;
	shapeDef.enablePreSolveEvents = enablePreSolveEvents;
	b3CreateHullShape( boxBodyId, &shapeDef, &box.base );

	for ( int i = 0; i < 20; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b3ContactData contacts[1];
	ENSURE( b3Body_GetContactData( boxBodyId, contacts, ARRAY_COUNT( contacts ) ) == 1 );

	memset( result, 0, sizeof( *result ) );
	result->position = b3Body_GetPosition( boxBodyId );
	result->rotation = b3Body_GetRotation( boxBodyId );
	result->linearVelocity = b3Body_GetLinearVelocity( boxBodyId );
	result->angularVelocity = b3Body_GetAngularVelocity( boxBodyId );

	b3DestroyWorld( worldId );
	if ( mesh != NULL )
	{
		b3DestroyMesh( mesh );
	}
	return 0;
}

static bool NullPreSolveContactStatesEqual( const NullPreSolveContactState* a, const NullPreSolveContactState* b )
{
	return a->position.x == b->position.x && a->position.y == b->position.y && a->position.z == b->position.z &&
		   a->rotation.v.x == b->rotation.v.x && a->rotation.v.y == b->rotation.v.y && a->rotation.v.z == b->rotation.v.z &&
		   a->rotation.s == b->rotation.s && a->linearVelocity.x == b->linearVelocity.x &&
		   a->linearVelocity.y == b->linearVelocity.y && a->linearVelocity.z == b->linearVelocity.z &&
		   a->angularVelocity.x == b->angularVelocity.x && a->angularVelocity.y == b->angularVelocity.y &&
		   a->angularVelocity.z == b->angularVelocity.z;
}

static int TestNullPreSolveCallbackUsesOrdinaryContacts( void )
{
	NullPreSolveContactState ordinaryConvex;
	NullPreSolveContactState flaggedConvex;
	ENSURE( RunPreSolveContact( false, false, NULL, NULL, &ordinaryConvex ) == 0 );
	ENSURE( RunPreSolveContact( false, true, NULL, NULL, &flaggedConvex ) == 0 );
	ENSURE( NullPreSolveContactStatesEqual( &ordinaryConvex, &flaggedConvex ) );

	NullPreSolveContactState ordinaryMesh;
	NullPreSolveContactState flaggedMesh;
	ENSURE( RunPreSolveContact( true, false, NULL, NULL, &ordinaryMesh ) == 0 );
	ENSURE( RunPreSolveContact( true, true, NULL, NULL, &flaggedMesh ) == 0 );
	ENSURE( NullPreSolveContactStatesEqual( &ordinaryMesh, &flaggedMesh ) );
	return 0;
}

static bool ObservePreSolveContact( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;

	if ( data->phase == b3_preSolveDiscrete )
	{
		int* callCount = context;
		*callCount += 1;
	}

	return true;
}

static int TestPreSolveNoOpPreservesWarmStart( void )
{
	NullPreSolveContactState ordinaryState;
	NullPreSolveContactState observedState;
	int callCount = 0;

	// Contact recycling is disabled inside the helper for both worlds so this isolates callback warm starting.
	for ( int meshIndex = 0; meshIndex < 2; ++meshIndex )
	{
		callCount = 0;
		ENSURE( RunPreSolveContact( meshIndex != 0, false, NULL, NULL, &ordinaryState ) == 0 );
		ENSURE( RunPreSolveContact( meshIndex != 0, true, ObservePreSolveContact, &callCount, &observedState ) == 0 );
		ENSURE( callCount > 1 );
		ENSURE( memcmp( &ordinaryState, &observedState, sizeof( ordinaryState ) ) == 0 );
	}
	return 0;
}

typedef struct RepeatedPreSolveCapture
{
	int callCount;
} RepeatedPreSolveCapture;

static bool CountPreSolveContact( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	RepeatedPreSolveCapture* capture = context;
	if ( data->phase == b3_preSolveDiscrete )
	{
		capture->callCount += 1;
	}
	return true;
}

static int TestPreSolveRunsEveryStep( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull ground = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 1.0f, 0.0f };
	b3BodyId boxBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3CreateHullShape( boxBodyId, &shapeDef, &box.base );

	RepeatedPreSolveCapture capture = { 0 };
	b3World_SetPreSolveCallback( worldId, CountPreSolveContact, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( capture.callCount == 1 );

	capture.callCount = 0;
	for ( int i = 0; i < 8; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE( capture.callCount == 8 );
	b3DestroyWorld( worldId );
	return 0;
}

typedef struct SurfacePreSolveCapture
{
	b3ShapeId meshShapeId;
	b3ShapeId meshBoxShapeId;
	b3ShapeId heightShapeId;
	b3ShapeId heightBoxShapeId;
	uint64_t meshMaterialId;
	uint64_t heightMaterialId;
	bool sawMesh;
	bool sawHeightField;
	bool materialIdsValid;
	bool orderValid;
} SurfacePreSolveCapture;

static bool CaptureSurfacePreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	SurfacePreSolveCapture* capture = context;
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	bool isMesh = B3_ID_EQUALS( shapeIdA, capture->meshShapeId );
	bool isHeightField = B3_ID_EQUALS( shapeIdA, capture->heightShapeId );
	if ( isMesh )
	{
		capture->orderValid = capture->orderValid && B3_ID_EQUALS( shapeIdB, capture->meshBoxShapeId );
	}
	else if ( isHeightField )
	{
		capture->orderValid = capture->orderValid && B3_ID_EQUALS( shapeIdB, capture->heightBoxShapeId );
	}
	else
	{
		capture->orderValid = false;
	}

	for ( int manifoldIndex = 0; manifoldIndex < data->manifoldCount; ++manifoldIndex )
	{
		b3Manifold* manifold = data->manifolds + manifoldIndex;
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			b3ManifoldPoint* point = manifold->points + pointIndex;
			if ( isMesh )
			{
				capture->sawMesh = capture->sawMesh || point->triangleIndex != B3_NULL_INDEX;
				uint64_t materialId = b3Shape_GetContactMaterialId( shapeIdA, data->childIndexA, point->triangleIndex );
				capture->materialIdsValid = capture->materialIdsValid && materialId == capture->meshMaterialId;
			}
			else if ( isHeightField )
			{
				capture->sawHeightField = capture->sawHeightField || point->triangleIndex != B3_NULL_INDEX;
				uint64_t materialId = b3Shape_GetContactMaterialId( shapeIdA, data->childIndexA, point->triangleIndex );
				capture->materialIdsValid = capture->materialIdsValid && materialId == capture->heightMaterialId;
			}
		}
	}

	return true;
}

typedef struct TogglePreSolveCapture
{
	b3ShapeId leftGroundShapeId;
	b3ShapeId leftBoxShapeId;
	b3ShapeId rightGroundShapeId;
	b3ShapeId rightBoxShapeId;
	int leftCount;
	int rightCount;
	bool orderValid;
} TogglePreSolveCapture;

static bool CountTogglePreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	TogglePreSolveCapture* capture = context;
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	if ( B3_ID_EQUALS( shapeIdA, capture->leftGroundShapeId ) && B3_ID_EQUALS( shapeIdB, capture->leftBoxShapeId ) )
	{
		capture->leftCount += 1;
	}
	else if ( B3_ID_EQUALS( shapeIdA, capture->rightGroundShapeId ) && B3_ID_EQUALS( shapeIdB, capture->rightBoxShapeId ) )
	{
		capture->rightCount += 1;
	}
	else
	{
		capture->orderValid = false;
	}

	return true;
}

static int TestPreSolveEnableAfterContact( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull leftGround = b3MakeOffsetBoxHull( 1.0f, 0.5f, 1.0f, (b3Vec3){ -2.0f, 0.0f, 0.0f } );
	b3BoxHull rightGround = b3MakeOffsetBoxHull( 1.0f, 0.5f, 1.0f, (b3Vec3){ 2.0f, 0.0f, 0.0f } );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId leftGroundShapeId = b3CreateHullShape( groundBodyId, &shapeDef, &leftGround.base );
	b3ShapeId rightGroundShapeId = b3CreateHullShape( groundBodyId, &shapeDef, &rightGround.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	bodyDef.position = (b3Pos){ -2.0f, 1.0f, 0.0f };
	b3BodyId leftBoxBodyId = b3CreateBody( worldId, &bodyDef );
	bodyDef.position.x = 2.0f;
	b3BodyId rightBoxBodyId = b3CreateBody( worldId, &bodyDef );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	b3ShapeId leftBoxShapeId = b3CreateHullShape( leftBoxBodyId, &shapeDef, &box.base );
	b3ShapeId rightBoxShapeId = b3CreateHullShape( rightBoxBodyId, &shapeDef, &box.base );

	TogglePreSolveCapture capture = {
		.leftGroundShapeId = leftGroundShapeId,
		.leftBoxShapeId = leftBoxShapeId,
		.rightGroundShapeId = rightGroundShapeId,
		.rightBoxShapeId = rightBoxShapeId,
		.orderValid = true,
	};
	b3World_SetPreSolveCallback( worldId, CountTogglePreSolve, &capture );

	// Establish both contacts while pre-solve is disabled.
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( capture.leftCount == 0 && capture.rightCount == 0 );
	b3ContactData contactData[1];
	ENSURE( b3Body_GetContactData( leftBoxBodyId, contactData, ARRAY_COUNT( contactData ) ) == 1 );
	ENSURE( b3Body_GetContactData( rightBoxBodyId, contactData, ARRAY_COUNT( contactData ) ) == 1 );

	// Enabling one shape updates its existing contact immediately. The enabled contact cannot recycle,
	// so the callback runs on every subsequent narrow-phase update. The other shape on the same body
	// must not affect its contact.
	b3Shape_EnablePreSolveEvents( leftGroundShapeId, true );
	for ( int i = 0; i < 4; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE( capture.leftCount == 4 );
	ENSURE( capture.rightCount == 0 );
	ENSURE( capture.orderValid );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestPreSolveDisableAfterContact( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull ground = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enablePreSolveEvents = true;
	b3ShapeId groundShapeId = b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	bodyDef.position = (b3Pos){ 0.0f, 1.0f, 0.0f };
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	b3BodyId boxBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3ShapeId boxShapeId = b3CreateHullShape( boxBodyId, &shapeDef, &box.base );

	TogglePreSolveCapture capture = {
		.leftGroundShapeId = groundShapeId,
		.leftBoxShapeId = boxShapeId,
		.orderValid = true,
	};
	b3World_SetPreSolveCallback( worldId, CountTogglePreSolve, &capture );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( capture.leftCount == 1 );

	// The contact remains enabled while the other participating shape still requests pre-solve.
	b3Shape_EnablePreSolveEvents( groundShapeId, false );
	for ( int i = 0; i < 2; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	ENSURE( capture.leftCount == 3 );

	// Once both current shape flags are clear, the callback stops and ordinary recycling is eligible again.
	b3Shape_EnablePreSolveEvents( boxShapeId, false );
	for ( int i = 0; i < 4; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	ENSURE( capture.leftCount == 3 );
	ENSURE( capture.orderValid );

	b3DestroyWorld( worldId );
	return 0;
}

typedef struct RecyclingPreSolveCapture
{
	int callCount;
} RecyclingPreSolveCapture;

static bool ModifyRecyclingPreSolveGeometry( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;

	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	RecyclingPreSolveCapture* capture = context;
	capture->callCount += 1;
	for ( int manifoldIndex = 0; manifoldIndex < data->manifoldCount; ++manifoldIndex )
	{
		b3Manifold* manifold = data->manifolds + manifoldIndex;
		manifold->normal = (b3Vec3){ 1.0f, 0.0f, 0.0f };
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			b3Pos point = { 4.0f + pointIndex, 4.0f, 4.0f };
			b3PreSolve_SetPoint( data, manifoldIndex, pointIndex, point );
			manifold->points[pointIndex].separation = -2.0f;
			manifold->points[pointIndex].maxNormalImpulse = 0.0f;
		}
	}

	return true;
}

static int TestPreSolveDisableRebuildsGeometry( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId groundBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull ground = b3MakeBoxHull( 3.0f, 0.5f, 3.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundBodyId, &shapeDef, &ground.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	bodyDef.position = (b3Pos){ 0.0f, 0.9f, 0.0f };
	bodyDef.motionLocks.angularX = true;
	bodyDef.motionLocks.angularY = true;
	bodyDef.motionLocks.angularZ = true;
	b3BodyId boxBodyId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3ShapeId boxShapeId = b3CreateHullShape( boxBodyId, &shapeDef, &box.base );

	RecyclingPreSolveCapture capture = { 0 };
	b3World_SetPreSolveCallback( worldId, ModifyRecyclingPreSolveGeometry, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( capture.callCount == 1 );

	b3ContactData contactData[1];
	ENSURE( b3Body_GetContactData( boxBodyId, contactData, ARRAY_COUNT( contactData ) ) == 1 );
	ENSURE( contactData[0].manifolds[0].normal.x > 0.9f );

	b3Shape_EnablePreSolveEvents( boxShapeId, false );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( capture.callCount == 1 );
	ENSURE( b3Body_GetContactData( boxBodyId, contactData, ARRAY_COUNT( contactData ) ) == 1 );

	int pointCount = 0;
	for ( int manifoldIndex = 0; manifoldIndex < contactData[0].manifoldCount; ++manifoldIndex )
	{
		const b3Manifold* manifold = contactData[0].manifolds + manifoldIndex;
		ENSURE( manifold->normal.y > 0.9f );
		ENSURE_SMALL( manifold->normal.x, 1.0e-5f );
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			const b3ManifoldPoint* point = manifold->points + pointIndex;
			pointCount += 1;
			ENSURE( point->separation > -0.2f && point->separation < 0.05f );
			ENSURE( b3LengthSquared( point->anchorA ) < 4.0f );
		}
	}
	ENSURE( pointCount > 0 );

	b3DestroyWorld( worldId );
	return 0;
}

static int TestPreSolveMeshAndHeightField( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3SurfaceMaterial meshMaterials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	meshMaterials[0].userMaterialId = 0x1111;
	meshMaterials[1].userMaterialId = 0x1234;
	b3Vec3 meshVertices[4] = {
		{ -2.0f, 0.0f, -2.0f },
		{ -2.0f, 0.0f, 2.0f },
		{ 2.0f, 0.0f, 2.0f },
		{ 2.0f, 0.0f, -2.0f },
	};
	int32_t meshIndices[6] = { 0, 1, 2, 0, 2, 3 };
	uint8_t meshMaterialIndices[2] = { 1, 1 };
	b3MeshDef meshDef = {
		.vertices = meshVertices,
		.indices = meshIndices,
		.materialIndices = meshMaterialIndices,
		.vertexCount = ARRAY_COUNT( meshVertices ),
		.triangleCount = 2,
		.identifyEdges = true,
	};
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	ENSURE( mesh != NULL );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId meshBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.materials = meshMaterials;
	shapeDef.materialCount = ARRAY_COUNT( meshMaterials );
	b3ShapeId meshShapeId = b3CreateMeshShape( meshBodyId, &shapeDef, mesh, (b3Vec3){ 1.0f, 1.0f, 1.0f } );

	float heights[16] = { 0 };
	uint8_t heightMaterialIndices[9];
	memset( heightMaterialIndices, 1, sizeof( heightMaterialIndices ) );
	b3HeightFieldDef heightDef = {
		.heights = heights,
		.materialIndices = heightMaterialIndices,
		.scale = { 1.0f, 1.0f, 1.0f },
		.countX = 4,
		.countZ = 4,
		.globalMinimumHeight = -1.0f,
		.globalMaximumHeight = 1.0f,
	};
	b3HeightFieldData* heightField = b3CreateHeightField( &heightDef );
	ENSURE( heightField != NULL );

	b3SurfaceMaterial heightMaterials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	heightMaterials[0].userMaterialId = 0x2222;
	heightMaterials[1].userMaterialId = 0x5678;
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 10.0f, 0.0f, 0.0f };
	b3BodyId heightBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.materials = heightMaterials;
	shapeDef.materialCount = ARRAY_COUNT( heightMaterials );
	b3ShapeId heightShapeId = b3CreateHeightFieldShape( heightBodyId, &shapeDef, heightField );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.4f, 0.0f };
	b3BodyId meshBoxBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeId meshBoxShapeId = b3CreateHullShape( meshBoxBodyId, &shapeDef, &box.base );

	bodyDef.position = (b3Pos){ 11.5f, 0.4f, 1.5f };
	b3BodyId heightBoxBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeId heightBoxShapeId = b3CreateHullShape( heightBoxBodyId, &shapeDef, &box.base );

	SurfacePreSolveCapture capture = {
		.meshShapeId = meshShapeId,
		.meshBoxShapeId = meshBoxShapeId,
		.heightShapeId = heightShapeId,
		.heightBoxShapeId = heightBoxShapeId,
		.meshMaterialId = meshMaterials[1].userMaterialId,
		.heightMaterialId = heightMaterials[1].userMaterialId,
		.materialIdsValid = true,
		.orderValid = true,
	};
	b3World_SetPreSolveCallback( worldId, CaptureSurfacePreSolve, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	ENSURE( capture.sawMesh );
	ENSURE( capture.sawHeightField );
	ENSURE( capture.materialIdsValid );
	ENSURE( capture.orderValid );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	b3DestroyHeightField( heightField );
	return 0;
}

static int TestContactMaterialInvalidIndices( void )
{
	b3SurfaceMaterial materials[2] = { b3DefaultSurfaceMaterial(), b3DefaultSurfaceMaterial() };
	materials[0].userMaterialId = 0xABC1;
	materials[1].userMaterialId = 0xABC2;

	b3MeshData* mesh = b3CreateGridMesh( 2, 2, 1.0f, 2, false );
	ENSURE( mesh != NULL );
	const uint8_t* meshMaterialIndices = b3GetMeshMaterialIndices( mesh );
	ENSURE( meshMaterialIndices != NULL );
	int nonzeroTriangleIndex = B3_NULL_INDEX;
	for ( int triangleIndex = 0; triangleIndex < mesh->triangleCount; ++triangleIndex )
	{
		if ( meshMaterialIndices[triangleIndex] == 1 )
		{
			nonzeroTriangleIndex = triangleIndex;
			break;
		}
	}
	ENSURE( nonzeroTriangleIndex != B3_NULL_INDEX );

	float heights[9] = { 0 };
	uint8_t heightMaterialIndices[4] = { 1, 1, 1, 1 };
	b3HeightFieldDef heightDef = {
		.heights = heights,
		.materialIndices = heightMaterialIndices,
		.scale = { 1.0f, 1.0f, 1.0f },
		.countX = 3,
		.countZ = 3,
		.globalMinimumHeight = -1.0f,
		.globalMaximumHeight = 1.0f,
	};
	b3HeightFieldData* heightField = b3CreateHeightField( &heightDef );
	ENSURE( heightField != NULL );

	b3BoxHull compoundHull = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
	b3SurfaceMaterial hullMaterial = b3DefaultSurfaceMaterial();
	hullMaterial.userMaterialId = 0xC001;
	b3CompoundHullDef hullDef = {
		.hull = &compoundHull.base,
		.transform = b3Transform_identity,
		.material = hullMaterial,
	};
	b3CompoundMeshDef compoundMeshDef = {
		.meshData = mesh,
		.transform = { .p = { 3.0f, 0.0f, 0.0f }, .q = b3Quat_identity },
		.scale = { 1.0f, 1.0f, 1.0f },
		.materials = materials,
		.materialCount = ARRAY_COUNT( materials ),
	};
	b3CompoundDef compoundDef = {
		.hulls = &hullDef,
		.hullCount = 1,
		.meshes = &compoundMeshDef,
		.meshCount = 1,
	};
	b3CompoundData* compound = b3CreateCompound( &compoundDef );
	ENSURE( compound != NULL );

	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;

	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.materials = materials;
	shapeDef.materialCount = ARRAY_COUNT( materials );
	b3BodyId meshBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeId meshShapeId = b3CreateMeshShape( meshBodyId, &shapeDef, mesh, b3Vec3_one );

	b3BodyId heightBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeId heightShapeId = b3CreateHeightFieldShape( heightBodyId, &shapeDef, heightField );

	b3BodyId compoundBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	b3ShapeId compoundShapeId = b3CreateBakedCompoundShape( compoundBodyId, &shapeDef, compound );

	ENSURE( b3Shape_GetContactMaterialId( meshShapeId, B3_NULL_INDEX, nonzeroTriangleIndex ) == materials[1].userMaterialId );
	ENSURE( b3Shape_GetContactMaterialId( meshShapeId, B3_NULL_INDEX, -1 ) == 0 );
	ENSURE( b3Shape_GetContactMaterialId( meshShapeId, B3_NULL_INDEX, mesh->triangleCount ) == 0 );

	int heightTriangleCount = 2 * ( heightField->rowCount - 1 ) * ( heightField->columnCount - 1 );
	ENSURE( b3Shape_GetContactMaterialId( heightShapeId, B3_NULL_INDEX, 0 ) == materials[1].userMaterialId );
	ENSURE( b3Shape_GetContactMaterialId( heightShapeId, B3_NULL_INDEX, -1 ) == 0 );
	ENSURE( b3Shape_GetContactMaterialId( heightShapeId, B3_NULL_INDEX, heightTriangleCount ) == 0 );

	int compoundMeshChildIndex = compound->capsuleCount + compound->hullCount;
	int compoundChildCount = compound->capsuleCount + compound->hullCount + compound->meshCount + compound->sphereCount;
	ENSURE( b3Shape_GetContactMaterialId( compoundShapeId, compoundMeshChildIndex, nonzeroTriangleIndex ) ==
			materials[1].userMaterialId );
	ENSURE( b3Shape_GetContactMaterialId( compoundShapeId, -1, nonzeroTriangleIndex ) == 0 );
	ENSURE( b3Shape_GetContactMaterialId( compoundShapeId, compoundChildCount, nonzeroTriangleIndex ) == 0 );
	ENSURE( b3Shape_GetContactMaterialId( compoundShapeId, compoundMeshChildIndex, -1 ) == 0 );
	ENSURE( b3Shape_GetContactMaterialId( compoundShapeId, compoundMeshChildIndex, mesh->triangleCount ) == 0 );

	b3DestroyWorld( worldId );
	b3DestroyCompound( compound );
	b3DestroyHeightField( heightField );
	b3DestroyMesh( mesh );
	return 0;
}

typedef struct CompactManifoldCapture
{
	b3ShapeId meshShapeId;
	b3ShapeId sphereShapeId;
	int callCount;
	int sourceManifoldCount;
	bool orderValid;
} CompactManifoldCapture;

static bool CompactOnePreSolveManifold( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	CompactManifoldCapture* capture = context;
	if ( data->phase != b3_preSolveDiscrete )
	{
		return true;
	}

	capture->callCount += 1;
	capture->orderValid =
		capture->orderValid && B3_ID_EQUALS( shapeIdA, capture->meshShapeId ) && B3_ID_EQUALS( shapeIdB, capture->sphereShapeId );
	capture->sourceManifoldCount = data->manifoldCount;
	if ( data->manifoldCount > 1 )
	{
		b3Manifold* manifold = data->manifolds;
		for ( int pointIndex = 0; pointIndex < manifold->pointCount; ++pointIndex )
		{
			manifold->points[pointIndex].enabled = false;
		}
	}

	return true;
}

static int TestPreSolvePartialManifoldCompaction( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3MeshData* mesh = b3CreateHollowBoxMesh( b3Vec3_zero, (b3Vec3){ 1.0f, 1.0f, 1.0f } );
	ENSURE( mesh != NULL );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId meshBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId meshShapeId = b3CreateMeshShape( meshBodyId, &shapeDef, mesh, b3Vec3_one );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.7f, 0.7f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { b3Vec3_zero, 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	CompactManifoldCapture capture = {
		.meshShapeId = meshShapeId,
		.sphereShapeId = sphereShapeId,
		.orderValid = true,
	};
	b3World_SetPreSolveCallback( worldId, CompactOnePreSolveManifold, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	ENSURE( capture.callCount == 1 );
	ENSURE( capture.sourceManifoldCount > 1 );
	ENSURE( capture.orderValid );
	b3ContactData contactData[1];
	ENSURE( b3Body_GetContactData( sphereBodyId, contactData, ARRAY_COUNT( contactData ) ) == 1 );
	ENSURE( contactData[0].manifoldCount == capture.sourceManifoldCount - 1 );
	ENSURE( contactData[0].manifoldCount > 0 );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

static int TestSensor( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Wall from x = 1 to x = 2
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 1.5f, 11.0f, 0.0f };
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull box = b3MakeBoxHull( 0.5f, 10.0f, 1.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.enableSensorEvents = true;
	b3CreateHullShape( wallId, &shapeDef, &box.base );

	// Bullet fired towards the wall
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.isBullet = true;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Pos){ 7.39814f, 4.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -20.0f, 0.0f, 0.0f };
	b3BodyId bulletId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.isSensor = true;
	shapeDef.enableSensorEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.1f };
	b3CreateSphereShape( bulletId, &shapeDef, &sphere );

	int beginCount = 0;
	int endCount = 0;

	while ( true )
	{
		float timeStep = 1.0f / 60.0f;
		int subStepCount = 4;
		b3World_Step( worldId, timeStep, subStepCount );

		b3Pos bulletPos = b3Body_GetPosition( bulletId );
		// printf( "Bullet pos: %g %g\n", bulletPos.x, bulletPos.y );

		b3SensorEvents events = b3World_GetSensorEvents( worldId );

		if ( events.beginCount > 0 )
		{
			beginCount += 1;
		}

		if ( events.endCount > 0 )
		{
			endCount += 1;
		}

		if ( bulletPos.x < -1.0f )
		{
			break;
		}
	}

	b3DestroyWorld( worldId );

	ENSURE( beginCount == 1 );
	ENSURE( endCount == 1 );

	return 0;
}

static int TestContactEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Static ground
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3ShapeId groundShapeId = b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Dynamic sphere dropped onto the ground; restitution causes it to bounce so we get end events
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 0.0f, 5.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableContactEvents = true;
	shapeDef.baseMaterial.restitution = 0.6f;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	int beginCount = 0;
	int endCount = 0;
	bool idsChecked = false;

	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactEvents events = b3World_GetContactEvents( worldId );

		if ( events.beginCount > 0 && idsChecked == false )
		{
			b3ContactBeginTouchEvent be = events.beginEvents[0];
			ENSURE( B3_ID_EQUALS( be.shapeIdA, groundShapeId ) );
			ENSURE( B3_ID_EQUALS( be.shapeIdB, sphereShapeId ) );
			ENSURE( b3Contact_IsValid( be.contactId ) );
			idsChecked = true;
		}

		beginCount += events.beginCount;
		endCount += events.endCount;
	}

	b3DestroyWorld( worldId );

	ENSURE( idsChecked );
	ENSURE( beginCount >= 1 );
	ENSURE( endCount >= 1 );

	return 0;
}

static int TestHitEvents( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.hitEventThreshold = 1.0f;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// Static ground
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, -0.5f, 0.0f };
	b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( 10.0f, 0.5f, 10.0f );
	b3ShapeDef groundShapeDef = b3DefaultShapeDef();
	b3CreateHullShape( groundId, &groundShapeDef, &groundBox.base );

	// Sphere driven into the ground fast enough to clear the hit threshold
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Pos){ 0.0f, 2.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ 0.0f, -30.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enableHitEvents = true;
	shapeDef.baseMaterial.userMaterialId = 7;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
	b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	int hitCount = 0;
	float capturedSpeed = 0.0f;
	uint64_t capturedMaterialA = 0;
	uint64_t capturedMaterialB = 0;
	b3Vec3 capturedNormal = { 0.0f, 0.0f, 0.0f };

	for ( int i = 0; i < 30; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );

		b3ContactEvents events = b3World_GetContactEvents( worldId );
		if ( events.hitCount > 0 && hitCount == 0 )
		{
			b3ContactHitEvent hit = events.hitEvents[0];
			capturedSpeed = hit.approachSpeed;
			capturedNormal = hit.normal;
			capturedMaterialA = hit.userMaterialIdA;
			capturedMaterialB = hit.userMaterialIdB;
		}

		hitCount += events.hitCount;
	}

	b3DestroyWorld( worldId );

	ENSURE( hitCount >= 1 );
	ENSURE( capturedSpeed > 1.0f );
	// Head-on vertical impact: normal lies along Y
	ENSURE_SMALL( capturedNormal.x, 0.01f );
	ENSURE_SMALL( capturedNormal.z, 0.01f );
	// One side of the contact carries the sphere's user material
	ENSURE( capturedMaterialA == 7 || capturedMaterialB == 7 );

	return 0;
}

// Hit-event material lookup must respect the compound child that participated in the
// contact. Two children with distinct userMaterialIds at separated positions, dropped
// sphere strikes one specifically. Without the fix, both children would report
// materials[0] and the strike on hull 1 would be misattributed.
static int TestCompoundHitEvents( void )
{
	const uint64_t kHullMaterialA = 11;
	const uint64_t kHullMaterialB = 22;
	const uint64_t kSphereMaterial = 99;
	const float kHullCenterX = 3.0f;

	for ( int side = 0; side < 2; ++side )
	{
		uint64_t expectedHullMaterial = ( side == 0 ) ? kHullMaterialA : kHullMaterialB;
		float spawnX = ( side == 0 ) ? -kHullCenterX : kHullCenterX;

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.hitEventThreshold = 1.0f;
		b3WorldId worldId = b3CreateWorld( &worldDef );

		// Build a compound with two hulls at opposite x positions, distinct userMaterialIds
		b3BoxHull boxA = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );
		b3BoxHull boxB = b3MakeBoxHull( 1.0f, 1.0f, 1.0f );

		b3SurfaceMaterial matA = b3DefaultSurfaceMaterial();
		matA.userMaterialId = kHullMaterialA;

		b3SurfaceMaterial matB = b3DefaultSurfaceMaterial();
		matB.userMaterialId = kHullMaterialB;

		b3CompoundHullDef hulls[2];
		hulls[0].hull = &boxA.base;
		hulls[0].transform = (b3Transform){ { -kHullCenterX, 0.0f, 0.0f }, b3Quat_identity };
		hulls[0].material = matA;
		hulls[1].hull = &boxB.base;
		hulls[1].transform = (b3Transform){ { kHullCenterX, 0.0f, 0.0f }, b3Quat_identity };
		hulls[1].material = matB;

		b3CompoundDef compoundDef = { 0 };
		compoundDef.hulls = hulls;
		compoundDef.hullCount = 2;
		b3CompoundData* compound = b3CreateCompound( &compoundDef );
		ENSURE( compound != NULL );

		// Static body holds the compound
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId compoundBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
		b3CreateBakedCompoundShape( compoundBodyId, &compoundShapeDef, compound );

		// Sphere driven straight down onto the chosen child
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.position = (b3Pos){ spawnX, 3.0f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, -30.0f, 0.0f };
		b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
		sphereShapeDef.density = 1.0f;
		sphereShapeDef.enableHitEvents = true;
		sphereShapeDef.baseMaterial.userMaterialId = kSphereMaterial;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere );

		int hitCount = 0;
		uint64_t capturedMaterialA = 0;
		uint64_t capturedMaterialB = 0;

		for ( int i = 0; i < 30; ++i )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );

			b3ContactEvents events = b3World_GetContactEvents( worldId );
			if ( events.hitCount > 0 && hitCount == 0 )
			{
				b3ContactHitEvent hit = events.hitEvents[0];
				capturedMaterialA = hit.userMaterialIdA;
				capturedMaterialB = hit.userMaterialIdB;
			}
			hitCount += events.hitCount;
		}

		b3DestroyWorld( worldId );
		b3DestroyCompound( compound );

		ENSURE( hitCount >= 1 );
		// Sphere material on one side
		ENSURE( capturedMaterialA == kSphereMaterial || capturedMaterialB == kSphereMaterial );
		// Struck compound child's material on the other side. The pre-fix code returned
		// materials[0] (kHullMaterialA) for both sides, so a strike on the +x child would fail.
		ENSURE( capturedMaterialA == expectedHullMaterial || capturedMaterialB == expectedHullMaterial );
	}

	return 0;
}

enum
{
	kChild0MaterialId = 101,
	kChild1MaterialId = 202,
	kProbeMaterialId = 999,
};

// No context pointer on mixing callbacks, so capture through file scope
static struct
{
	int callCount;
	bool sawChild0;
	bool sawChild1;
	float mixedFriction;
} materialCapture;

static float CaptureFrictionMix( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB )
{
	materialCapture.callCount += 1;

	if ( userMaterialIdA == kChild0MaterialId || userMaterialIdB == kChild0MaterialId )
	{
		materialCapture.sawChild0 = true;
	}

	if ( userMaterialIdA == kChild1MaterialId || userMaterialIdB == kChild1MaterialId )
	{
		materialCapture.sawChild1 = true;
		materialCapture.mixedFriction = sqrtf( frictionA * frictionB );
	}

	return sqrtf( frictionA * frictionB );
}

// Contact material selection must use the struck compound child's material, not entry 0
// of the compound material table. Child 0 gets low friction, child 1 high friction, and a
// unit friction sphere strikes only child 1, so the mixing callback must see child 1's
// values. Covers sphere, capsule, and hull children. Issue #69.
static int TestCompoundContactMaterials( void )
{
	b3SurfaceMaterial mat0 = b3DefaultSurfaceMaterial();
	mat0.friction = 0.04f;
	mat0.userMaterialId = kChild0MaterialId;

	b3SurfaceMaterial mat1 = b3DefaultSurfaceMaterial();
	mat1.friction = 0.81f;
	mat1.userMaterialId = kChild1MaterialId;

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	// Children at x = -3 and x = +3, all with their top face or surface at y = 0.5
	b3CompoundSphereDef spheres[2] = {
		{ .sphere = { { -3.0f, 0.0f, 0.0f }, 0.5f }, .material = mat0 },
		{ .sphere = { { 3.0f, 0.0f, 0.0f }, 0.5f }, .material = mat1 },
	};

	b3CompoundCapsuleDef capsules[2] = {
		{ .capsule = { { -3.0f, 0.0f, -0.5f }, { -3.0f, 0.0f, 0.5f }, 0.5f }, .material = mat0 },
		{ .capsule = { { 3.0f, 0.0f, -0.5f }, { 3.0f, 0.0f, 0.5f }, 0.5f }, .material = mat1 },
	};

	b3CompoundHullDef hulls[2] = {
		{ .hull = &box.base, .transform = { { -3.0f, 0.0f, 0.0f }, b3Quat_identity }, .material = mat0 },
		{ .hull = &box.base, .transform = { { 3.0f, 0.0f, 0.0f }, b3Quat_identity }, .material = mat1 },
	};

	for ( int childType = 0; childType < 3; ++childType )
	{
		b3CompoundDef compoundDef = { 0 };
		if ( childType == 0 )
		{
			compoundDef.spheres = spheres;
			compoundDef.sphereCount = 2;
		}
		else if ( childType == 1 )
		{
			compoundDef.capsules = capsules;
			compoundDef.capsuleCount = 2;
		}
		else
		{
			compoundDef.hulls = hulls;
			compoundDef.hullCount = 2;
		}

		b3CompoundData* compound = b3CreateCompound( &compoundDef );
		ENSURE( compound != NULL );

		memset( &materialCapture, 0, sizeof( materialCapture ) );

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.frictionCallback = CaptureFrictionMix;
		b3WorldId worldId = b3CreateWorld( &worldDef );

		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_staticBody;
		b3BodyId compoundBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef compoundShapeDef = b3DefaultShapeDef();
		b3CreateBakedCompoundShape( compoundBodyId, &compoundShapeDef, compound );

		// Sphere driven straight down onto child 1
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.gravityScale = 0.0f;
		bodyDef.position = (b3Pos){ 3.0f, 3.0f, 0.0f };
		bodyDef.linearVelocity = (b3Vec3){ 0.0f, -30.0f, 0.0f };
		b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef sphereShapeDef = b3DefaultShapeDef();
		sphereShapeDef.density = 1.0f;
		sphereShapeDef.baseMaterial.friction = 1.0f;
		sphereShapeDef.baseMaterial.userMaterialId = kProbeMaterialId;
		b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.5f };
		b3CreateSphereShape( sphereBodyId, &sphereShapeDef, &sphere );

		for ( int i = 0; i < 30; ++i )
		{
			b3World_Step( worldId, 1.0f / 60.0f, 4 );
		}

		b3DestroyWorld( worldId );
		b3DestroyCompound( compound );

		ENSURE( materialCapture.callCount > 0 );
		// The pre-fix code fed material table entry 0 to the mixing callback for every child
		ENSURE( materialCapture.sawChild0 == false );
		ENSURE( materialCapture.sawChild1 == true );
		// sqrt(0.81 * 1.0)
		ENSURE_SMALL( materialCapture.mixedFriction - 0.9f, 1e-5f );
	}

	return 0;
}

static int TestSetWorkerCount( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	ENSURE( b3World_IsValid( worldId ) );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	CreateJunkyard( worldId );
	StepJunkyard( worldId, 1 );

	b3World_SetWorkerCount( worldId, 4 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 2 );

	b3World_SetWorkerCount( worldId, 4 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 4 );

	StepJunkyard( worldId, 3 );

	b3World_SetWorkerCount( worldId, 0 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 4 );

	b3World_SetWorkerCount( worldId, -5 );
	ENSURE( b3World_GetWorkerCount( worldId ) == 1 );

	StepJunkyard( worldId, 5 );

	b3World_SetWorkerCount( worldId, B3_MAX_WORKERS + 10 );
	ENSURE( b3World_GetWorkerCount( worldId ) == B3_MAX_WORKERS );

	StepJunkyard( worldId, 2 );

	b3DestroyWorld( worldId );

	return 0;
}

static int TestPreSolveOverflowTuning( void )
{
	b3WorldId worlds[3];
	b3BodyId bodies[3][32];
	int bodyCounts[3] = { 0 };
	ContactTuningCapture capture = {
		.defaultSpeed = 3.0f,
		.maxPushSpeed = 0.03f,
		.dampingRatio = 1.0f,
		.defaultsValid = true,
	};
	for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
	{
		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.workerCount = 1;
		worldDef.enableContinuous = false;
		worldDef.enableSleep = false;
		if ( worldIndex == 1 )
		{
			worldDef.contactSpeed = capture.maxPushSpeed;
			worldDef.contactDampingRatio = capture.dampingRatio;
		}
		worlds[worldIndex] = b3CreateWorld( &worldDef );
		OverflowColorPileData pile = CreateOverflowColorPile( worlds[worldIndex] );
		b3World_Step( worlds[worldIndex], 0.0f, 1 );
		bodies[worldIndex][0] = pile.hubId;
		bodyCounts[worldIndex] = 1;
		b3ContactData contacts[32];
		ENSURE( b3Body_GetContactCapacity( pile.hubId ) <= ARRAY_COUNT( contacts ) );
		int contactCount = b3Body_GetContactData( pile.hubId, contacts, ARRAY_COUNT( contacts ) );
		for ( int i = 0; i < contactCount; ++i )
		{
			b3ShapeId shapes[2] = { contacts[i].shapeIdA, contacts[i].shapeIdB };
			for ( int j = 0; j < 2; ++j )
			{
				// Opt in every pile shape, including ordinary colored and overflow contacts.
				b3Shape_EnablePreSolveEvents( shapes[j], true );
				b3BodyId bodyId = b3Shape_GetBody( shapes[j] );
				if ( b3Body_GetType( bodyId ) != b3_dynamicBody || B3_ID_EQUALS( bodyId, pile.hubId ) )
				{
					continue;
				}
				ENSURE( bodyCounts[worldIndex] < ARRAY_COUNT( bodies[worldIndex] ) );
				bodies[worldIndex][bodyCounts[worldIndex]++] = bodyId;
			}
		}
		ENSURE( bodyCounts[worldIndex] == pile.neighborCount + 1 );
		b3Shape_EnablePreSolveEvents( pile.groundShapeId, true );
		if ( worldIndex == 0 )
		{
			b3World_SetPreSolveCallback( worlds[worldIndex], OverrideContactTuning, &capture );
		}
	}

	for ( int step = 0; step < 4; ++step )
	{
		capture.callCount = 0;
		capture.loadedCount = 0;
		for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
		{
			b3World_Step( worlds[worldIndex], 1.0f / 60.0f, 4 );
			ENSURE( b3World_GetCounters( worlds[worldIndex] ).colorCounts[B3_GRAPH_COLOR_COUNT - 1] > 0 );
			for ( int i = 0; i < bodyCounts[worldIndex]; ++i )
			{
				ENSURE( CheckFiniteContactBody( bodies[worldIndex][i] ) == 0 );
			}
		}
		ENSURE( capture.callCount > B3_GRAPH_COLOR_COUNT && capture.defaultsValid );
		if ( step > 0 )
		{
			ENSURE( capture.loadedCount > 0 );
		}
		float effect = 0.0f;
		for ( int i = 0; i < bodyCounts[0]; ++i )
		{
			b3Pos localPosition = b3Body_GetPosition( bodies[0][i] );
			ENSURE_SMALL( b3Length( b3SubPos( localPosition, b3Body_GetPosition( bodies[1][i] ) ) ), 1.0e-6f );
			ENSURE_SMALL( b3Length( b3Sub( b3Body_GetLinearVelocity( bodies[0][i] ),
									 b3Body_GetLinearVelocity( bodies[1][i] ) ) ), 1.0e-6f );
			effect += b3Length( b3SubPos( localPosition, b3Body_GetPosition( bodies[2][i] ) ) );
		}
		ENSURE( effect > 1.0e-4f );
	}
	for ( int worldIndex = 0; worldIndex < 3; ++worldIndex )
	{
		b3DestroyWorld( worlds[worldIndex] );
	}
	return 0;
}

// Verifies the b3*_Overflow solver path. The scene puts >B3_DYNAMIC_COLOR_COUNT
// dyn-dyn contacts on a single hub body so several land in the overflow color.
// The new Prepare/Store contactId-pairing asserts in contact_solver.c fire here
// if Store reads constraints from the wrong (base, spans) pair, so the test
// catches the bug even though world state ends up plausible (memory layout is
// stable within a build, so determinism alone is not a witness).
static int TestOverflowColorPile( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	OverflowColorPileData data = CreateOverflowColorPile( worldId );
	(void)data;

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;

	// One step would be enough to trip the asserts, but several steps also
	// exercise the warm-start path (Store -> manifold impulse -> Prepare).
	int stepCount = 10;
	for ( int i = 0; i < stepCount; ++i )
	{
		b3World_Step( worldId, timeStep, subStepCount );
	}

	// Confirm the scene actually populated the overflow color. Without this,
	// a future change to graph coloring could silently turn the test into a
	// no-op.
	b3Counters counters = b3World_GetCounters( worldId );
	int overflowContacts = counters.colorCounts[B3_GRAPH_COLOR_COUNT - 1];

	b3DestroyWorld( worldId );

	ENSURE( overflowContacts > 0 );
	return 0;
}

// Exposes that b3Body_EnableSleep mutates body->flags but never syncs
// bodySim->flags / bodyState->flags. When a body created with
// enableSleep=false is later flipped on, body->flags gains b3_enableSleep
// but bodySim/bodyState do not. The flag-sync assertion in
// b3ValidateSolverSets then fires on the next world step.
//
// Under Debug + BOX3D_VALIDATE this crashes; after b3Body_EnableSleep
// calls b3SyncBodyFlags the test runs cleanly.
static int EnableSleepFlagSyncTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = false;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	ENSURE( b3Body_IsSleepEnabled( bodyId ) == false );

	b3Body_EnableSleep( bodyId, true );
	ENSURE( b3Body_IsSleepEnabled( bodyId ) == true );

	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3DestroyWorld( worldId );
	return 0;
}

// Exposes that b3Body_SetBullet writes only bodySim->flags without
// touching body->flags, while b3SyncBodyFlags overwrites bodySim from
// body->flags. Any subsequent b3Body_SetMotionLocks or b3Body_SetType
// silently wipes (or re-asserts) the bullet bit.
//
// Reproducer A: create with isBullet=false, SetBullet(true), then
// SetMotionLocks. b3Body_IsBullet returns false today (wiped); should be
// true after fix.
//
// Reproducer B: create with isBullet=true, SetBullet(false), then
// SetMotionLocks. b3Body_IsBullet returns true today (re-asserted); should
// be false after fix.
static int SetBulletDriftTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = false;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		ENSURE( b3Body_IsBullet( bodyId ) == false );

		b3Body_SetBullet( bodyId, true );
		ENSURE( b3Body_IsBullet( bodyId ) == true );

		b3MotionLocks locks = { 0 };
		locks.linearX = true;
		b3Body_SetMotionLocks( bodyId, locks );

		ENSURE( b3Body_IsBullet( bodyId ) == true );
	}

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.isBullet = true;
		b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

		ENSURE( b3Body_IsBullet( bodyId ) == true );

		b3Body_SetBullet( bodyId, false );
		ENSURE( b3Body_IsBullet( bodyId ) == false );

		b3MotionLocks locks = { 0 };
		locks.linearX = true;
		b3Body_SetMotionLocks( bodyId, locks );

		ENSURE( b3Body_IsBullet( bodyId ) == false );
	}

	b3DestroyWorld( worldId );
	return 0;
}

// Regression: b3Body_EnableSleep used to set world->locked = true before checking
// for a no-op change, then early-return without unlocking. The next mutator call
// would then trip the assert in b3GetUnlockedWorld.
static int EnableSleepNoopUnlockTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.enableSleep = true;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	// No-op: enableSleep is already true. Must not leak the world lock.
	b3Body_EnableSleep( bodyId, true );

	// Would assert in b3GetUnlockedWorld if the lock had leaked.
	b3Body_EnableSleep( bodyId, false );
	ENSURE( b3Body_IsSleepEnabled( bodyId ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

static int EnableContactRecyclingTest( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;

	// Default is enabled
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == true );

	b3Body_EnableContactRecycling( bodyA, false );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == false );

	b3Body_EnableContactRecycling( bodyA, true );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyA ) == true );

	// Per-def opt-out at creation
	bodyDef.enableContactRecycling = false;
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDef );
	ENSURE( b3Body_IsContactRecyclingEnabled( bodyB ) == false );

	// Stepping after toggling must not trip the flag-sync validator
	b3World_Step( worldId, 1.0f / 60.0f, 4 );

	b3DestroyWorld( worldId );
	return 0;
}

// Identical hull data is shared through a reference counted world database.
static int TestHullDatabase( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BoxHull box = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	b3BodyId bodyA = b3CreateBody( worldId, &bodyDef );
	b3BodyId bodyB = b3CreateBody( worldId, &bodyDef );

	b3ShapeDef shapeDef = b3DefaultShapeDef();

	// Two shapes built from identical data share one owned copy in the world database.
	b3ShapeId shapeA = b3CreateHullShape( bodyA, &shapeDef, &box.base );
	b3ShapeId shapeB = b3CreateHullShape( bodyB, &shapeDef, &box.base );

	const b3HullData* gotA = b3Shape_GetHull( shapeA );
	const b3HullData* gotB = b3Shape_GetHull( shapeB );

	// Both shapes point at the single shared copy
	ENSURE( gotA == gotB );

	// The shared copy is owned by the world, not the caller's stack hull
	ENSURE( gotA != &box.base );

	// A box built on an independent stack frame must de-duplicate to the same shared copy.
	// This holds only if content hashing sees deterministic padding bytes.
	b3BoxHull box2 = b3MakeBoxHull( 0.5f, 0.5f, 0.5f );
	b3BodyId bodyC = b3CreateBody( worldId, &bodyDef );
	b3ShapeId shapeC = b3CreateHullShape( bodyC, &shapeDef, &box2.base );
	ENSURE( b3Shape_GetHull( shapeC ) == gotA );
	b3DestroyShape( shapeC, true );

	// Setting a shape's hull to its own sole shared copy must not free it mid update.
	b3BoxHull box3 = b3MakeBoxHull( 0.3f, 0.3f, 0.3f );
	b3BodyId bodyD = b3CreateBody( worldId, &bodyDef );
	b3ShapeId shapeD = b3CreateHullShape( bodyD, &shapeDef, &box3.base );
	const b3HullData* gotD = b3Shape_GetHull( shapeD );
	b3Shape_SetHull( shapeD, gotD );
	ENSURE( b3Shape_GetHull( shapeD ) == gotD );
	b3DestroyShape( shapeD, true );

	// Releasing one reference keeps the other alive
	b3DestroyShape( shapeA, true );
	const b3HullData* stillB = b3Shape_GetHull( shapeB );
	ENSURE( stillB == gotB );

	b3DestroyShape( shapeB, true );

	// World destroy asserts the database drained to zero references
	b3DestroyWorld( worldId );
	return 0;
}

typedef struct ExplosionResult
{
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
} ExplosionResult;

// Explode just off the +x side of a centered sphere and capture the impulse it receives.
// The shape, the blast point, and the witness math all run in the body local frame, so the
// result must not depend on how far the body sits from the world origin.
static ExplosionResult RunExplosion( b3Pos base )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = base;
	b3BodyId bodyId = b3CreateBody( worldId, &bodyDef );

	b3Sphere sphere = { b3Vec3_zero, 1.0f };
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateSphereShape( bodyId, &shapeDef, &sphere );

	// Blast sits 3 units along +x, so the body is pushed back along -x
	b3ExplosionDef explosionDef = b3DefaultExplosionDef();
	explosionDef.position = b3OffsetPos( base, (b3Vec3){ 3.0f, 0.0f, 0.0f } );
	explosionDef.radius = 5.0f;
	explosionDef.falloff = 0.0f;
	explosionDef.impulsePerArea = 10.0f;
	b3World_Explode( worldId, &explosionDef );

	ExplosionResult result;
	result.linearVelocity = b3Body_GetLinearVelocity( bodyId );
	result.angularVelocity = b3Body_GetAngularVelocity( bodyId );

	b3DestroyWorld( worldId );
	return result;
}

static int TestExplosion( void )
{
	ExplosionResult origin = RunExplosion( b3Pos_zero );

	// Pushed away from the blast along -x. A centered sphere has no transverse or angular component.
	ENSURE( origin.linearVelocity.x < -1.0e-4f );
	ENSURE_SMALL( origin.linearVelocity.y, 1.0e-6f );
	ENSURE_SMALL( origin.linearVelocity.z, 1.0e-6f );
	ENSURE_SMALL( b3Length( origin.angularVelocity ), 1.0e-6f );

	// The same blast far from the origin must produce the same impulse. The world position only
	// reaches float in the relative difference, so the result holds where a naive cast would not.
	ExplosionResult far = RunExplosion( (b3Pos){ 1.0e7f, 1.0e7f, 1.0e7f } );
	ENSURE_SMALL( far.linearVelocity.x - origin.linearVelocity.x, 1.0e-5f );
	ENSURE_SMALL( far.linearVelocity.y - origin.linearVelocity.y, 1.0e-5f );
	ENSURE_SMALL( far.linearVelocity.z - origin.linearVelocity.z, 1.0e-5f );

	return 0;
}

typedef struct ContinuousPreSolveCapture
{
	int continuousCount;
	bool dataValid;
} ContinuousPreSolveCapture;

static bool RejectContinuousPreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)shapeIdA;
	(void)shapeIdB;
	ContinuousPreSolveCapture* capture = context;
	if ( data->phase == b3_preSolveContinuous )
	{
		capture->continuousCount += 1;
		capture->dataValid = capture->dataValid && data->manifolds == NULL && data->manifoldCount == 0 &&
							 B3_IS_NULL( data->contactId ) && b3IsValidPosition( data->point ) && b3IsNormalized( data->normal ) && 0.0f < data->fraction &&
							 data->fraction < 1.0f;
	}

	return false;
}

static int TestContinuousPreSolveRejectsCandidate( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull wall = b3MakeBoxHull( 0.1f, 5.0f, 5.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wallId, &shapeDef, &wall.base );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -30.0f, 0.0f, 0.0f };
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { b3Vec3_zero, 0.25f };
	b3CreateSphereShape( ballId, &shapeDef, &sphere );

	ContinuousPreSolveCapture capture = { .dataValid = true };
	b3World_SetPreSolveCallback( worldId, RejectContinuousPreSolve, &capture );
	for ( int i = 0; i < 20; ++i )
	{
		b3World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	ENSURE( capture.continuousCount > 0 );
	ENSURE( capture.dataValid );
	ENSURE( b3Body_GetPosition( ballId ).x < -1.0f );

	b3DestroyWorld( worldId );
	return 0;
}

typedef struct LaterContinuousCandidateCapture
{
	b3ShapeId staticShapeId;
	b3ShapeId sphereShapeId;
	int expectedChildIndex;
	int rejectedNearCount;
	int acceptedFarCount;
	float nearFraction;
	float farFraction;
	bool dataValid;
} LaterContinuousCandidateCapture;

static bool SelectLaterContinuousCandidate( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	LaterContinuousCandidateCapture* capture = context;
	if ( data->phase != b3_preSolveContinuous )
	{
		return true;
	}

	capture->dataValid = capture->dataValid && B3_ID_EQUALS( shapeIdA, capture->staticShapeId ) &&
						 B3_ID_EQUALS( shapeIdB, capture->sphereShapeId ) && data->childIndexA == capture->expectedChildIndex &&
						 data->childIndexB == B3_NULL_INDEX && data->triangleIndexA != B3_NULL_INDEX &&
						 data->triangleIndexB == B3_NULL_INDEX && B3_IS_NULL( data->contactId );

	if ( data->point.x > 0.5f )
	{
		capture->rejectedNearCount += 1;
		capture->nearFraction = data->fraction;
		return false;
	}

	capture->acceptedFarCount += 1;
	capture->farFraction = data->fraction;
	return true;
}

static int RunLaterContinuousCandidateTest( bool useCompound )
{
	b3Vec3 vertices[8] = {
		{ 1.0f, -5.0f, -5.0f }, { 1.0f, 5.0f, -5.0f }, { 1.0f, 5.0f, 5.0f }, { 1.0f, -5.0f, 5.0f },
		{ 0.0f, -5.0f, -5.0f }, { 0.0f, 5.0f, -5.0f }, { 0.0f, 5.0f, 5.0f }, { 0.0f, -5.0f, 5.0f },
	};
	int32_t indices[12] = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
	b3MeshDef meshDef = {
		.vertices = vertices,
		.indices = indices,
		.vertexCount = ARRAY_COUNT( vertices ),
		.triangleCount = 4,
	};
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	ENSURE( mesh != NULL );

	b3CompoundData* compound = NULL;
	if ( useCompound )
	{
		b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
		b3CompoundMeshDef compoundMeshDef = {
			.meshData = mesh,
			.transform = b3Transform_identity,
			.scale = b3Vec3_one,
			.materials = &material,
			.materialCount = 1,
		};
		b3CompoundDef compoundDef = {
			.meshes = &compoundMeshDef,
			.meshCount = 1,
		};
		compound = b3CreateCompound( &compoundDef );
		ENSURE( compound != NULL );
	}

	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	b3BodyId staticBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId staticShapeId = useCompound ? b3CreateBakedCompoundShape( staticBodyId, &shapeDef, compound )
										  : b3CreateMeshShape( staticBodyId, &shapeDef, mesh, b3Vec3_one );

	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -210.0f, 0.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { b3Vec3_zero, 0.25f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );

	LaterContinuousCandidateCapture capture = {
		.staticShapeId = staticShapeId,
		.sphereShapeId = sphereShapeId,
		.expectedChildIndex = useCompound ? 0 : B3_NULL_INDEX,
		.dataValid = true,
	};
	b3World_SetPreSolveCallback( worldId, SelectLaterContinuousCandidate, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 1 );

	ENSURE( capture.rejectedNearCount > 0 );
	ENSURE( capture.acceptedFarCount > 0 );
	ENSURE( capture.nearFraction < capture.farFraction );
	ENSURE( capture.dataValid );
	b3Pos position = b3Body_GetPosition( sphereBodyId );
	ENSURE( 0.15f < position.x && position.x < 0.5f );

	b3DestroyWorld( worldId );
	if ( compound != NULL )
	{
		b3DestroyCompound( compound );
	}
	b3DestroyMesh( mesh );
	return 0;
}

static int TestContinuousPreSolveAcceptsLaterTriangle( void )
{
	ENSURE( RunLaterContinuousCandidateTest( false ) == 0 );
	ENSURE( RunLaterContinuousCandidateTest( true ) == 0 );
	return 0;
}

typedef struct ContinuousContactIdCapture
{
	b3ShapeId nearShapeId;
	b3ShapeId farShapeId;
	b3ShapeId sphereShapeId;
	b3ContactId discreteContactId;
	int nearChildIndex;
	int farChildIndex;
	int discreteCount;
	int nearCount;
	int farCount;
	int discreteAction; // 0: accept, 1: reject, 2: disable all points
	const void* continuousThread;
	float marker;
	float maxPushSpeed;
	float contactDampingRatio;
	bool farSharesContact;
	bool expectFresh;
	bool dataValid;
} ContinuousContactIdCapture;

static bool CheckContinuousContactId( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	ContinuousContactIdCapture* capture = context;
	bool nearPair = B3_ID_EQUALS( shapeIdA, capture->nearShapeId ) && B3_ID_EQUALS( shapeIdB, capture->sphereShapeId );
	bool farPair = B3_ID_EQUALS( shapeIdA, capture->farShapeId ) && B3_ID_EQUALS( shapeIdB, capture->sphereShapeId );
	if ( nearPair == false && farPair == false )
	{
		return true;
	}

	if ( data->phase == b3_preSolveDiscrete )
	{
		capture->discreteCount += 1;
		capture->dataValid = capture->dataValid && nearPair && data->childIndexA == capture->nearChildIndex &&
							 b3Contact_IsValid( data->contactId );
		capture->discreteContactId = data->contactId;
		if ( b3Contact_IsValid( data->contactId ) == false )
		{
			return false;
		}
		b3ContactData contact = b3Contact_GetData( data->contactId );
		capture->dataValid = capture->dataValid && B3_ID_EQUALS( contact.shapeIdA, shapeIdA ) &&
							 B3_ID_EQUALS( contact.shapeIdB, shapeIdB ) && contact.manifolds == data->manifolds &&
							 contact.manifoldCount == data->manifoldCount;
		for ( int i = 0; i < data->manifoldCount; ++i )
		{
			b3Manifold* manifold = data->manifolds + i;
			manifold->normal = (b3Vec3){ 0.0f, 1.0f, 0.0f };
			manifold->maxPushSpeed = capture->maxPushSpeed;
			manifold->contactDampingRatio = capture->contactDampingRatio;
			for ( int j = 0; j < manifold->pointCount; ++j )
			{
				manifold->points[j].separation = -0.01f;
				manifold->points[j].friction = 0.0f;
				manifold->points[j].maxNormalImpulse = capture->marker;
				manifold->points[j].enabled = capture->discreteAction != 2;
			}
		}
		return capture->discreteAction != 1;
	}

	bool near = data->point.x > 0.5f;
	if ( near )
	{
		capture->nearCount += 1;
	}
	else
	{
		capture->farCount += 1;
	}
	capture->dataValid = capture->dataValid && data->manifolds == NULL && data->manifoldCount == 0 &&
						 data->childIndexA == ( near ? capture->nearChildIndex : capture->farChildIndex );
	bool expectContact = capture->expectFresh && ( near || capture->farSharesContact );
	if ( expectContact )
	{
		capture->dataValid = capture->dataValid && B3_ID_EQUALS( data->contactId, capture->discreteContactId ) &&
							 b3Contact_IsValid( data->contactId );
		if ( b3Contact_IsValid( data->contactId ) )
		{
			b3ContactData contact = b3Contact_GetData( data->contactId );
			capture->dataValid = capture->dataValid && contact.manifoldCount > 0;
			float totalImpulse = 0.0f;
			for ( int i = 0; i < contact.manifoldCount; ++i )
			{
				const b3Manifold* manifold = contact.manifolds + i;
				capture->dataValid = capture->dataValid && manifold->normal.y == 1.0f &&
									 manifold->maxPushSpeed == capture->maxPushSpeed &&
									 manifold->contactDampingRatio == capture->contactDampingRatio;
				for ( int j = 0; j < manifold->pointCount; ++j )
				{
					const b3ManifoldPoint* point = manifold->points + j;
					capture->dataValid = capture->dataValid && point->maxNormalImpulse == capture->marker &&
										 b3IsValidFloat( point->normalImpulse ) && b3IsValidFloat( point->totalNormalImpulse );
					totalImpulse += point->totalNormalImpulse;
				}
			}
			capture->dataValid = capture->dataValid && b3IsValidFloat( totalImpulse ) && totalImpulse > 0.0f;
		}
	}
	else
	{
		capture->dataValid = capture->dataValid && B3_IS_NULL( data->contactId );
	}

	// Reject only the near plane. Even a matching contact id does not authorize the far triangle.
	return near == false;
}

static int TestContinuousPreSolveContactId( void )
{
	// One mesh contact, different shapes on the same body, and different children in one compound.
	for ( int layout = 0; layout < 3; ++layout )
	{
		b3Vec3 vertices[8] = {
			{ 1.0f, -5.0f, -5.0f }, { 1.0f, 5.0f, -5.0f }, { 1.0f, 5.0f, 5.0f }, { 1.0f, -5.0f, 5.0f },
			{ 0.0f, -5.0f, -5.0f }, { 0.0f, 5.0f, -5.0f }, { 0.0f, 5.0f, 5.0f }, { 0.0f, -5.0f, 5.0f },
		};
		int32_t indices[12] = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
		b3MeshDef meshDef = {
			.vertices = vertices,
			.indices = indices,
			.vertexCount = layout == 0 ? 8 : 4,
			.triangleCount = layout == 0 ? 4 : 2,
		};
		b3MeshData* nearMesh = b3CreateMesh( &meshDef, NULL, 0 );
		ENSURE( nearMesh != NULL );
		b3MeshData* farMesh = NULL;
		if ( layout != 0 )
		{
			meshDef.vertices = vertices + 4;
			farMesh = b3CreateMesh( &meshDef, NULL, 0 );
			ENSURE( farMesh != NULL );
		}

		b3CompoundData* compound = NULL;
		if ( layout == 2 )
		{
			b3SurfaceMaterial material = b3DefaultSurfaceMaterial();
			b3CompoundMeshDef meshes[2] = {
				{ .meshData = nearMesh, .transform = b3Transform_identity, .scale = b3Vec3_one,
				  .materials = &material, .materialCount = 1 },
				{ .meshData = farMesh, .transform = b3Transform_identity, .scale = b3Vec3_one,
				  .materials = &material, .materialCount = 1 },
			};
			b3CompoundDef compoundDef = { .meshes = meshes, .meshCount = 2 };
			compound = b3CreateCompound( &compoundDef );
			ENSURE( compound != NULL );
		}

		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.gravity = b3Vec3_zero;
		worldDef.workerCount = 1;
		b3WorldId worldId = b3CreateWorld( &worldDef );
		b3BodyDef bodyDef = b3DefaultBodyDef();
		b3BodyId staticBodyId = b3CreateBody( worldId, &bodyDef );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3ShapeId nearShapeId = layout == 2 ? b3CreateBakedCompoundShape( staticBodyId, &shapeDef, compound )
										  : b3CreateMeshShape( staticBodyId, &shapeDef, nearMesh, b3Vec3_one );
		b3ShapeId farShapeId = layout == 1 ? b3CreateMeshShape( staticBodyId, &shapeDef, farMesh, b3Vec3_one ) : nearShapeId;

		bodyDef.type = b3_dynamicBody;
		bodyDef.position = (b3Pos){ 1.255f, 0.0f, 0.0f };
		b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
		shapeDef.enablePreSolveEvents = true;
		b3Sphere sphere = { b3Vec3_zero, 0.25f };
		b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );
		ContinuousContactIdCapture capture = {
			.nearShapeId = nearShapeId,
			.farShapeId = farShapeId,
			.sphereShapeId = sphereShapeId,
			.nearChildIndex = layout == 2 ? 0 : B3_NULL_INDEX,
			.farChildIndex = layout == 2 ? 1 : B3_NULL_INDEX,
			.marker = 1000000.0f,
			.maxPushSpeed = FLT_MAX,
			.contactDampingRatio = 1.0f,
			.farSharesContact = layout == 0,
			.expectFresh = true,
			.dataValid = true,
		};
		b3World_SetPreSolveCallback( worldId, CheckContinuousContactId, &capture );
		// A collision-only update must not make the following advancing step's stamp ambiguous.
		b3World_Step( worldId, 0.0f, 1 );
		ENSURE( capture.discreteCount == 1 && capture.dataValid );
		for ( int step = 0; step < 8; ++step )
		{
			b3ContactId destroyedContactId = b3_nullContactId;
			if ( step == 6 && layout == 0 )
			{
				// Reuse the freed contact slot with a new shape generation, but no current approval.
				destroyedContactId = capture.discreteContactId;
				b3DestroyBody( sphereBodyId );
				ENSURE( b3Contact_IsValid( destroyedContactId ) == false );
				sphereBodyId = b3CreateBody( worldId, &bodyDef );
				capture.sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );
			}
			capture.discreteAction = step == 2 || step == 6 ? 1 : step == 4 ? 2 : 0;
			capture.expectFresh = capture.discreteAction == 0;
			capture.discreteCount = 0;
			capture.nearCount = 0;
			capture.farCount = 0;
			capture.marker += 1.0f;
			b3Body_SetTransform( sphereBodyId, bodyDef.position, b3Quat_identity );
			b3Body_SetLinearVelocity( sphereBodyId, (b3Vec3){ -210.0f, 0.0f, 0.0f } );
			b3World_Step( worldId, 1.0f / 60.0f, 1 );
			ENSURE( capture.discreteCount == 1 );
			ENSURE( capture.nearCount > 0 && capture.farCount > 0 );
			ENSURE( capture.dataValid );
			ENSURE( CheckFiniteContactBody( sphereBodyId ) == 0 );
			if ( B3_IS_NON_NULL( destroyedContactId ) )
			{
				ENSURE( capture.discreteContactId.index1 == destroyedContactId.index1 );
				ENSURE( capture.discreteContactId.generation != destroyedContactId.generation );
				ENSURE( b3Contact_IsValid( destroyedContactId ) == false );
			}
			float x = b3Body_GetPosition( sphereBodyId ).x;
			ENSURE( 0.15f < x && x < 0.5f );

			if ( step == 0 && layout == 0 )
			{
				// Record only the seed snapshot, with a solved, callback-modified contact and its step stamp.
				b3Recording* recording = b3CreateRecording( 0 );
				ENSURE( recording != NULL );
				b3World_StartRecording( worldId, recording );
				b3World_StopRecording( worldId );
				b3RecPlayer* player = b3CreatePlayer( b3Recording_GetData( recording ), b3Recording_GetSize( recording ), 1 );
				ENSURE( player != NULL );
				b3DestroyRecording( recording );
				b3WorldId restoredWorldId = b3RecPlayer_GetWorldId( player );
				b3BodyId restoredWallId = b3RecPlayer_GetBodyId( player, 0 );
				b3BodyId restoredSphereId = b3RecPlayer_GetBodyId( player, 1 );
				ContinuousContactIdCapture restoredCapture = capture;
				ENSURE( b3Body_GetShapes( restoredWallId, &restoredCapture.nearShapeId, 1 ) == 1 );
				ENSURE( b3Body_GetShapes( restoredSphereId, &restoredCapture.sphereShapeId, 1 ) == 1 );
				restoredCapture.farShapeId = restoredCapture.nearShapeId;
				b3World_SetPreSolveCallback( restoredWorldId, CheckContinuousContactId, &restoredCapture );
				b3Pos firstPosition = b3Pos_zero;
				b3Vec3 firstVelocity = b3Vec3_zero;
				for ( int continuation = 0; continuation < 4; ++continuation )
				{
					// Restore in place while the callback is attached; host callback wiring is not serialized.
					b3RecPlayer_Restart( player );
					b3ContactData restoredContact[1];
					ENSURE( b3Body_GetContactData( restoredSphereId, restoredContact, 1 ) == 1 );
					ENSURE( restoredContact[0].manifoldCount > 0 );
					ENSURE( restoredContact[0].manifolds[0].maxPushSpeed == capture.maxPushSpeed );
					ENSURE( restoredContact[0].manifolds[0].contactDampingRatio == capture.contactDampingRatio );
					ENSURE( restoredContact[0].manifolds[0].points[0].maxNormalImpulse == capture.marker );
					restoredCapture.discreteContactId = restoredContact[0].contactId;
					restoredCapture.discreteAction = continuation % 3;
					restoredCapture.expectFresh = restoredCapture.discreteAction == 0;
					restoredCapture.discreteCount = 0;
					restoredCapture.nearCount = 0;
					restoredCapture.farCount = 0;
					restoredCapture.marker += 1.0f;
					b3Body_SetTransform( restoredSphereId, bodyDef.position, b3Quat_identity );
					b3Body_SetLinearVelocity( restoredSphereId, (b3Vec3){ -210.0f, 0.0f, 0.0f } );
					b3World_Step( restoredWorldId, 1.0f / 60.0f, 1 );
					ENSURE( restoredCapture.discreteCount == 1 && restoredCapture.nearCount > 0 && restoredCapture.farCount > 0 );
					ENSURE( restoredCapture.dataValid );
					ENSURE( CheckFiniteContactBody( restoredSphereId ) == 0 );
					b3Pos position = b3Body_GetPosition( restoredSphereId );
					b3Vec3 velocity = b3Body_GetLinearVelocity( restoredSphereId );
					ENSURE( 0.15f < position.x && position.x < 0.5f );
					if ( continuation == 0 )
					{
						firstPosition = position;
						firstVelocity = velocity;
					}
					else if ( continuation == 3 )
					{
						ENSURE( memcmp( &position, &firstPosition, sizeof( position ) ) == 0 );
						ENSURE( memcmp( &velocity, &firstVelocity, sizeof( velocity ) ) == 0 );
					}
				}
				b3DestroyPlayer( player );
			}
		}

		b3DestroyWorld( worldId );
		if ( compound != NULL )
		{
			b3DestroyCompound( compound );
		}
		if ( farMesh != NULL )
		{
			b3DestroyMesh( farMesh );
		}
		b3DestroyMesh( nearMesh );
	}
	return 0;
}

static bool CheckParallelContinuousContactId( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3PreSolveData* data, void* context )
{
	(void)context;
	// Each isolated static/dynamic pair has its own capture. Narrow phase finishes before CCD,
	// and a fast body's CCD candidates are processed by one task, so no counter is shared by writers.
	ContinuousContactIdCapture* capture = b3Shape_GetUserData( shapeIdB );
	if ( data->phase == b3_preSolveContinuous )
	{
#if defined( _MSC_VER )
		static __declspec( thread ) int threadToken;
#else
		static _Thread_local int threadToken;
#endif
		capture->continuousThread = &threadToken;
	}
	return CheckContinuousContactId( shapeIdA, shapeIdB, data, capture );
}

static int TestParallelContinuousPreSolveContactId( void )
{
	// Exceeds the collide (20), CCD/finalize (16), and bullet CCD (8) task ranges by many blocks.
	enum { pairCount = 256 };
	b3Vec3 vertices[8] = {
		{ 1.0f, -5.0f, -5.0f }, { 1.0f, 5.0f, -5.0f }, { 1.0f, 5.0f, 5.0f }, { 1.0f, -5.0f, 5.0f },
		{ 0.0f, -5.0f, -5.0f }, { 0.0f, 5.0f, -5.0f }, { 0.0f, 5.0f, 5.0f }, { 0.0f, -5.0f, 5.0f },
	};
	int32_t indices[12] = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
	b3MeshDef meshDef = { .vertices = vertices, .indices = indices, .vertexCount = 8, .triangleCount = 4 };
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	ENSURE( mesh != NULL );
	b3Pos referencePositions[3][pairCount];
	b3Vec3 referenceVelocities[3][pairCount];
	// Each CCD path gets its own serial reference and four-worker run.
	for ( int run = 0; run < 4; ++run )
	{
		int parallel = run % 2;
		bool isBullet = run >= 2;
		b3WorldDef worldDef = b3DefaultWorldDef();
		worldDef.gravity = b3Vec3_zero;
		worldDef.workerCount = parallel != 0 ? 4 : 1;
		b3WorldId worldId = b3CreateWorld( &worldDef );
		ENSURE( b3World_GetWorkerCount( worldId ) == worldDef.workerCount );
		b3BodyId bodies[pairCount];
		ContinuousContactIdCapture captures[pairCount] = { 0 };
		for ( int i = 0; i < pairCount; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.position.z = 12.0f * i;
			b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
			b3ShapeDef shapeDef = b3DefaultShapeDef();
			b3ShapeId wallShapeId = b3CreateMeshShape( wallId, &shapeDef, mesh, b3Vec3_one );
			bodyDef.type = b3_dynamicBody;
			bodyDef.position.x = 1.255f;
			bodyDef.isBullet = isBullet;
			bodies[i] = b3CreateBody( worldId, &bodyDef );
			ENSURE( b3Body_IsBullet( bodies[i] ) == isBullet );
			shapeDef.enablePreSolveEvents = true;
			shapeDef.userData = captures + i;
			b3Sphere sphere = { b3Vec3_zero, 0.25f };
			b3ShapeId sphereShapeId = b3CreateSphereShape( bodies[i], &shapeDef, &sphere );
			captures[i] = (ContinuousContactIdCapture){
				.nearShapeId = wallShapeId,
				.farShapeId = wallShapeId,
				.sphereShapeId = sphereShapeId,
				.nearChildIndex = B3_NULL_INDEX,
				.farChildIndex = B3_NULL_INDEX,
				.marker = 1000000.0f + 4.0f * i,
				.maxPushSpeed = FLT_MAX,
				.contactDampingRatio = 1.0f,
				.farSharesContact = true,
				.expectFresh = true,
				.dataValid = true,
			};
		}
		b3World_SetPreSolveCallback( worldId, CheckParallelContinuousContactId, NULL );
		bool multipleCCDThreads = false;
		for ( int step = 0; step < 3; ++step )
		{
			for ( int i = 0; i < pairCount; ++i )
			{
				captures[i].discreteCount = 0;
				captures[i].nearCount = 0;
				captures[i].farCount = 0;
				captures[i].continuousThread = NULL;
				// Mix fresh approvals, rejections, and empty manifolds on the middle step, then re-approve.
				captures[i].discreteAction = step == 1 ? i % 3 : 0;
				captures[i].expectFresh = captures[i].discreteAction == 0;
				captures[i].marker += 1.0f;
				b3Body_SetTransform( bodies[i], (b3Pos){ 1.255f, 0.0f, 12.0f * i }, b3Quat_identity );
				b3Body_SetLinearVelocity( bodies[i], (b3Vec3){ -210.0f, 0.0f, 0.0f } );
			}
			b3World_Step( worldId, 1.0f / 60.0f, 1 );
			for ( int i = 0; i < pairCount; ++i )
			{
				ENSURE( captures[i].discreteCount == 1 && captures[i].nearCount > 0 && captures[i].farCount > 0 );
				ENSURE( captures[i].dataValid && captures[i].continuousThread != NULL );
				multipleCCDThreads = multipleCCDThreads || captures[i].continuousThread != captures[0].continuousThread;
				ENSURE( CheckFiniteContactBody( bodies[i] ) == 0 );
				b3Pos position = b3Body_GetPosition( bodies[i] );
				b3Vec3 velocity = b3Body_GetLinearVelocity( bodies[i] );
				ENSURE( 0.15f < position.x && position.x < 0.5f );
				if ( parallel == 0 )
				{
					referencePositions[step][i] = position;
					referenceVelocities[step][i] = velocity;
				}
				else
				{
					ENSURE( memcmp( &position, &referencePositions[step][i], sizeof( position ) ) == 0 );
					ENSURE( memcmp( &velocity, &referenceVelocities[step][i], sizeof( velocity ) ) == 0 );
				}
			}
		}
		// workerCount alone is not evidence that callbacks actually ran on more than one thread.
		ENSURE( multipleCCDThreads == ( parallel != 0 ) );
		b3DestroyWorld( worldId );
	}
	b3DestroyMesh( mesh );
	return 0;
}

static int TestContinuousPreSolveStaleContactId( void )
{
	b3Vec3 vertices[4] = {
		{ 1.0f, -5.0f, -5.0f }, { 1.0f, 5.0f, -5.0f }, { 1.0f, 5.0f, 5.0f }, { 1.0f, -5.0f, 5.0f },
	};
	int32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
	b3MeshDef meshDef = { .vertices = vertices, .indices = indices, .vertexCount = 4, .triangleCount = 2 };
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	ENSURE( mesh != NULL );
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = b3Vec3_zero;
	worldDef.workerCount = 1;
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3BodyDef bodyDef = b3DefaultBodyDef();
	b3BodyId staticBodyId = b3CreateBody( worldId, &bodyDef );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3ShapeId meshShapeId = b3CreateMeshShape( staticBodyId, &shapeDef, mesh, b3Vec3_one );
	bodyDef.type = b3_dynamicBody;
	bodyDef.position = (b3Pos){ 1.255f, 0.0f, 0.0f };
	b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { b3Vec3_zero, 0.25f };
	b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );
	ContinuousContactIdCapture capture = {
		.nearShapeId = meshShapeId,
		.farShapeId = meshShapeId,
		.sphereShapeId = sphereShapeId,
		.nearChildIndex = B3_NULL_INDEX,
		.farChildIndex = B3_NULL_INDEX,
		.marker = 1000000.0f,
		.maxPushSpeed = FLT_MAX,
		.contactDampingRatio = 1.0f,
		.expectFresh = true,
		.dataValid = true,
	};
	b3World_SetPreSolveCallback( worldId, CheckContinuousContactId, &capture );
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	ENSURE( capture.discreteCount == 1 && capture.dataValid );
	b3ContactId oldContactId = capture.discreteContactId;
	b3Body_SetAwake( sphereBodyId, false );
	ENSURE( b3Body_IsAwake( sphereBodyId ) == false );

	// A new impacting body wakes the sleeper after narrow phase has already collected its work.
	// Its old mesh contact is solved this step, but did not receive this step's discrete callback.
	bodyDef.position = (b3Pos){ 1.95f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -60.0f, 0.0f, 0.0f };
	b3BodyId strikerBodyId = b3CreateBody( worldId, &bodyDef );
	shapeDef.enablePreSolveEvents = false;
	sphere.radius = 0.5f;
	b3CreateSphereShape( strikerBodyId, &shapeDef, &sphere );
	capture.discreteCount = 0;
	capture.nearCount = 0;
	capture.expectFresh = false;
	b3World_Step( worldId, 1.0f / 60.0f, 1 );
	ENSURE( b3Body_IsAwake( sphereBodyId ) );
	ENSURE( capture.discreteCount == 0 );
	ENSURE( capture.nearCount > 0 );
	ENSURE( capture.dataValid );
	ENSURE( b3Contact_IsValid( oldContactId ) );
	b3ContactData oldContact = b3Contact_GetData( oldContactId );
	ENSURE( oldContact.manifoldCount > 0 && oldContact.manifolds[0].maxPushSpeed == FLT_MAX );
	ENSURE( oldContact.manifolds[0].contactDampingRatio == 1.0f );

	b3DestroyWorld( worldId );
	b3DestroyMesh( mesh );
	return 0;
}

static int TestPreSolveStaleManifoldTuning( void )
{
	b3Vec3 vertices[4] = {
		{ 1.0f, -5.0f, -5.0f }, { 1.0f, 5.0f, -5.0f }, { 1.0f, 5.0f, 5.0f }, { 1.0f, -5.0f, 5.0f },
	};
	int32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
	b3MeshDef meshDef = { .vertices = vertices, .indices = indices, .vertexCount = 4, .triangleCount = 2 };
	b3MeshData* mesh = b3CreateMesh( &meshDef, NULL, 0 );
	ENSURE( mesh != NULL );
	for ( int meshIndex = 0; meshIndex < 2; ++meshIndex )
	{
		// Independently expose stale speed, stale damping, and both together.
		for ( int tuning = 0; tuning < 3; ++tuning )
		{
			b3Pos positions[2];
			b3Vec3 velocities[2];
			float worldSpeed = tuning == 1 ? FLT_MAX : 0.03f;
			for ( int reference = 0; reference < 2; ++reference )
			{
				b3WorldDef worldDef = b3DefaultWorldDef();
				worldDef.gravity = b3Vec3_zero;
				worldDef.workerCount = 1;
				worldDef.contactDampingRatio = 1.0f;
				b3WorldId worldId = b3CreateWorld( &worldDef );
				b3BodyDef bodyDef = b3DefaultBodyDef();
				bodyDef.position.x = meshIndex == 0 ? 0.5f : 0.0f;
				b3BodyId staticBodyId = b3CreateBody( worldId, &bodyDef );
				b3ShapeDef shapeDef = b3DefaultShapeDef();
				b3BoxHull wall = b3MakeBoxHull( 0.5f, 5.0f, 5.0f );
				b3ShapeId wallShapeId = meshIndex == 0 ? b3CreateHullShape( staticBodyId, &shapeDef, &wall.base )
													 : b3CreateMeshShape( staticBodyId, &shapeDef, mesh, b3Vec3_one );

				bodyDef.type = b3_dynamicBody;
				bodyDef.position = (b3Pos){ 1.255f, 0.0f, 0.0f };
				b3BodyId sphereBodyId = b3CreateBody( worldId, &bodyDef );
				shapeDef.enablePreSolveEvents = true;
				b3Sphere sphere = { b3Vec3_zero, 0.25f };
				b3ShapeId sphereShapeId = b3CreateSphereShape( sphereBodyId, &shapeDef, &sphere );
				ContinuousContactIdCapture capture = {
					.nearShapeId = wallShapeId,
					.farShapeId = wallShapeId,
					.sphereShapeId = sphereShapeId,
					.nearChildIndex = B3_NULL_INDEX,
					.farChildIndex = B3_NULL_INDEX,
					.marker = 1000000.0f,
					.maxPushSpeed = reference == 0 && tuning != 1 ? 0.0f : worldSpeed,
					.contactDampingRatio = reference == 0 && tuning != 0 ? 1.0f : -1.0f,
					.dataValid = true,
				};
				b3World_SetPreSolveCallback( worldId, CheckContinuousContactId, &capture );
				// Store different tuning without solving, so both sleepers start from identical states.
				b3World_Step( worldId, 0.0f, 1 );
				ENSURE( capture.discreteCount == 1 && capture.dataValid );
				b3ContactId oldContactId = capture.discreteContactId;
				b3Body_SetAwake( sphereBodyId, false );
				ENSURE( b3Body_IsAwake( sphereBodyId ) == false );
				b3World_SetContactTuning( worldId, worldDef.contactHertz, 10.0f, worldSpeed );

				// As in the stale CCD-id fixture, this contact wakes the sleeper only after narrow phase.
				bodyDef.position = (b3Pos){ 1.95f, 0.0f, 0.0f };
				bodyDef.linearVelocity = (b3Vec3){ -60.0f, 0.0f, 0.0f };
				b3BodyId strikerBodyId = b3CreateBody( worldId, &bodyDef );
				shapeDef.enablePreSolveEvents = false;
				sphere.radius = 0.5f;
				b3CreateSphereShape( strikerBodyId, &shapeDef, &sphere );
				capture.discreteCount = 0;
				b3World_Step( worldId, 1.0f / 60.0f, 1 );
				ENSURE( b3Body_IsAwake( sphereBodyId ) );
				ENSURE( capture.discreteCount == 0 && capture.nearCount > 0 && capture.dataValid );
				ENSURE( b3Contact_IsValid( oldContactId ) );
				b3ContactData oldContact = b3Contact_GetData( oldContactId );
				ENSURE( oldContact.manifoldCount == 1 );
				ENSURE( oldContact.manifolds[0].maxPushSpeed == capture.maxPushSpeed );
				ENSURE( oldContact.manifolds[0].contactDampingRatio == capture.contactDampingRatio );
				ENSURE( oldContact.manifolds[0].points[0].maxNormalImpulse == capture.marker );
				positions[reference] = b3Body_GetPosition( sphereBodyId );
				velocities[reference] = b3Body_GetLinearVelocity( sphereBodyId );
				ENSURE( CheckFiniteContactBody( sphereBodyId ) == 0 );
				ENSURE( CheckFiniteContactBody( strikerBodyId ) == 0 );
				b3DestroyWorld( worldId );
			}
			// Old manifold fields remain observable, but cannot retune this step's constraints.
			ENSURE( positions[1].y > 0.0f );
			ENSURE( memcmp( positions, positions + 1, sizeof( positions[0] ) ) == 0 );
			ENSURE( memcmp( velocities, velocities + 1, sizeof( velocities[0] ) ) == 0 );
		}
	}
	b3DestroyMesh( mesh );
	return 0;
}

// Ensure correct move events from bodies involved in CCD and ensure a null pre-solve callback is safe.
static int TestContinuousMoveEvent( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );
	b3World_EnableContinuous( worldId, true );

	// Thin static wall, near face at x = 0.1
	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_staticBody;
	bodyDef.position = (b3Pos){ 0.0f, 0.0f, 0.0f };
	b3BodyId wallId = b3CreateBody( worldId, &bodyDef );
	b3BoxHull wallBox = b3MakeBoxHull( 0.1f, 5.0f, 5.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( wallId, &shapeDef, &wallBox.base );

	// Fast dynamic sphere fired at the wall.
	bodyDef = b3DefaultBodyDef();
	bodyDef.type = b3_dynamicBody;
	bodyDef.gravityScale = 0.0f;
	bodyDef.position = (b3Pos){ 3.0f, 0.0f, 0.0f };
	bodyDef.linearVelocity = (b3Vec3){ -30.0f, 0.0f, 0.0f };
	b3BodyId ballId = b3CreateBody( worldId, &bodyDef );
	shapeDef = b3DefaultShapeDef();
	shapeDef.density = 1.0f;
	shapeDef.enablePreSolveEvents = true;
	b3Sphere sphere = { { 0.0f, 0.0f, 0.0f }, 0.25f };
	b3CreateSphereShape( ballId, &shapeDef, &sphere );

	float timeStep = 1.0f / 60.0f;
	int subStepCount = 4;
	bool haveMove = false;

	for ( int step = 0; step < 30; ++step )
	{
		b3World_Step( worldId, timeStep, subStepCount );

		b3WorldTransform xf = b3Body_GetTransform( ballId );

		b3BodyEvents events = b3World_GetBodyEvents( worldId );
		for ( int i = 0; i < events.moveCount; ++i )
		{
			b3BodyMoveEvent* event = events.moveEvents + i;
			if ( B3_ID_EQUALS( event->bodyId, ballId ) == false )
			{
				continue;
			}

			haveMove = true;

			// The move event must carry the same pose the body reports, CCD rewind included
			ENSURE( event->transform.p.x == xf.p.x );
			ENSURE( event->transform.p.y == xf.p.y );
			ENSURE( event->transform.p.z == xf.p.z );
			ENSURE( event->transform.q.v.x == xf.q.v.x );
			ENSURE( event->transform.q.v.y == xf.q.v.y );
			ENSURE( event->transform.q.v.z == xf.q.v.z );
			ENSURE( event->transform.q.s == xf.q.s );
		}
	}

	ENSURE( haveMove == true );

	// Tunnel check
	b3Pos finalPos = b3Body_GetPosition( ballId );
	ENSURE( 0.2f < finalPos.x && finalPos.x < 0.8f );

	b3DestroyWorld( worldId );

	return 0;
}

int WorldTest( void )
{
	RUN_SUBTEST( HelloWorld );
	RUN_SUBTEST( EmptyWorld );
	RUN_SUBTEST( DestroyAllBodiesWorld );
	RUN_SUBTEST( TestIsValid );
	RUN_SUBTEST( TestWorldRecycle );
	RUN_SUBTEST( TestWorldCoverage );
	RUN_SUBTEST( TestPreSolveRejectsContact );
	RUN_SUBTEST( TestPreSolveDisablesAllPoints );
	RUN_SUBTEST( TestPreSolveMutableContact );
	RUN_SUBTEST( TestPreSolveMaxNormalImpulse );
	RUN_SUBTEST( TestPreSolveMaxPushSpeed );
	RUN_SUBTEST( TestPreSolveContactDampingRatio );
	RUN_SUBTEST( TestPreSolveDampingManifoldIsolation );
	RUN_SUBTEST( TestPreSolveLoadedTuningTransitions );
	RUN_SUBTEST( TestPreSolveOverflowTuning );
	RUN_SUBTEST( TestPreSolveZeroRestitution );
	RUN_SUBTEST( TestPreSolveFrictionAffectsSliding );
	RUN_SUBTEST( TestNullPreSolveCallbackUsesOrdinaryContacts );
	RUN_SUBTEST( TestPreSolveNoOpPreservesWarmStart );
	RUN_SUBTEST( TestPreSolveRunsEveryStep );
	RUN_SUBTEST( TestPreSolveEnableAfterContact );
	RUN_SUBTEST( TestPreSolveDisableAfterContact );
	RUN_SUBTEST( TestPreSolveDisableRebuildsGeometry );
	RUN_SUBTEST( TestPreSolveMeshAndHeightField );
	RUN_SUBTEST( TestContactMaterialInvalidIndices );
	RUN_SUBTEST( TestPreSolvePartialManifoldCompaction );
	RUN_SUBTEST( TestExplosion );
	RUN_SUBTEST( TestSensor );
	RUN_SUBTEST( TestContinuousPreSolveRejectsCandidate );
	RUN_SUBTEST( TestContinuousPreSolveAcceptsLaterTriangle );
	RUN_SUBTEST( TestContinuousPreSolveContactId );
	RUN_SUBTEST( TestParallelContinuousPreSolveContactId );
	RUN_SUBTEST( TestContinuousPreSolveStaleContactId );
	RUN_SUBTEST( TestPreSolveStaleManifoldTuning );
	RUN_SUBTEST( TestContinuousMoveEvent );
	RUN_SUBTEST( TestContactEvents );
	RUN_SUBTEST( TestHitEvents );
	RUN_SUBTEST( TestCompoundHitEvents );
	RUN_SUBTEST( TestCompoundContactMaterials );
	RUN_SUBTEST( TestOverflowColorPile );
	RUN_SUBTEST( SetBulletDriftTest );
	RUN_SUBTEST( EnableSleepFlagSyncTest );
	RUN_SUBTEST( EnableSleepNoopUnlockTest );
	RUN_SUBTEST( EnableContactRecyclingTest );
	RUN_SUBTEST( TestSetWorkerCount );
	RUN_SUBTEST( TestHullDatabase );

	return 0;
}
