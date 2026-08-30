#ifndef __RBI_BOX3D_API_H__
#define __RBI_BOX3D_API_H__

// Fix for legacy MSVC environments failing on C11 compile-time assertions
#if defined(_MSC_VER) && !defined(__cplusplus) && !defined(_Static_assert)
    #define _Static_assert(expr, msg) typedef char __static_assert_t[(expr) ? 1 : -1]
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque pointer types wrapping Box3D internal types */
typedef struct rb3dDynamicsWorld rb3dDynamicsWorld;
typedef struct rb3dRigidBody rb3dRigidBody;
typedef struct rb3dShape rb3dShape;

/* World Management */
rb3dDynamicsWorld *rb3dNewDynamicsWorld(void);
void rb3dDeleteDynamicsWorld(rb3dDynamicsWorld *world);
void rb3dStepSimulation(rb3dDynamicsWorld *world, float timeStep, int maxSubSteps, float fixedTimeStep);
void rb3dSetGravity(rb3dDynamicsWorld *world, const float gravity[3]);

/* Rigid Body Management */
rb3dRigidBody *rb3dCreateRigidBody(rb3dDynamicsWorld *world, float mass, const float loc[3], const float rot[4]);
void rb3dDeleteRigidBody(rb3dDynamicsWorld *world, rb3dRigidBody *body);
void rb3dAddRigidBody(rb3dDynamicsWorld *world, rb3dRigidBody *body);
void rb3dRemoveRigidBody(rb3dDynamicsWorld *world, rb3dRigidBody *body);

/* Shape Management */
rb3dShape *rb3dCreateBoxShape(float halfX, float halfY, float halfZ);
void rb3dDeleteShape(rb3dShape *shape);
void rb3dSetShape(rb3dRigidBody *body, rb3dShape *shape);

/* Data Sync Tools */
void rb3dGetTransform(rb3dRigidBody *body, float loc[3], float rot[4]);

#ifdef __cplusplus
}
#endif

#endif /* __RBI_BOX3D_API_H__ */
