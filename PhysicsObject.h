#ifndef PHYSICS_RT_PHYSICSOBJECT_H
#define PHYSICS_RT_PHYSICSOBJECT_H

#include "CK3dEntity.h"

#include "ivp_physics.hxx"
#include "ivp_real_object.hxx"

class PhysicsContactData;

class IPhysicsObject
{
public:
    virtual const char *GetName() const = 0;
    virtual CK3dEntity *GetEntity() const = 0;

    virtual void SetGameData(void *data) = 0;
    virtual void *GetGameData() const = 0;

    virtual void SetGameFlags(unsigned int flags) = 0;
    virtual unsigned int GetGameFlags() const = 0;

    virtual void Wake() = 0;
    virtual void Sleep() = 0;

    virtual bool IsStatic() const = 0;
    virtual bool IsMovable() const = 0;
    virtual bool IsCollisionEnabled() const = 0;
    virtual bool IsGravityEnabled() const = 0;
    virtual bool IsMotionEnabled() const = 0;

    virtual void EnableCollisions(bool enable) = 0;
    virtual void EnableGravity(bool enable) = 0;
    virtual void EnableMotion(bool enable) = 0;

    virtual void RecheckCollisionFilter() = 0;

    virtual float GetMass() const = 0;
    virtual float GetInvMass() const = 0;
    virtual void SetMass(float mass) = 0;

    virtual void GetInertia(VxVector &inertia) const = 0;
    virtual void GetInvInertia(VxVector &inertia) const = 0;
    virtual void SetInertia(const VxVector &inertia) = 0;

    virtual void GetDamping(float *speed, float *rot) = 0;
    virtual void SetDamping(const float *speed, const float *rot) = 0;

    virtual void ApplyForceCenter(const VxVector &forceVector) = 0;
    virtual void ApplyForceOffset(const VxVector &forceVector, const VxVector &worldPosition) = 0;
    virtual void ApplyTorqueCenter(const VxVector &torqueImpulse) = 0;

    virtual void CalculateForceOffset(const VxVector &forceVector, const VxVector &worldPosition, VxVector &centerForce, VxVector &centerTorque) = 0;
    virtual void CalculateVelocityOffset(const VxVector &forceVector, const VxVector &worldPosition, VxVector &centerVelocity, VxVector &centerAngularVelocity) = 0;

    virtual void GetPosition(VxVector *worldPosition, VxVector *angles) = 0;
    virtual void GetPositionMatrix(VxMatrix &positionMatrix) = 0;

    virtual void SetPosition(const VxVector &worldPosition, const VxVector &angles, bool isTeleport) = 0;
    virtual void SetPositionMatrix(const VxMatrix &matrix, bool isTeleport) = 0;

    virtual void GetVelocity(VxVector *velocity, VxVector *angularVelocity) = 0;
    virtual void GetVelocityAtPoint(const VxVector &worldPosition, VxVector &velocity) = 0;
    virtual void SetVelocity(const VxVector *velocity, const VxVector *angularVelocity) = 0;
    virtual void AddVelocity(const VxVector *velocity, const VxVector *angularVelocity) = 0;

    virtual float GetEnergy() = 0;
};

class PhysicsObject : public IPhysicsObject
{
public:
    PhysicsObject();
    ~PhysicsObject();

    void Init(IVP_Real_Object *obj, CK3dEntity *entity);

    virtual const char *GetName() const;
    virtual CK3dEntity *GetEntity() const;

    IVP_Real_Object *GetObject() const;

    virtual void *GetGameData() const;
    virtual void SetGameData(void *data);

    virtual unsigned int GetGameFlags() const;
    virtual void SetGameFlags(unsigned int flags);

    virtual void Wake();
    virtual void Sleep();

    virtual bool IsStatic() const;
    virtual bool IsMovable() const;
    virtual bool IsCollisionEnabled() const;
    virtual bool IsGravityEnabled() const;
    virtual bool IsMotionEnabled() const;
    bool IsControlling(IVP_Controller *controller) const;

    virtual void EnableCollisions(bool enable);
    virtual void EnableGravity(bool enable);
    virtual void EnableMotion(bool enable);

    virtual void RecheckCollisionFilter();

    virtual float GetMass() const;
    virtual float GetInvMass() const;
    virtual void SetMass(float mass);

    virtual void GetInertia(VxVector &inertia) const;
    virtual void GetInvInertia(VxVector &inertia) const;
    virtual void SetInertia(const VxVector &inertia);

    virtual void GetDamping(float *speed, float *rot);
    virtual void SetDamping(const float *speed, const float *rot);

    virtual void ApplyForceCenter(const VxVector &forceVector);
    virtual void ApplyForceOffset(const VxVector &forceVector, const VxVector &worldPosition);
    virtual void ApplyTorqueCenter(const VxVector &torqueImpulse);

    virtual void CalculateForceOffset(const VxVector &forceVector, const VxVector &worldPosition, VxVector &centerForce, VxVector &centerTorque);
    virtual void CalculateVelocityOffset(const VxVector &forceVector, const VxVector &worldPosition, VxVector &centerVelocity, VxVector &centerAngularVelocity);

    virtual void GetPosition(VxVector *worldPosition, VxVector *angles);
    virtual void GetPositionMatrix(VxMatrix &positionMatrix);

    virtual void SetPosition(const VxVector &worldPosition, const VxVector &angles, bool isTeleport);
    virtual void SetPositionMatrix(const VxMatrix &matrix, bool isTeleport);

    virtual void GetVelocity(VxVector *velocity, VxVector *angularVelocity);
    virtual void GetVelocityAtPoint(const VxVector &worldPosition, VxVector &velocity);
    virtual void SetVelocity(const VxVector *velocity, const VxVector *angularVelocity);
    virtual void AddVelocity(const VxVector *velocity, const VxVector *angularVelocity);

    virtual float GetEnergy();

    CKBehavior *m_Behavior;
    IVP_Real_Object *m_RealObject;
    PhysicsContactData *m_ContactData;
    void *m_GameData;
    unsigned int m_GameFlags;
};

#endif // PHYSICS_RT_PHYSICSOBJECT_H
