#include "PhysicsObject.h"

#include "CKIpionManager.h"

#include "Convert.h"

#ifndef RAD2DEG
#define RAD2DEG(x) ((float)(x) * (float)(180.f / PI))
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((float)(x) * (float)(PI / 180.f))
#endif

#define IVP_MAX_SPEED (2000.f)
#define IVP_MAX_ROT_SPEED (DEG2RAD(360.f * 10.f))

PhysicsObject::PhysicsObject()
    : m_Behavior(NULL), m_RealObject(NULL), m_ContactData(NULL), m_GameData(NULL), m_GameFlags(0) {}

PhysicsObject::~PhysicsObject()
{
    if (m_ContactData)
    {
        delete m_ContactData;
        m_ContactData = NULL;
    }
}

void PhysicsObject::Init(IVP_Real_Object *obj, CK3dEntity *entity)
{
    assert(obj != NULL);
    assert(entity != NULL);

    obj->client_data = entity;
    m_RealObject = obj;
}

const char *PhysicsObject::GetName() const
{
    return m_RealObject->get_name();
}

CK3dEntity *PhysicsObject::GetEntity() const
{
    return (CK3dEntity *)m_RealObject->client_data;
}

IVP_Real_Object *PhysicsObject::GetObject() const
{
    return m_RealObject;
}

void *PhysicsObject::GetGameData() const
{
    return m_GameData;
}

void PhysicsObject::SetGameData(void *data)
{
    m_GameData = data;
}

unsigned int PhysicsObject::GetGameFlags() const
{
    return m_GameFlags;
}

void PhysicsObject::SetGameFlags(unsigned int flags)
{
    m_GameFlags = flags;
}

void PhysicsObject::Wake()
{
    m_RealObject->ensure_in_simulation();
}

void PhysicsObject::Sleep()
{
    m_RealObject->disable_simulation();
}

bool PhysicsObject::IsStatic() const
{
    if (m_RealObject->get_core()->physical_unmoveable)
        return true;
    return false;
}

bool PhysicsObject::IsMovable() const
{
    if (IsStatic() || !IsMotionEnabled())
        return false;
    return true;
}

bool PhysicsObject::IsCollisionEnabled() const
{
    return m_RealObject->is_collision_detection_enabled() != 0;
}

bool PhysicsObject::IsGravityEnabled() const
{
    if (!IsStatic())
    {
        return IsControlling(m_RealObject->get_core()->environment->get_gravity_controller());
    }

    return false;
}

bool PhysicsObject::IsMotionEnabled() const
{
    return m_RealObject->get_core()->pinned == IVP_FALSE;
}

bool PhysicsObject::IsControlling(IVP_Controller *controller) const
{
    IVP_Core *core = m_RealObject->get_core();
    for (int i = 0; i < core->controllers_of_core.len(); i++)
    {
        if (core->controllers_of_core.element_at(i) == controller)
            return true;
    }

    return false;
}

void PhysicsObject::EnableCollisions(bool enable)
{
    if (enable)
    {
        m_RealObject->enable_collision_detection(IVP_TRUE);
    }
    else
    {
        m_RealObject->enable_collision_detection(IVP_FALSE);
    }
}

void PhysicsObject::EnableGravity(bool enable)
{
    if (IsStatic())
        return;

    bool isEnabled = IsGravityEnabled();
    if (enable == isEnabled)
        return;

    IVP_Core *core = m_RealObject->get_core();
    IVP_Controller *pGravity = core->environment->get_gravity_controller();
    if (enable)
    {
        core->add_core_controller(pGravity);
    }
    else
    {
        core->rem_core_controller(pGravity);
    }
}

void PhysicsObject::EnableMotion(bool enable)
{
    bool isMovable = IsMotionEnabled();
    if (isMovable == enable)
        return;

    m_RealObject->set_pinned(enable ? IVP_FALSE : IVP_TRUE);

    RecheckCollisionFilter();
}

void PhysicsObject::RecheckCollisionFilter()
{
    m_RealObject->recheck_collision_filter();
}

float PhysicsObject::GetMass() const
{
    return m_RealObject->get_core()->get_mass();
}

float PhysicsObject::GetInvMass() const
{
    return m_RealObject->get_core()->get_inv_mass();
}

void PhysicsObject::SetMass(float mass)
{
    assert(mass > 0);
    m_RealObject->change_mass(mass);
}

void PhysicsObject::GetInertia(VxVector &inertia) const
{
    const IVP_U_Float_Point *pRI = m_RealObject->get_core()->get_rot_inertia();
    VxConvertVector(*pRI, inertia);
}

void PhysicsObject::GetInvInertia(VxVector &inertia) const
{
    const IVP_U_Float_Point *pRI = m_RealObject->get_core()->get_inv_rot_inertia();
    VxConvertVector(*pRI, inertia);
}

void PhysicsObject::SetInertia(const VxVector &inertia)
{
    IVP_U_Float_Point ri;
    VxConvertVector(inertia, ri);
    ri.k[0] = (float)IVP_Inline_Math::fabsd(ri.k[0]);
    ri.k[1] = (float)IVP_Inline_Math::fabsd(ri.k[1]);
    ri.k[2] = (float)IVP_Inline_Math::fabsd(ri.k[2]);
    m_RealObject->get_core()->set_rotation_inertia(&ri);
}

void PhysicsObject::GetDamping(float *speed, float *rot)
{
    IVP_Core *pCore = m_RealObject->get_core();
    if (speed)
    {
        *speed = pCore->speed_damp_factor;
    }
    if (rot)
    {
        *rot = pCore->rot_speed_damp_factor.k[0];
    }
}

void PhysicsObject::SetDamping(const float *speed, const float *rot)
{
    IVP_Core *pCore = m_RealObject->get_core();
    if (speed)
    {
        pCore->speed_damp_factor = *speed;
    }
    if (rot)
    {
        pCore->rot_speed_damp_factor.set(*rot, *rot, *rot);
    }
}

void PhysicsObject::ApplyForceCenter(const VxVector &forceVector)
{
    if (!IsMovable())
        return;

    IVP_U_Float_Point tmp;
    VxConvertVector(forceVector, tmp);

    IVP_Core *core = m_RealObject->get_core();
    tmp.mult(core->get_inv_mass());
    tmp.k[0] = IVP_Inline_Math::clamp(tmp.k[0], -IVP_MAX_SPEED, IVP_MAX_SPEED);
    tmp.k[1] = IVP_Inline_Math::clamp(tmp.k[1], -IVP_MAX_SPEED, IVP_MAX_SPEED);
    tmp.k[2] = IVP_Inline_Math::clamp(tmp.k[2], -IVP_MAX_SPEED, IVP_MAX_SPEED);
    m_RealObject->async_add_speed_object_ws(&tmp);
}

void PhysicsObject::ApplyForceOffset(const VxVector &forceVector, const VxVector &worldPosition)
{
    if (!IsMovable())
        return;

    IVP_U_Float_Point force;
    VxConvertVector(forceVector, force);

    IVP_U_Point pos;
    VxConvertVector(worldPosition, pos);

    IVP_Core *core = m_RealObject->get_core();
    core->async_push_core_ws(&pos, &force);
    core->speed_change.k[0] = IVP_Inline_Math::clamp(core->speed_change.k[0], -IVP_MAX_SPEED, IVP_MAX_SPEED);
    core->speed_change.k[1] = IVP_Inline_Math::clamp(core->speed_change.k[1], -IVP_MAX_SPEED, IVP_MAX_SPEED);
    core->speed_change.k[2] = IVP_Inline_Math::clamp(core->speed_change.k[2], -IVP_MAX_SPEED, IVP_MAX_SPEED);

    core->rot_speed_change.k[0] = IVP_Inline_Math::clamp(core->rot_speed_change.k[0], -IVP_MAX_ROT_SPEED, IVP_MAX_ROT_SPEED);
    core->rot_speed_change.k[1] = IVP_Inline_Math::clamp(core->rot_speed_change.k[1], -IVP_MAX_ROT_SPEED, IVP_MAX_ROT_SPEED);
    core->rot_speed_change.k[2] = IVP_Inline_Math::clamp(core->rot_speed_change.k[2], -IVP_MAX_ROT_SPEED, IVP_MAX_ROT_SPEED);
    Wake();
}

void PhysicsObject::CalculateForceOffset(const VxVector &forceVector, const VxVector &worldPosition,
                                         VxVector &centerForce, VxVector &centerTorque)
{
    IVP_U_Float_Point force;
    VxConvertVector(forceVector, force);

    IVP_U_Point pos;
    VxConvertVector(worldPosition, pos);

    IVP_Core *core = m_RealObject->get_core();

    const IVP_U_Matrix *m_world_f_core = core->get_m_world_f_core_PSI();

    IVP_U_Float_Point point_d_ws;
    point_d_ws.subtract(&pos, m_world_f_core->get_position());

    IVP_U_Float_Point cross_point_dir;

    cross_point_dir.calc_cross_product(&point_d_ws, &force);
    m_world_f_core->inline_vimult3(&cross_point_dir, &cross_point_dir);

    VxConvertVector(cross_point_dir, centerTorque);
    VxConvertVector(force, centerForce);
}

void PhysicsObject::CalculateVelocityOffset(const VxVector &forceVector, const VxVector &worldPosition,
                                            VxVector &centerVelocity, VxVector &centerAngularVelocity)
{
    IVP_U_Float_Point force;
    VxConvertVector(forceVector, force);

    IVP_U_Point pos;
    VxConvertVector(worldPosition, pos);

    IVP_Core *core = m_RealObject->get_core();

    const IVP_U_Matrix *m_world_f_core = core->get_m_world_f_core_PSI();

    IVP_U_Float_Point point_d_ws;
    point_d_ws.subtract(&pos, m_world_f_core->get_position());

    IVP_U_Float_Point cross_point_dir;

    cross_point_dir.calc_cross_product(&point_d_ws, &force);
    m_world_f_core->inline_vimult3(&cross_point_dir, &cross_point_dir);

    cross_point_dir.set_pairwise_mult(&cross_point_dir, core->get_inv_rot_inertia());
    VxConvertVector(cross_point_dir, centerAngularVelocity);

    force.set_multiple(&force, core->get_inv_mass());
    VxConvertVector(force, centerVelocity);
}

void PhysicsObject::ApplyTorqueCenter(const VxVector &torqueImpulse)
{
    if (!IsMovable())
        return;

    IVP_U_Float_Point torque;
    VxConvertVector(torqueImpulse, torque);

    IVP_Core *core = m_RealObject->get_core();
    core->async_rot_push_core_multiple_ws(&torque, 1.0);
    core->rot_speed_change.k[0] = IVP_Inline_Math::clamp(core->rot_speed_change.k[0], -IVP_MAX_ROT_SPEED, IVP_MAX_ROT_SPEED);
    core->rot_speed_change.k[1] = IVP_Inline_Math::clamp(core->rot_speed_change.k[1], -IVP_MAX_ROT_SPEED, IVP_MAX_ROT_SPEED);
    core->rot_speed_change.k[2] = IVP_Inline_Math::clamp(core->rot_speed_change.k[2], -IVP_MAX_ROT_SPEED, IVP_MAX_ROT_SPEED);
    Wake();
}

void PhysicsObject::GetPosition(VxVector *worldPosition, VxVector *angles)
{
    IVP_U_Matrix matrix;
    m_RealObject->get_m_world_f_object_AT(&matrix);

    if (worldPosition)
    {
        IVP_U_Point *pt = matrix.get_position();
        worldPosition->Set((float)pt->k[0], (float)pt->k[1], (float)pt->k[2]);
    }

    if (angles)
    {
        matrix.get_angles(&angles->x, &angles->y, &angles->z);
    }
}

void PhysicsObject::GetPositionMatrix(VxMatrix &positionMatrix)
{
    IVP_U_Matrix matrix;
    m_RealObject->get_m_world_f_object_AT(&matrix);
    VxConvertMatrix(matrix, positionMatrix);
}

void PhysicsObject::SetPosition(const VxVector &worldPosition, const VxVector &angles, bool isTeleport)
{
    IVP_U_Point pos;
    VxConvertVector(worldPosition, pos);

    VxMatrix mat;
    Vx3DMatrixFromEulerAngles(mat, angles.x, angles.y, angles.z);

    IVP_U_Matrix3 tmp;
    VxConvertMatrix(mat, tmp);

    IVP_U_Quat rot;
    rot.set_quaternion(&tmp);

    if (m_RealObject->is_collision_detection_enabled() && isTeleport)
    {
        EnableCollisions(false);
        m_RealObject->beam_object_to_new_position(&rot, &pos, IVP_FALSE);
        EnableCollisions(true);
    }
    else
    {
        m_RealObject->beam_object_to_new_position(&rot, &pos, IVP_FALSE);
    }
}

void PhysicsObject::SetPositionMatrix(const VxMatrix &matrix, bool isTeleport)
{
    IVP_U_Quat rot;
    IVP_U_Matrix mat;

    VxConvertMatrix(matrix, mat);
    rot.set_quaternion(&mat);

    if (m_RealObject->is_collision_detection_enabled() && isTeleport)
    {
        EnableCollisions(false);
        m_RealObject->beam_object_to_new_position(&rot, &mat.vv, IVP_FALSE);
        EnableCollisions(true);
    }
    else
    {
        m_RealObject->beam_object_to_new_position(&rot, &mat.vv, IVP_FALSE);
    }
}

void PhysicsObject::GetVelocity(VxVector *velocity, VxVector *angularVelocity)
{
    if (!velocity && !angularVelocity)
        return;

    IVP_Core *core = m_RealObject->get_core();

    if (velocity)
    {
        IVP_U_Float_Point speed;
        speed.add(&core->speed, &core->speed_change);
        VxConvertVector(speed, *velocity);
    }

    if (angularVelocity)
    {
        IVP_U_Float_Point rotSpeed;
        rotSpeed.add(&core->rot_speed, &core->rot_speed_change);
        VxConvertVector(rotSpeed, *angularVelocity);
    }
}

void PhysicsObject::GetVelocityAtPoint(const VxVector &worldPosition, VxVector &velocity)
{
    IVP_Core *core = m_RealObject->get_core();

    IVP_U_Point pos;
    VxConvertVector(worldPosition, pos);

    IVP_U_Float_Point rotSpeed;
    rotSpeed.add(&core->rot_speed, &core->rot_speed_change);

    IVP_U_Float_Point av_ws;
    core->get_m_world_f_core_PSI()->vmult3(&rotSpeed, &av_ws);

    IVP_U_Float_Point pos_rel;
    pos_rel.subtract(&pos, core->get_position_PSI());
    IVP_U_Float_Point cross;
    cross.inline_calc_cross_product(&av_ws, &pos_rel);

    IVP_U_Float_Point speed;
    speed.add(&core->speed, &cross);
    speed.add(&core->speed_change);

    VxConvertVector(speed, velocity);
}

void PhysicsObject::SetVelocity(const VxVector *velocity, const VxVector *angularVelocity)
{
    if (!IsMovable())
        return;

    Wake();

    IVP_Core *core = m_RealObject->get_core();

    if (velocity)
    {
        VxConvertVector(*velocity, core->speed_change);
        core->speed.set_to_zero();
    }

    if (angularVelocity)
    {
        VxConvertVector(*angularVelocity, core->rot_speed_change);
        core->rot_speed.set_to_zero();
    }
}

void PhysicsObject::AddVelocity(const VxVector *velocity, const VxVector *angularVelocity)
{
    if (!IsMovable())
        return;

    Wake();

    IVP_Core *core = m_RealObject->get_core();

    if (velocity)
    {
        IVP_U_Float_Point ivpVelocity;
        VxConvertVector(*velocity, ivpVelocity);
        core->speed_change.add(&ivpVelocity);
    }

    if (angularVelocity)
    {
        IVP_U_Float_Point ivpAngularVelocity;
        VxConvertVector(*angularVelocity, ivpAngularVelocity);

        core->rot_speed_change.add(&ivpAngularVelocity);
    }
}

float PhysicsObject::GetEnergy()
{
    IVP_Core *core = m_RealObject->get_core();
    IVP_DOUBLE energy;
    IVP_U_Float_Point tmp;

    energy = 0.5 * core->get_mass() * core->speed.dot_product(&core->speed); // 1/2mvv
    tmp.set_pairwise_mult(&core->rot_speed, core->get_rot_inertia());        // wI
    energy += 0.5 * tmp.dot_product(&core->rot_speed);                       // 1/2mvv + 1/2wIw

    return (float)energy;
}