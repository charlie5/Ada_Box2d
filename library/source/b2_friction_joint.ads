#include "b2_api.h"
#include "b2_joint.h"

/// Friction joint definition.
struct B2_API b2FrictionJointDef : public b2JointDef
{
	b2FrictionJointDef()
	{
		type = e_frictionJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		maxForce = 0.0f;
		maxTorque = 0.0f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The maximum friction force in N.
	float maxForce;

	/// The maximum friction torque in N-m.
	float maxTorque;
};

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class B2_API b2FrictionJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(float inv_dt) const override;
	float GetReactionTorque(float inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set the maximum friction force in N.
	void SetMaxForce(float force);

	/// Get the maximum friction force in N.
	float GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float torque);

	/// Get the maximum friction torque in N*m.
	float GetMaxTorque() const;

	/// Dump joint to dmLog
	void Dump() override;

protected:

	friend class b2Joint;

	b2FrictionJoint(const b2FrictionJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;

	// Solver shared
	b2Vec2 m_linearImpulse;
	float m_angularImpulse;
	float m_maxForce;
	float m_maxTorque;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	b2Mat22 m_linearMass;
	float m_angularMass;
};

