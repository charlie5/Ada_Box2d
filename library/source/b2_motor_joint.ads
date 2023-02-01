#include "b2_api.h"
#include "b2_joint.h"

/// Motor joint definition.
struct B2_API b2MotorJointDef : public b2JointDef
{
	b2MotorJointDef()
	{
		type = e_motorJoint;
		linearOffset.SetZero();
		angularOffset = 0.0f;
		maxForce = 1.0f;
		maxTorque = 1.0f;
		correctionFactor = 0.3f;
	}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body* bodyA, b2Body* bodyB);

	/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
	b2Vec2 linearOffset;

	/// The bodyB angle minus bodyA angle in radians.
	float angularOffset;

	/// The maximum motor force in N.
	float maxForce;

	/// The maximum motor torque in N-m.
	float maxTorque;

	/// Position correction factor in the range [0,1].
	float correctionFactor;
};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
class B2_API b2MotorJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const override;
	b2Vec2 GetAnchorB() const override;

	b2Vec2 GetReactionForce(float inv_dt) const override;
	float GetReactionTorque(float inv_dt) const override;

	/// Set/get the target linear offset, in frame A, in meters.
	void SetLinearOffset(const b2Vec2& linearOffset);
	const b2Vec2& GetLinearOffset() const;

	/// Set/get the target angular offset, in radians.
	void SetAngularOffset(float angularOffset);
	float GetAngularOffset() const;

	/// Set the maximum friction force in N.
	void SetMaxForce(float force);

	/// Get the maximum friction force in N.
	float GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float torque);

	/// Get the maximum friction torque in N*m.
	float GetMaxTorque() const;

	/// Set the position correction factor in the range [0,1].
	void SetCorrectionFactor(float factor);

	/// Get the position correction factor in the range [0,1].
	float GetCorrectionFactor() const;

	/// Dump to b2Log
	void Dump() override;

protected:

	friend class b2Joint;

	b2MotorJoint(const b2MotorJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data) override;
	void SolveVelocityConstraints(const b2SolverData& data) override;
	bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	b2Vec2 m_linearOffset;
	float m_angularOffset;
	b2Vec2 m_linearImpulse;
	float m_angularImpulse;
	float m_maxForce;
	float m_maxTorque;
	float m_correctionFactor;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	b2Vec2 m_linearError;
	float m_angularError;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	b2Mat22 m_linearMass;
	float m_angularMass;
};

