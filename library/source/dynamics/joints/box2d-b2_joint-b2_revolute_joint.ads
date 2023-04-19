with
     box2d.b2_Body;


package box2d.b2_Joint.b2_revolute_Joint
--
--  A revolute joint constrains two bodies to share a common point while they
--  are free to rotate about the point. The relative rotation about the shared
--  point is the joint angle. You can limit the relative rotation with
--  a joint limit that specifies a lower and upper angle. You can use a motor
--  to drive the relative rotation about the shared point. A maximum motor torque
--  is provided so that infinite forces are not generated.
--
is
   use b2_Body;


   ----------------------
   --- b2RevoluteJointDef
   --

   --  Revolute joint definition. This requires defining an anchor point where the
   --  bodies are joined. The definition uses local anchor points so that the
   --  initial configuration can violate the constraint slightly. You also need to
   --  specify the initial relative angle for joint limits. This helps when saving
   --  and loading a game.
   --
   --  The local anchor points are measured from the body's origin
   --  rather than the center of mass because:
   --
   --  1. you might not know where the center of mass will be.
   --  2. if you add/remove shapes from a body and recompute the mass,
   --     the joints will be broken.


   --  struct b2RevoluteJointDef : public b2JointDef
   --  {
   --    The local anchor point relative to bodyA's origin.
   --    b2Vec2 localAnchorA;
   --
   --    The local anchor point relative to bodyB's origin.
   --    b2Vec2 localAnchorB;
   --
   --    The bodyB angle minus bodyA angle in the reference state (radians).
   --    float referenceAngle;
   --
   --    A flag to enable joint limits.
   --    bool enableLimit;
   --
   --    The lower angle for the joint limit (radians).
   --    float lowerAngle;
   --
   --    The upper angle for the joint limit (radians).
   --    float upperAngle;
   --
   --    A flag to enable the joint motor.
   --    bool enableMotor;
   --
   --    The desired motor speed. Usually in radians per second.
   --    float motorSpeed;
   --
   --    The maximum motor torque used to achieve the desired motor speed.
   --    Usually in N-m.
   --    float maxMotorTorque;
   --  };
   --

   type b2RevoluteJointDef is new b2_Joint.b2JointDef with
      record
         localAnchorA   : b2Vec2;      -- The local anchor point relative to bodyA's origin.
         localAnchorB   : b2Vec2;      -- The local anchor point relative to bodyB's origin.

         referenceAngle : Real;        -- The bodyB angle minus bodyA angle in the reference state (radians).
         enableLimit    : Boolean;     -- A flag to enable joint limits.

         lowerAngle     : Real;        -- The lower angle for the joint limit (radians).
         upperAngle     : Real;        -- The upper angle for the joint limit (radians).

         enableMotor    : Boolean;     -- A flag to enable the joint motor.
         motorSpeed     : Real;        -- The desired motor speed. Usually in radians per second.
         maxMotorTorque : Real;        -- The maximum motor torque used to achieve the desired motor speed.
      end record;


   --    b2RevoluteJointDef()
   --    {
   --       type = e_revoluteJoint;
   --       localAnchorA.Set(0.0f, 0.0f);
   --       localAnchorB.Set(0.0f, 0.0f);
   --       referenceAngle = 0.0f;
   --       lowerAngle = 0.0f;
   --       upperAngle = 0.0f;
   --       maxMotorTorque = 0.0f;
   --       motorSpeed = 0.0f;
   --       enableLimit = false;
   --       enableMotor = false;
   --    }
   --

   function  to_b2RevoluteJointDef return b2RevoluteJointDef;



   --    Initialize the bodies, anchors, and reference angle using a world
   --    anchor point.
   --
   --    void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);
   --

   procedure initialize (Self : out b2RevoluteJointDef;   bodyA, bodyB   : access b2Body;
                                                          Anchor         : in     b2Vec2);






   -------------------
   --- b2RevoluteJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2RevoluteJoint     is new b2_Joint.b2Joint with private;
   type b2RevoluteJoint_ptr is access all b2RevoluteJoint'Class;



   --  class b2RevoluteJoint : public b2Joint
   --  {
   --  public:
   --
   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2RevoluteJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2RevoluteJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2RevoluteJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2RevoluteJoint;   inv_dt : in Real) return Real;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2RevoluteJoint) return b2Vec2
     with inline;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2RevoluteJoint) return b2Vec2
     with inline;



   --    Get the reference angle.
   --
   --    float GetReferenceAngle() const { return m_referenceAngle; }
   --

   function getReferenceAngle (Self : in b2RevoluteJoint) return Real
     with inline;



   --    Get the current joint angle in radians.
   --
   --    float GetJointAngle() const;
   --

   function getJointAngle (Self : in b2RevoluteJoint) return Real;



   --    Get the current joint angle speed in radians per second.
   --
   --    float GetJointSpeed() const;
   --

   function getJointSpeed (Self : in b2RevoluteJoint) return Real;



   --    Is the joint limit enabled?
   --
   --    bool IsLimitEnabled() const;
   --

   function isLimitEnabled (Self : in b2RevoluteJoint) return Boolean;



   --    Enable/disable the joint limit.
   --
   --    void EnableLimit(bool flag);
   --

   procedure enableLimit (Self : in out b2RevoluteJoint;   Flag : in Boolean);



   --    Get the lower joint limit in radians.
   --
   --    float GetLowerLimit() const;
   --

   function getLowerLimit (Self : in b2RevoluteJoint) return Real;



   --    Get the upper joint limit in radians.
   --
   --    float GetUpperLimit() const;
   --

   function getUpperLimit (Self : in b2RevoluteJoint) return Real;



   --    Set the joint limits in radians.
   --
   --    void SetLimits(float lower, float upper);
   --

   procedure setLimits (Self : in out b2RevoluteJoint;   Lower, Upper : in Real);



   --    Is the joint motor enabled?
   --
   --    bool IsMotorEnabled() const;
   --

   function isMotorEnabled (Self : in b2RevoluteJoint) return Boolean;



   --    Enable/disable the joint motor.
   --
   --    void EnableMotor(bool flag);
   --

   procedure enableMotor (Self : in out b2RevoluteJoint;   Flag : in Boolean);



   --    Set the motor speed in radians per second.
   --
   --    void SetMotorSpeed(float speed);
   --

   procedure setMotorSpeed (Self : in out b2RevoluteJoint;   Speed : in Real);



   --    Get the motor speed in radians per second.
   --
   --    inline float b2RevoluteJoint::GetMotorSpeed() const
   --    {
   --      return m_motorSpeed;
   --    }
   --

   function getMotorSpeed (Self : in b2RevoluteJoint) return Real
     with inline;



   --    Set the maximum motor torque, usually in N-m.
   --
   --    void SetMaxMotorTorque(float torque);
   --

   procedure setMaxMotorTorque (Self : in out b2RevoluteJoint;   Torque : in Real);



   --    float GetMaxMotorTorque() const { return m_maxMotorTorque; }
   --

   function getMaxMotorTorque (Self : in b2RevoluteJoint) return Real;



   --    Get the current motor torque given the inverse time step.
   --    Unit is N*m.
   --
   --    float GetMotorTorque(float inv_dt) const;
   --

   function getMotorTorque (Self : in b2RevoluteJoint;   inv_dt : Real) return Real;



   --    void Draw(b2Draw* draw) const override;
   --

   overriding
   procedure draw (Self : in b2RevoluteJoint;   Draw : access b2Draw'Class);



   overriding
   procedure dump (Self : in b2RevoluteJoint);




   --------------
   --  protected:
   --
   --    friend class b2Joint;
   --    friend class b2GearJoint;
   --
   --    b2RevoluteJoint(const b2RevoluteJointDef* def);
   --

   package Forge
   is

      --    b2DistanceJoint(const b2DistanceJointDef* data);
      --

      function to_b2RevoluteJoint (Def : in b2RevoluteJointDef) return b2RevoluteJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2RevoluteJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2RevoluteJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2RevoluteJoint;   Data : in b2SolverData) return Boolean;




   --    // Solver shared
   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    b2Vec2 m_impulse;
   --    float m_motorImpulse;
   --    float m_lowerImpulse;
   --    float m_upperImpulse;
   --    bool m_enableMotor;
   --    float m_maxMotorTorque;
   --    float m_motorSpeed;
   --    bool m_enableLimit;
   --    float m_referenceAngle;
   --    float m_lowerAngle;
   --    float m_upperAngle;
   --
   --    // Solver temp
   --    int32 m_indexA;
   --    int32 m_indexB;
   --    b2Vec2 m_rA;
   --    b2Vec2 m_rB;
   --    b2Vec2 m_localCenterA;
   --    b2Vec2 m_localCenterB;
   --    float m_invMassA;
   --    float m_invMassB;
   --    float m_invIA;
   --    float m_invIB;
   --    b2Mat22 m_K;
   --    float m_angle;
   --    float m_axialMass;


   type b2RevoluteJoint is new b2_Joint.b2Joint with
      record
         -- Solver shared.
         --
         m_localAnchorA,
         m_localAnchorB   : b2Vec2;
         m_impulse        : b2Vec2;
         m_motorImpulse,
         m_lowerImpulse,
         m_upperImpulse   : Real;

         m_enableMotor    : Boolean;
         m_maxMotorTorque,
         m_motorSpeed     : Real;
         m_enableLimit    : Boolean;

         m_referenceAngle : Real;
         m_lowerAngle,
         m_upperAngle     : Real;


         -- Solver temp.
         --
         m_indexA,
         m_indexB       : Natural;

         m_rA,
         m_rB           : b2Vec2;

         m_localCenterA,
         m_localCenterB : b2Vec2;

         m_invMassA,
         m_invMassB     : Real;

         m_invIA,
         m_invIB        : Real;

         m_K            : b2Mat22;

         m_angle        : Real;
         m_axialMass    : Real;
      end record;


end box2d.b2_Joint.b2_revolute_Joint;
