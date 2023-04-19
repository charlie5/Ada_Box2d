with
     box2d.b2_Body;


package box2d.b2_Joint.b2_prismatic_Joint
--
--  A prismatic joint. This joint provides one degree of freedom: translation
--  along an axis fixed in bodyA. Relative rotation is prevented. You can
--  use a joint limit to restrict the range of motion and a joint motor to
--  drive the motion or to model joint friction.
--
is
   use box2d.b2_Body;

   -----------------------
   --- b2PrismaticJointDef
   --

   --  Prismatic joint definition. This requires defining a line of
   --  motion using an axis and an anchor point. The definition uses local
   --  anchor points and a local axis so that the initial configuration
   --  can violate the constraint slightly. The joint translation is zero
   --  when the local anchor points coincide in world space. Using local
   --  anchors and a local axis helps when saving and loading a game.
   --  struct b2PrismaticJointDef : public b2JointDef
   --

   --  {
   --    The local anchor point relative to bodyA's origin.
   --    b2Vec2 localAnchorA;
   --
   --    The local anchor point relative to bodyB's origin.
   --    b2Vec2 localAnchorB;
   --
   --    The local translation unit axis in bodyA.
   --    b2Vec2 localAxisA;
   --
   --    The constrained angle between the bodies: bodyB_angle - bodyA_angle.
   --    float referenceAngle;
   --
   --    Enable/disable the joint limit.
   --    bool enableLimit;
   --
   --    The lower translation limit, usually in meters.
   --    float lowerTranslation;
   --
   --    The upper translation limit, usually in meters.
   --    float upperTranslation;
   --
   --    Enable/disable the joint motor.
   --    bool enableMotor;
   --
   --    The maximum motor torque, usually in N-m.
   --    float maxMotorForce;
   --
   --    The desired motor speed in radians per second.
   --    float motorSpeed;
   --  };
   --

   type b2PrismaticJointDef is new b2_Joint.b2JointDef with
      record
         localAnchorA      : b2Vec2;      -- The local anchor point relative to bodyA's origin.
         localAnchorB      : b2Vec2;      -- The local anchor point relative to bodyB's origin.

         localAxisA        : b2Vec2;      -- The local translation unit axis in bodyA.
         referenceAngle    : Real;        -- The constrained angle between the bodies: bodyB_angle - bodyA_angle.
         enableLimit       : Boolean;     -- Enable/disable the joint limit.

         lowerTranslation,                -- The lower translation limit, usually in meters.
         upperTranslation  : Real;        -- The upper translation limit, usually in meters.

         enableMotor       : Boolean;     -- Enable/disable the joint motor.
         maxMotorForce     : Real;        -- The maximum motor torque, usually in N-m.
         motorSpeed        : Real;        -- The desired motor speed in radians per second.
      end record;


   --    b2PrismaticJointDef()
   --    {
   --       type = e_prismaticJoint;
   --       localAnchorA.SetZero();
   --       localAnchorB.SetZero();
   --       localAxisA.Set(1.0f, 0.0f);
   --       referenceAngle = 0.0f;
   --       enableLimit = false;
   --       lowerTranslation = 0.0f;
   --       upperTranslation = 0.0f;
   --       enableMotor = false;
   --       maxMotorForce = 0.0f;
   --       motorSpeed = 0.0f;
   --    }
   --

   function  to_b2PrismaticJointDef return b2PrismaticJointDef;



   --    Initialize the bodies, anchors, axis, and reference angle using the world
   --    anchor and unit world axis.
   --    void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);
   --

   procedure initialize (Self : out b2PrismaticJointDef;   BodyA, BodyB : access b2Body;
                                                           Anchor       : in     b2Vec2;
                                                           Axis         : in     b2Vec2);






   --------------------
   --- b2PrismaticJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2PrismaticJoint     is new b2_Joint.b2Joint with private;
   type b2PrismaticJoint_ptr is access all b2PrismaticJoint'Class;

   --  class b2DistanceJoint : public b2Joint
   --  {
   --  public:
   --
   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2PrismaticJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2PrismaticJoint) return b2Vec2;



   --    The local anchor point relative to bodyA's origin.
   --
   --    const b2Vec2& GetLocalAnchorA() const  { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2PrismaticJoint) return b2Vec2
     with inline;


   --    The local anchor point relative to bodyB's origin.
   --
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2PrismaticJoint) return b2Vec2
     with inline;



   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2PrismaticJoint;   inv_dt : in Real) return b2Vec2;



   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2PrismaticJoint;   inv_dt : in Real) return Real;




   --    The local joint axis relative to bodyA.
   --
   --    const b2Vec2& GetLocalAxisA() const { return m_localXAxisA; }
   --

   function GetLocalAxisA (Self : in b2PrismaticJoint) return b2Vec2
     with inline;





   --    Get the reference angle.
   --
   --    float GetReferenceAngle() const { return m_referenceAngle; }
   --

   function GetReferenceAngle (Self : in b2PrismaticJoint) return Real
     with inline;




   --    Get the current joint translation, usually in meters.
   --
   --    float GetJointTranslation() const;
   --

   function GetJointTranslation (Self : in b2PrismaticJoint) return Real;




   --    Get the current joint translation speed, usually in meters per second.
   --
   --    float GetJointSpeed() const;
   --

   function GetJointSpeed (Self : in b2PrismaticJoint) return Real;




   --    Is the joint limit enabled?
   --
   --    bool IsLimitEnabled() const;
   --

   function IsLimitEnabled (Self : in b2PrismaticJoint) return Boolean;




   --    Enable/disable the joint limit.
   --
   --    void EnableLimit(bool flag);
   --

   procedure EnableLimit (Self : in out b2PrismaticJoint;   Flag : in Boolean);



   --    Get the lower joint limit, usually in meters.
   --
   --    float GetLowerLimit() const;
   --

   function GetLowerLimit (Self : in b2PrismaticJoint) return Real;




   --    Get the upper joint limit, usually in meters.
   --
   --    float GetUpperLimit() const;
   --

   function GetUpperLimit (Self : in b2PrismaticJoint) return Real;




   --    Set the joint limits, usually in meters.
   --
   --    void SetLimits(float lower, float upper);
   --

   procedure SetLimits (Self : in out b2PrismaticJoint;   Lower, Upper : in Real);





   --    Is the joint motor enabled?
   --
   --    bool IsMotorEnabled() const;
   --

   function IsMotorEnabled (Self : in b2PrismaticJoint) return Boolean;




   --    Enable/disable the joint motor.
   --
   --    void EnableMotor(bool flag);
   --

   procedure EnableMotor (Self : in out b2PrismaticJoint;   Flag : in Boolean);




   --    Set the motor speed, usually in meters per second.
   --
   --    void SetMotorSpeed(float speed);
   --

   procedure SetMotorSpeed (Self : in out b2PrismaticJoint;   Speed : in Real);





   --  Get the motor speed, usually in meters per second.
   --
   --  inline float b2PrismaticJoint::GetMotorSpeed() const
   --  {
   --    return m_motorSpeed;
   --  }
   --

   function GetMotorSpeed (Self : in b2PrismaticJoint) return Real
     with inline;




   --    Set the maximum motor force, usually in N.
   --
   --    void SetMaxMotorForce(float force);
   --

   procedure SetMaxMotorForce (Self : in out b2PrismaticJoint;   Force : in Real);




   --    float GetMaxMotorForce() const { return m_maxMotorForce; }
   --

   function GetMaxMotorForce (Self : in b2PrismaticJoint) return Real
     with inline;



   --    Get the current motor force given the inverse time step, usually in N.
   --
   --    float GetMotorForce(float inv_dt) const;
   --

   function GetMotorForce (Self : in b2PrismaticJoint;   inv_dt : in Real) return Real;




   --    Dump to b2Log
   --
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2PrismaticJoint);



   --    void Draw(b2Draw* draw) const override;
   --
   --

   overriding
   procedure draw (Self : in b2PrismaticJoint;   Draw : access b2Draw'Class);






   --------------
   --  protected:
   --

   --    friend class b2Joint;
   --    friend class b2GearJoint;
   --
   --    b2PrismaticJoint(const b2PrismaticJointDef* def);
   --

   package Forge
   is
      --    b2DistanceJoint(const b2DistanceJointDef* data);
      --
      function to_b2PrismaticJoint (Def : in b2PrismaticJointDef) return b2PrismaticJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2PrismaticJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2PrismaticJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2PrismaticJoint;   Data : in b2SolverData) return Boolean;




   --  class b2PrismaticJoint : public b2Joint
   --  {
   --  public:
   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    b2Vec2 m_localXAxisA;
   --    b2Vec2 m_localYAxisA;
   --    float m_referenceAngle;
   --    b2Vec2 m_impulse;
   --    float m_motorImpulse;
   --    float m_lowerImpulse;
   --    float m_upperImpulse;
   --    float m_lowerTranslation;
   --    float m_upperTranslation;
   --    float m_maxMotorForce;
   --    float m_motorSpeed;
   --    bool m_enableLimit;
   --    bool m_enableMotor;
   --
   --    // Solver temp
   --    int32 m_indexA;
   --    int32 m_indexB;
   --    b2Vec2 m_localCenterA;
   --    b2Vec2 m_localCenterB;
   --    float m_invMassA;
   --    float m_invMassB;
   --    float m_invIA;
   --    float m_invIB;
   --    b2Vec2 m_axis, m_perp;
   --    float m_s1, m_s2;
   --    float m_a1, m_a2;
   --    b2Mat22 m_K;
   --    float m_translation;
   --    float m_axialMass;
   --  };
   --

   type b2PrismaticJoint is new b2_Joint.b2Joint with
      record
         m_localAnchorA,
         m_localAnchorB     : b2Vec2;

         m_localXAxisA,
         m_localYAxisA      : b2Vec2;

         m_referenceAngle   : Real;

         m_impulse          : b2Vec2;
         m_motorImpulse,
         m_lowerImpulse,
         m_upperImpulse     : Real;

         m_lowerTranslation,
         m_upperTranslation : Real;

         m_maxMotorForce,
         m_motorSpeed       : Real;

         m_enableLimit,
         m_enableMotor      : Boolean;

         -- Solver temp
         --
         m_indexA,
         m_indexB       : Natural;

         m_localCenterA,
         m_localCenterB : b2Vec2;

         m_invMassA,
         m_invMassB     : Real;

         m_invIA,
         m_invIB        : Real;

         m_axis,
         m_perp         : b2Vec2;

         m_s1,
         m_s2           : Real;

         m_a1,
         m_a2           : Real;

         m_K            : b2Mat22;

         m_translation  : Real;
         m_axialMass    : Real;
      end record;


end box2d.b2_Joint.b2_prismatic_Joint;
