with
     box2d.b2_Body;


package box2d.b2_Joint.b2_wheel_Joint
--
--  A wheel joint. This joint provides two degrees of freedom: translation
--  along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
--  line constraint with a rotational motor and a linear spring/damper. The spring/damper is
--  initialized upon creation. This joint is designed for vehicle suspensions.
--
is
   use b2_Body;


   -------------------
   --- b2WheelJointDef
   --

   --  Wheel joint definition. This requires defining a line of
   --  motion using an axis and an anchor point. The definition uses local
   --  anchor points and a local axis so that the initial configuration
   --  can violate the constraint slightly. The joint translation is zero
   --  when the local anchor points coincide in world space. Using local
   --  anchors and a local axis helps when saving and loading a game.
   --

   --  struct b2WheelJointDef : public b2JointDef
   --  {
   --    The local anchor point relative to bodyA's origin.
   --    b2Vec2 localAnchorA;
   --
   --    The local anchor point relative to bodyB's origin.
   --    b2Vec2 localAnchorB;
   --
   --    The local translation axis in bodyA.
   --    b2Vec2 localAxisA;
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
   --    float maxMotorTorque;
   --
   --    The desired motor speed in radians per second.
   --    float motorSpeed;
   --
   --    Suspension stiffness. Typically in units N/m.
   --    float stiffness;
   --
   --    Suspension damping. Typically in units of N*s/m.
   --    float damping;
   --  };
   --

   type b2WheelJointDef is new b2_Joint.b2JointDef with
      record
         localAnchorA     : b2Vec2;     -- The local anchor point relative to bodyA's origin.
         localAnchorB     : b2Vec2;     -- The local anchor point relative to bodyB's origin.

         localAxisA       : b2Vec2;     -- The local translation axis in bodyA.
         enableLimit      : Boolean;    -- Enable/disable the joint limit.

         lowerTranslation : Real;       -- The lower translation limit, usually in meters.
         upperTranslation : Real;       -- The upper translation limit, usually in meters.

         enableMotor      : Boolean;    -- Enable/disable the joint motor.
         maxMotorTorque   : Real;       -- The maximum motor torque, usually in N-m.
         motorSpeed       : Real;       -- The desired motor speed in radians per second.

         stiffness        : Real;       -- Suspension stiffness. Typically in units N/m.
         damping          : Real;       -- Suspension damping. Typically in units of N*s/m.
      end record;


   --    b2WheelJointDef()
   --    {
   --       type = e_wheelJoint;
   --       localAnchorA.SetZero();
   --       localAnchorB.SetZero();
   --       localAxisA.Set(1.0f, 0.0f);
   --       enableLimit = false;
   --       lowerTranslation = 0.0f;
   --       upperTranslation = 0.0f;
   --       enableMotor = false;
   --       maxMotorTorque = 0.0f;
   --       motorSpeed = 0.0f;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --

   function  to_b2WheelJointDef return b2WheelJointDef;



   --    Initialize the bodies, anchors, axis, and reference angle using the world
   --    anchor and world axis.
   --    void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor, const b2Vec2& axis);
   --

   procedure initialize (Self : out b2WheelJointDef;   BodyA,  BodyB : access b2Body;
                                                       Anchor, Axis  : in     b2Vec2);






   ----------------
   --- b2WheelJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2WheelJoint     is new b2_Joint.b2Joint with private;
   type b2WheelJoint_ptr is access all b2WheelJoint'Class;


   --  class b2DistanceJoint : public b2Joint
   --  {
   --  public:
   --
   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2WheelJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2WheelJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2WheelJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2WheelJoint;   inv_dt : in Real) return Real;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2WheelJoint) return b2Vec2
     with inline;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2WheelJoint) return b2Vec2
     with inline;



   --    The local joint axis relative to bodyA.
   --
   --    const b2Vec2& GetLocalAxisA() const { return m_localXAxisA; }
   --

   function getLocalAxisA (Self : in b2WheelJoint) return b2Vec2
     with inline;



   --    Get the current joint translation, usually in meters.
   --
   --    float GetJointTranslation() const;
   --

   function getJointTranslation (Self : in b2WheelJoint) return Real;



   --    Get the current joint linear speed, usually in meters per second.
   --
   --    float GetJointLinearSpeed() const;
   --

   function getJointLinearSpeed (Self : in b2WheelJoint) return Real;



   --    Get the current joint angle in radians.
   --
   --    float GetJointAngle() const;
   --

   function getJointAngle (Self : in b2WheelJoint) return Real;



   --    Get the current joint angular speed in radians per second.
   --
   --    float GetJointAngularSpeed() const;
   --

   function getJointAngularSpeed (Self : in b2WheelJoint) return Real;




   --    Is the joint limit enabled?
   --
   --    bool IsLimitEnabled() const;
   --

   function isLimitEnabled (Self : in b2WheelJoint) return Boolean;




   --    Enable/disable the joint translation limit.
   --
   --    void EnableLimit(bool flag);
   --

   procedure enableLimit (Self : in out b2WheelJoint;   Flag : in Boolean);




   --    Get the lower joint translation limit, usually in meters.
   --
   --    float GetLowerLimit() const;
   --

   function getLowerLimit (Self : in b2WheelJoint) return Real;



   --    Get the upper joint translation limit, usually in meters.
   --
   --    float GetUpperLimit() const;
   --

   function getUpperLimit (Self : in b2WheelJoint) return Real;



   --    Set the joint translation limits, usually in meters.
   --
   --    void SetLimits(float lower, float upper);
   --

   procedure setLimits (Self : in out b2WheelJoint;   Lower, Upper : in Real);



   --    Is the joint motor enabled?
   --
   --    bool IsMotorEnabled() const;
   --

   function isMotorEnabled (Self : in b2WheelJoint) return Boolean;



   --    Enable/disable the joint motor.
   --
   --    void EnableMotor(bool flag);
   --

   procedure enableMotor (Self : in out b2WheelJoint;   Flag : in Boolean);



   --    Set the motor speed, usually in radians per second.
   --
   --    void SetMotorSpeed(float speed);
   --

   procedure setMotorSpeed (Self : in out b2WheelJoint;   Speed : in Real);



   --    Get the motor speed, usually in radians per second.
   --
   --    float GetMotorSpeed() const;
   --
   --  inline float b2WheelJoint::GetMotorSpeed() const
   --  {
   --    return m_motorSpeed;
   --  }
   --

   function getMotorSpeed (Self : in b2WheelJoint) return Real
     with inline;



   --    Set/Get the maximum motor force, usually in N-m.
   --
   --    void SetMaxMotorTorque(float torque);
   --

   procedure setMaxMotorTorque (Self : in out b2WheelJoint;   Torque : in Real)
     with inline;



   --    float GetMaxMotorTorque() const;
   --
   --  inline float b2WheelJoint::GetMaxMotorTorque() const
   --  {
   --    return m_maxMotorTorque;
   --  }
   --

   function getMaxMotorTorque (Self : in b2WheelJoint) return Real
     with inline;



   --    Get the current motor torque given the inverse time step, usually in N-m.
   --
   --    float GetMotorTorque(float inv_dt) const;
   --

   function getMotorTorque (Self : in b2WheelJoint;   inv_dt : in Real) return Real;


   --    Access spring stiffness
   --

   --    void SetStiffness(float stiffness);
   --

   procedure setStiffness (Self : in out b2WheelJoint;   Stiffness : in Real);


   --    float GetStiffness() const;
   --

   function  getStiffness (Self : in b2WheelJoint) return Real;



   -- Access damping.
   --

   procedure setDamping (Self : in out b2WheelJoint;   Damping : in Real);

   function  getDamping (Self : in b2WheelJoint) return Real;




   --    Dump joint to dmLog
   --
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2WheelJoint);




   --    void Draw(b2Draw* draw) const override;
   --

   overriding
   procedure draw (Self : in b2WheelJoint;   Draw : access b2Draw'Class);





   --------------
   --  protected:
   --
   --    friend class b2Joint;


   package Forge
   is

      --    b2WheelJoint(const b2WheelJointDef* def);
      --

      function to_b2WheelJoint (Def : in b2WheelJointDef) return b2WheelJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2WheelJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2WheelJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2WheelJoint;   Data : in b2SolverData) return Boolean;



   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    b2Vec2 m_localXAxisA;
   --    b2Vec2 m_localYAxisA;
   --
   --    float m_impulse;
   --    float m_motorImpulse;
   --    float m_springImpulse;
   --
   --    float m_lowerImpulse;
   --    float m_upperImpulse;
   --    float m_translation;
   --    float m_lowerTranslation;
   --    float m_upperTranslation;
   --
   --    float m_maxMotorTorque;
   --    float m_motorSpeed;
   --
   --    bool m_enableLimit;
   --    bool m_enableMotor;
   --
   --    float m_stiffness;
   --    float m_damping;
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
   --
   --    b2Vec2 m_ax, m_ay;
   --    float m_sAx, m_sBx;
   --    float m_sAy, m_sBy;
   --
   --    float m_mass;
   --    float m_motorMass;
   --    float m_axialMass;
   --    float m_springMass;
   --
   --    float m_bias;
   --    float m_gamma;
   --

   type b2WheelJoint is new b2_Joint.b2Joint with
      record
         m_localAnchorA,
         m_localAnchorB     : b2Vec2;

         m_localXAxisA,
         m_localYAxisA      : b2Vec2;

         m_impulse,
         m_motorImpulse,
         m_springImpulse    : Real;

         m_lowerImpulse,
         m_upperImpulse     : Real;

         m_translation,
         m_lowerTranslation,
         m_upperTranslation : Real;

         m_maxMotorTorque,
         m_motorSpeed       : Real;

         m_enableLimit,
         m_enableMotor      : Boolean;

         m_stiffness,
         m_damping          : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB           : Natural;

         m_localCenterA,
         m_localCenterB     : b2Vec2;

         m_invMassA,
         m_invMassB         : Real;

         m_invIA,
         m_invIB            : Real;

         m_ax,
         m_ay               : b2Vec2;

         m_sAx,
         m_sBx              : Real;

         m_sAy,
         m_sBy              : Real;

         m_mass,
         m_motorMass,
         m_axialMass,
         m_springMass       : Real;

         m_bias,
         m_gamma            : Real;
      end record;


end box2d.b2_Joint.b2_wheel_Joint;
