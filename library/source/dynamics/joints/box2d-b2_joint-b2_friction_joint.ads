package box2d.b2_Joint.b2_friction_Joint
is

   --  Friction joint definition.
   --
   --  struct b2FrictionJointDef : public b2JointDef
   --  {
   --    The local anchor point relative to bodyA's origin.
   --    b2Vec2 localAnchorA;
   --
   --    The local anchor point relative to bodyB's origin.
   --    b2Vec2 localAnchorB;
   --
   --    The maximum friction force in N.
   --    float maxForce;
   --
   --    The maximum friction torque in N-m.
   --    float maxTorque;
   --  };
   --


   type b2FrictionJointDef is new b2_Joint.b2JointDef with
      record
         localAnchorA : b2Vec2;     -- The local anchor point relative to bodyA's origin.
         localAnchorB : b2Vec2;     -- The local anchor point relative to bodyB's origin.
         maxForce     : Real;       -- The maximum friction force in N.
         maxTorque    : Real;       -- The maximum friction torque in N-m.
      end record;


   --    b2FrictionJointDef()
   --    {
   --       type = e_frictionJoint;
   --       localAnchorA.SetZero();
   --       localAnchorB.SetZero();
   --       maxForce = 0.0f;
   --       maxTorque = 0.0f;
   --    }
   --

   function  to_b2FrictionJointDef return b2FrictionJointDef;



   --    Initialize the bodies, anchors, axis, and reference angle using the world
   --    anchor and world axis.
   --
   --    void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);
   --

   procedure initialize (Self : out b2FrictionJointDef;   bodyA, bodyB : access b2_Body.b2Body;
                                                          anchor       : in     b2Vec2);






   -------------------
   --- b2FrictionJoint
   --

   --  Friction joint. This is used for top-down friction.
   --  It provides 2D translational friction and angular friction.
   --
   --  class b2FrictionJoint : public b2Joint
   --  {

   type b2FrictionJoint is new b2_Joint.b2Joint with private;



   --  public:
   --

   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --
   overriding
   function getAnchorA (Self : in b2FrictionJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2FrictionJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2FrictionJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2FrictionJoint;   inv_dt : in Real) return Real;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2FrictionJoint) return b2Vec2
     with inline;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2FrictionJoint) return b2Vec2
     with inline;



   --    Dump joint to dmLog
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2FrictionJoint);



   --    /// Dump joint to dmLog.
   --
   --    void Draw(b2Draw* draw) const override;
   --


   --    Set the maximum friction force in N.
   --
   --    void SetMaxForce(float force);
   --

   procedure setMaxForce (Self : in out b2FrictionJoint;   Force : in Real);


   --    Get the maximum friction force in N.
   --
   --    float GetMaxForce() const;
   --

   function getMaxForce (Self : in b2FrictionJoint) return Real;


   --    Set the maximum friction torque in N*m.
   --
   --    void SetMaxTorque(float torque);
   --

   procedure SetMaxTorque (Self : in out b2FrictionJoint;   Torque : in Real);


   --    Get the maximum friction torque in N*m.
   --
   --    float GetMaxTorque() const;
   --

   function getMaxTorque (Self : in b2FrictionJoint) return Real;




   -- protected:
   --

   package Forge
   is
      --    b2FrictionJoint(const b2FrictionJointDef* def);
      --

      function to_b2FrictionJoint (Def : in b2FrictionJointDef) return b2FrictionJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2FrictionJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2FrictionJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2FrictionJoint;   Data : in b2SolverData) return Boolean;



   --  protected:
   --
   --    friend class b2Joint;
   --


   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --
   --    // Solver shared
   --    b2Vec2 m_linearImpulse;
   --    float m_angularImpulse;
   --    float m_maxForce;
   --    float m_maxTorque;
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
   --    b2Mat22 m_linearMass;
   --    float m_angularMass;
   --  };
   --

   type b2FrictionJoint is new b2_Joint.b2Joint with
      record
         m_localAnchorA,
         m_localAnchorB  : b2Vec2;

         -- Solver shared.
         --
         m_linearImpulse : b2Vec2;
         m_angularImpulse,
         m_maxForce,
         m_maxTorque     : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB        : Natural;

         m_rA,
         m_rB            : b2Vec2;

         m_localCenterA,
         m_localCenterB  : b2Vec2;

         m_invMassA,
         m_invMassB,

         m_invIA,
         m_invIB         : Real;

         m_linearMass    : b2Mat22;
         m_angularMass   : Real;
      end record;


end box2d.b2_Joint.b2_friction_Joint;
