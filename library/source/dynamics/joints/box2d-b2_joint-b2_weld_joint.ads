with
     box2d.b2_Body;


package box2d.b2_Joint.b2_weld_Joint
--
--  A weld joint essentially glues two bodies together. A weld joint may
--  distort somewhat because the island constraint solver is approximate.
--
is
   use b2_Body;


   ------------------
   --- b2WeldJointDef
   --

   --  Weld joint definition. You need to specify local anchor points
   --  where they are attached and the relative body angle. The position
   --  of the anchor points is important for computing the reaction torque.
   --

   --  struct b2WeldJointDef : public b2JointDef
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
   --    The rotational stiffness in N*m
   --    Disable softness with a value of 0
   --    float stiffness;
   --
   --    The rotational damping in N*m*s
   --    float damping;
   --  };
   --

   type b2WeldJointDef is new b2_Joint.b2JointDef with
      record
         localAnchorA   : b2Vec2;     -- The local anchor point relative to bodyA's origin.
         localAnchorB   : b2Vec2;     -- The local anchor point relative to bodyB's origin.

         referenceAngle : Real;       -- The bodyB angle minus bodyA angle in the reference state (radians).
         stiffness      : Real;       -- The rotational stiffness in N*m. Disable softness with a value of 0.
         damping        : Real;       -- The rotational damping in N*m*s.
      end record;


   --    b2WeldJointDef()
   --    {
   --       type = e_weldJoint;
   --       localAnchorA.Set(0.0f, 0.0f);
   --       localAnchorB.Set(0.0f, 0.0f);
   --       referenceAngle = 0.0f;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --

   function  to_b2WeldJointDef return b2WeldJointDef;




   --    Initialize the bodies, anchors, reference angle, stiffness, and damping.
   --    @param bodyA the first body connected by this joint
   --    @param bodyB the second body connected by this joint
   --    @param anchor the point of connection in world coordinates
   --    void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);
   --

   procedure initialize (Self : out b2WeldJointDef;   bodyA, bodyB : access b2Body;
                                                      Anchor       : in     b2Vec2);






   ---------------
   --- b2WeldJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2WeldJoint     is new b2_Joint.b2Joint with private;
   type b2WeldJoint_ptr is access all b2WeldJoint'Class;

   --  class b2DistanceJoint : public b2Joint
   --  {
   --  public:
   --
   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2WeldJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2WeldJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2WeldJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2WeldJoint;   inv_dt : in Real) return Real;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2WeldJoint) return b2Vec2
     with inline;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2WeldJoint) return b2Vec2
     with inline;



   --    Get the reference angle.
   --    float GetReferenceAngle() const { return m_referenceAngle; }
   --

   function getReferenceAngle (Self : in b2WeldJoint) return Real
     with inline;



   --    Set/get the linear stiffness in N/m
   --
   --    void SetStiffness(float stiffness) { m_stiffness = stiffness; }
   --

   procedure setStiffness (Self : in out b2WeldJoint;   stiffness : in Real)
     with inline;



   --    float GetStiffness() const { return m_stiffness; }
   --

   function getStiffness (Self : in b2WeldJoint) return Real
     with inline;



   --    Set/get linear damping in N*s/m
   --
   --    void SetDamping(float damping) { m_damping = damping; }
   --

   procedure setDamping (Self : in out b2WeldJoint;   damping : in Real)
     with inline;



   --    float GetDamping() const { return m_damping; }
   --

   function getDamping (Self : in b2WeldJoint) return Real
     with inline;



   --    Dump joint to dmLog
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2WeldJoint);






   --------------
   --  protected:
   --
   --    friend class b2Joint;
   --

   --    b2WeldJoint(const b2WeldJointDef* def);
   --

   package Forge
   is

      --    b2DistanceJoint(const b2DistanceJointDef* data);
      --

      function to_b2WeldJoint (Def : in b2WeldJointDef) return b2WeldJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2WeldJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2WeldJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2WeldJoint;   Data : in b2SolverData) return Boolean;




   --    float m_stiffness;
   --    float m_damping;
   --    float m_bias;
   --
   --    // Solver shared
   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    float m_referenceAngle;
   --    float m_gamma;
   --    b2Vec3 m_impulse;
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
   --    b2Mat33 m_mass;


   type b2WeldJoint is new b2_Joint.b2Joint with
      record
         m_stiffness,
         m_damping,
         m_bias            : Real;

         -- Solver shared.
         --
         m_localAnchorA,
         m_localAnchorB    : b2Vec2;

         m_referenceAngle,
         m_gamma           : Real;
         m_impulse         : b2Vec3;

         -- Solver temp.
         --
         m_indexA,
         m_indexB          : Natural;

         m_rA,
         m_rB              : b2Vec2;

         m_localCenterA,
         m_localCenterB    : b2Vec2;

         m_invMassA,
         m_invMassB,

         m_invIA,
         m_invIB           : Real;

         m_mass            : b2Mat33;
      end record;


end box2d.b2_Joint.b2_weld_Joint;
