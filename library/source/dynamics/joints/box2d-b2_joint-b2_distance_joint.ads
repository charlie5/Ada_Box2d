with
     box2d.b2_Body,
     box2d.b2_Math;
     --  box2d.b2_Settings;


private
with
     box2d.b2_time_Step;


package box2d.b2_Joint.b2_Distance_Joint
is
   use b2_Body;


   ----------------------
   --- b2DistanceJointDef
   --

   --  Distance joint definition. This requires defining an anchor point on both
   --  bodies and the non-zero distance of the distance joint. The definition uses
   --  local anchor points so that the initial configuration can violate the
   --  constraint slightly. This helps when saving and loading a game.
   --
   --  struct b2DistanceJointDef : public b2JointDef
   --  {
   --    b2DistanceJointDef()
   --    {
   --       type = e_distanceJoint;
   --       localAnchorA.Set(0.0f, 0.0f);
   --       localAnchorB.Set(0.0f, 0.0f);
   --       length = 1.0f;
   --       minLength = 0.0f;
   --       maxLength = FLT_MAX;
   --       stiffness = 0.0f;
   --       damping = 0.0f;
   --    }
   --


   --    Initialize the bodies, anchors, and rest length using world space anchors.
   --    The minimum and maximum lengths are set to the rest length.
   --
   --    void Initialize (b2Body* bodyA, b2Body* bodyB,
   --                     const b2Vec2& anchorA, const b2Vec2& anchorB);
   --


   --    The local anchor point relative to bodyA's origin.
   --    b2Vec2 localAnchorA;
   --
   --    The local anchor point relative to bodyB's origin.
   --    b2Vec2 localAnchorB;
   --
   --    The rest length of this joint. Clamped to a stable minimum value.
   --    float length;
   --
   --    Minimum length. Clamped to a stable minimum value.
   --    float minLength;
   --
   --    Maximum length. Must be greater than or equal to the minimum length.
   --    float maxLength;
   --
   --    The linear stiffness in N/m.
   --    float stiffness;
   --
   --    The linear damping in N*s/m.
   --    float damping;
   --  };
   --

   type b2DistanceJointDef is new b2_Joint.b2JointDef with
      record
         localAnchorA : b2Vec2;     -- The local anchor point relative to bodyA's origin.
         localAnchorB : b2Vec2;     -- The local anchor point relative to bodyB's origin.
         length       : Real;       -- The rest length of this joint. Clamped to a stable minimum value.
         minLength    : Real;       -- Minimum length. Clamped to a stable minimum value.
         maxLength    : Real;       -- Maximum length. Must be greater than or equal to the minimum length.
         stiffness    : Real;       -- The linear stiffness in N/m.
         damping      : Real;       -- The linear damping in N*s/m.
      end record;


   function  to_b2DistanceJointDef return b2DistanceJointDef;

   procedure initialize (Self : out b2DistanceJointDef;   bodyA,   bodyB   : access b2Body;
                                                          anchorA, anchorB : in     b2Vec2);






   -------------------
   --- b2DistanceJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2DistanceJoint     is new b2_Joint.b2Joint with private;
   type b2DistanceJoint_ptr is access all b2DistanceJoint;



   --  class b2DistanceJoint : public b2Joint
   --  {
   --  public:
   --
   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2DistanceJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2DistanceJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2DistanceJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2DistanceJoint;   inv_dt : in Real) return Real;



   --    The local anchor point relative to bodyA's origin.
   --    const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }
   --

   function getLocalAnchorA (Self : in b2DistanceJoint) return b2Vec2
     with inline;


   --    The local anchor point relative to bodyB's origin.
   --    const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }
   --

   function getLocalAnchorB (Self : in b2DistanceJoint) return b2Vec2
     with inline;



   --    Get the rest length
   --    float GetLength() const { return m_length; }
   --

   function getLength (Self : in b2DistanceJoint) return Real
     with inline;



   --    Set the rest length
   --    @returns clamped rest length
   --    float SetLength(float length);
   --

   function setLength (Self : in out b2DistanceJoint;   Length : in Real) return Real;



   --    Get the minimum length
   --    float GetMinLength() const { return m_minLength; }
   --

   function getMinLength (Self : in b2DistanceJoint) return Real
     with inline;



   --    Set the minimum length
   --    @returns the clamped minimum length
   --    float SetMinLength(float minLength);
   --

   function setMinLength (Self : in out b2DistanceJoint;   minLength : in Real) return Real;




   --    Get the maximum length
   --    float GetMaxLength() const { return m_maxLength; }
   --
   function getMaxLength (Self : in b2DistanceJoint) return Real
     with inline;



   --    Set the maximum length
   --    @returns the clamped maximum length
   --    float SetMaxLength(float maxLength);
   --

   function setMaxLength (Self : in out b2DistanceJoint;   maxLength : in Real) return Real;



   --    Get the current length
   --    float GetCurrentLength() const;
   --

   function getCurrentLength (Self : in b2DistanceJoint) return Real;




   --    Set/get the linear stiffness in N/m
   --
   --    void SetStiffness(float stiffness) { m_stiffness = stiffness; }
   --

   procedure setStiffness (Self : in out b2DistanceJoint;   stiffness : in Real)
     with inline;


   --    float GetStiffness() const { return m_stiffness; }
   --

   function getStiffness (Self : in b2DistanceJoint) return Real
     with inline;


   --    Set/get linear damping in N*s/m
   --
   --    void SetDamping(float damping) { m_damping = damping; }
   --

   procedure setDamping (Self : in out b2DistanceJoint;   damping : in Real)
     with inline;


   --    float GetDamping() const { return m_damping; }
   --

   function getDamping (Self : in b2DistanceJoint) return Real
     with inline;



   --    Dump joint to dmLog
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2DistanceJoint);



   --    ///
   --    void Draw(b2Draw* draw) const override;
   --

   overriding
   procedure draw (Self : in b2DistanceJoint;   Draw : access b2Draw'Class);




   -- protected:
   --

   package Forge
   is

      --    b2DistanceJoint(const b2DistanceJointDef* data);
      --

      function to_b2DistanceJoint (Def : in b2DistanceJointDef) return b2DistanceJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2DistanceJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2DistanceJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2DistanceJoint;   Data : in b2SolverData) return Boolean;




   --    float m_stiffness;
   --    float m_damping;
   --    float m_bias;
   --    float m_length;
   --    float m_minLength;
   --    float m_maxLength;
   --
   --    // Solver shared
   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    float m_gamma;
   --    float m_impulse;
   --    float m_lowerImpulse;
   --    float m_upperImpulse;
   --
   --    // Solver temp
   --    int32 m_indexA;
   --    int32 m_indexB;
   --    b2Vec2 m_u;
   --    b2Vec2 m_rA;
   --    b2Vec2 m_rB;
   --    b2Vec2 m_localCenterA;
   --    b2Vec2 m_localCenterB;
   --    float m_currentLength;
   --    float m_invMassA;
   --    float m_invMassB;
   --    float m_invIA;
   --    float m_invIB;
   --    float m_softMass;
   --    float m_mass;
   --  };


   type b2DistanceJoint is new b2_Joint.b2Joint with
      record
         m_stiffness,
         m_damping,
         m_bias,
         m_length,
         m_minLength,
         m_maxLength    : Real;

         -- Solver shared.
         --
         m_localAnchorA,
         m_localAnchorB : b2Vec2;
         m_gamma,
         m_impulse,
         m_lowerImpulse,
         m_upperImpulse : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB       : Natural;

         m_u            : b2Vec2;

         m_rA,
         m_rB           : b2Vec2;

         m_localCenterA,
         m_localCenterB : b2Vec2;

         m_currentLength,
         m_invMassA,
         m_invMassB,
         m_invIA,
         m_invIB,
         m_softMass,
         m_mass         : Real;
      end record;

end box2d.b2_Joint.b2_Distance_Joint;
