with
     box2d.b2_Body;


package box2d.b2_Joint.b2_pulley_Joint
--
--  The pulley joint is connected to two bodies and two fixed ground points.
--  The pulley supports a ratio such that:
--  length1 + ratio * length2 <= constant
--  Yes, the force transmitted is scaled by the ratio.
--  Warning: the pulley joint can get a bit squirrelly by itself. They often
--  work better when combined with prismatic joints. You should also cover the
--  the anchor points with static shapes to prevent one side from going to
--  zero length.
--
is
   use b2_Body;


   b2_minPulleyLength : constant Real;



   --------------------
   --- b2PulleyJointDef
   --


   --  Pulley joint definition. This requires two ground anchors,
   --  two dynamic body anchor points, and a pulley ratio.

   --  struct b2PulleyJointDef : public b2JointDef
   --  {
   --    The first ground anchor in world coordinates. This point never moves.
   --    b2Vec2 groundAnchorA;
   --
   --    The second ground anchor in world coordinates. This point never moves.
   --    b2Vec2 groundAnchorB;
   --
   --    The local anchor point relative to bodyA's origin.
   --    b2Vec2 localAnchorA;
   --
   --    The local anchor point relative to bodyB's origin.
   --    b2Vec2 localAnchorB;
   --
   --    The a reference length for the segment attached to bodyA.
   --    float lengthA;
   --
   --    The a reference length for the segment attached to bodyB.
   --    float lengthB;
   --
   --    The pulley ratio, used to simulate a block-and-tackle.
   --    float ratio;
   --  };
   --

   type b2PulleyJointDef is new b2_Joint.b2JointDef with
      record
         groundAnchorA : b2Vec2;     -- The first ground anchor in world coordinates. This point never moves.
         groundAnchorB : b2Vec2;     -- The second ground anchor in world coordinates. This point never moves.

         localAnchorA : b2Vec2;      -- The local anchor point relative to bodyA's origin.
         localAnchorB : b2Vec2;      -- The local anchor point relative to bodyB's origin.

         lengthA      : Real;        -- The a reference length for the segment attached to bodyA.
         lengthB      : Real;        -- The a reference length for the segment attached to bodyB.

         Ratio        : Real;        -- The pulley ratio, used to simulate a block-and-tackle.
      end record;


   --    b2PulleyJointDef()
   --    {
   --       type = e_pulleyJoint;
   --       groundAnchorA.Set(-1.0f, 1.0f);
   --       groundAnchorB.Set(1.0f, 1.0f);
   --       localAnchorA.Set(-1.0f, 0.0f);
   --       localAnchorB.Set(1.0f, 0.0f);
   --       lengthA = 0.0f;
   --       lengthB = 0.0f;
   --       ratio = 1.0f;
   --       collideConnected = true;
   --    }
   --

   function  to_b2PulleyJointDef return b2PulleyJointDef;




   --    Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
   --
   --    void Initialize(b2Body* bodyA, b2Body* bodyB,
   --                const b2Vec2& groundAnchorA, const b2Vec2& groundAnchorB,
   --                const b2Vec2& anchorA, const b2Vec2& anchorB,
   --                float ratio);
   --

   procedure initialize (Self : out b2PulleyJointDef;   bodyA,   bodyB               : access b2Body;
                                                        groundAnchorA, groundAnchorB : in     b2Vec2;
                                                        AnchorA, AnchorB             : in     b2Vec2;
                                                        Ratio                        : in     Real);





   -----------------
   --- b2PulleyJoint
   --

   --  A distance joint constrains two points on two bodies to remain at a fixed
   --  distance from each other. You can view this as a massless, rigid rod.
   --

   type b2PulleyJoint     is new b2_Joint.b2Joint with private;
   type b2PulleyJoint_ptr is access all b2PulleyJoint'Class;

   --    Get the first ground anchor.
   --
   --    b2Vec2 GetGroundAnchorA() const;
   --

   function getGroundAnchorA (Self : in b2PulleyJoint) return b2Vec2;



   --    Get the second ground anchor.
   --
   --    b2Vec2 GetGroundAnchorB() const;
   --

   function getGroundAnchorB (Self : in b2PulleyJoint) return b2Vec2;



   --    b2Vec2 GetAnchorA() const override;
   --    b2Vec2 GetAnchorB() const override;
   --

   overriding
   function getAnchorA (Self : in b2PulleyJoint) return b2Vec2;

   overriding
   function getAnchorB (Self : in b2PulleyJoint) return b2Vec2;



   --    Get the reaction force given the inverse time step.
   --    Unit is N.
   --    b2Vec2 GetReactionForce(float inv_dt) const override;
   --

   overriding
   function getReactionForce (Self : in b2PulleyJoint;   inv_dt : in Real) return b2Vec2;



   --    Get the reaction torque given the inverse time step.
   --    Unit is N*m. This is always zero for a distance joint.
   --    float GetReactionTorque(float inv_dt) const override;
   --

   overriding
   function getReactionTorque (Self : in b2PulleyJoint;   inv_dt : in Real) return Real;




   --    Get the current length of the segment attached to bodyA.
   --    float GetLengthA() const;
   --

   function getLengthA (Self : in b2PulleyJoint) return Real
     with inline;


   --    Get the current length of the segment attached to bodyB.
   --    float GetLengthB() const;
   --

   function getLengthB (Self : in b2PulleyJoint) return Real
     with inline;



   --    Get the pulley ratio.
   --
   --    float GetRatio() const;
   --

   function getRatio (Self : in out b2PulleyJoint) return Real;



   --    Get the current length of the segment attached to bodyA.
   --
   --    float GetCurrentLengthA() const;
   --

   function getCurrentLengthA (Self : in b2PulleyJoint) return Real
     with inline;



   --    Get the current length of the segment attached to bodyB.
   --
   --    float GetCurrentLengthB() const;
   --

   function getCurrentLengthB (Self : in b2PulleyJoint) return Real
     with inline;



   --    Implement b2Joint::ShiftOrigin
   --
   --    void ShiftOrigin(const b2Vec2& newOrigin) override;
   --

   overriding
   procedure shiftOrigin (Self : in out b2PulleyJoint;   newOrigin : in b2Vec2);



   --    Dump joint to dmLog.
   --
   --    void Dump() override;
   --

   overriding
   procedure dump (Self : in b2PulleyJoint);




   --------------
   --  protected:
   --
   --    friend class b2Joint;

   package Forge
   is

      --    b2PulleyJoint(const b2PulleyJointDef* data);
      --

      function to_b2PulleyJoint (Def : in b2PulleyJointDef) return b2PulleyJoint;

   end Forge;




private

   --    friend class b2Joint;
   --


   --    void InitVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure initVelocityConstraints (Self : in out b2PulleyJoint;   Data : in b2SolverData);



   --    void SolveVelocityConstraints(const b2SolverData& data) override;
   --

   overriding
   procedure solveVelocityConstraints (Self : in out b2PulleyJoint;   Data : in b2SolverData);



   --    bool SolvePositionConstraints(const b2SolverData& data) override;
   --

   overriding
   function solvePositionConstraints (Self : in out b2PulleyJoint;   Data : in b2SolverData) return Boolean;



   --    b2Vec2 m_groundAnchorA;
   --    b2Vec2 m_groundAnchorB;
   --    float m_lengthA;
   --    float m_lengthB;
   --
   --    // Solver shared
   --    b2Vec2 m_localAnchorA;
   --    b2Vec2 m_localAnchorB;
   --    float m_constant;
   --    float m_ratio;
   --    float m_impulse;
   --
   --    // Solver temp
   --    int32 m_indexA;
   --    int32 m_indexB;
   --    b2Vec2 m_uA;
   --    b2Vec2 m_uB;
   --    b2Vec2 m_rA;
   --    b2Vec2 m_rB;
   --    b2Vec2 m_localCenterA;
   --    b2Vec2 m_localCenterB;
   --    float m_invMassA;
   --    float m_invMassB;
   --    float m_invIA;
   --    float m_invIB;
   --    float m_mass;

   type b2PulleyJoint is new b2_Joint.b2Joint with
      record
         m_groundAnchorA,
         m_groundAnchorB : b2Vec2;

         m_lengthA,
         m_lengthB       : Real;

         -- Solver shared.
         --
         m_localAnchorA,
         m_localAnchorB  : b2Vec2;

         m_constant,
         m_ratio,
         m_impulse       : Real;

         -- Solver temp.
         --
         m_indexA,
         m_indexB        : Natural;

         m_uA,
         m_uB            : b2Vec2;

         m_rA,
         m_rB            : b2Vec2;

         m_localCenterA,
         m_localCenterB  : b2Vec2;

         m_invMassA,
         m_invMassB,
         m_invIA,
         m_invIB,
         m_mass          : Real;
      end record;



   --  const float b2_minPulleyLength = 2.0f;
   --

   b2_minPulleyLength : constant Real := 2.0;


end box2d.b2_Joint.b2_pulley_Joint;
