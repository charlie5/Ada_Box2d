with
     box2d.b2_time_Step,
     box2d.b2_Draw,
     box2d.b2_Math;
     --  box2d.b2_Settings;

limited
with
     box2d.b2_Body;


package box2d.b2_Joint
is
   use b2_time_Step,
       box2d.b2_Draw,
       b2_Math;
       --  b2_Settings;



   --  The base joint class. Joints are used to constraint two bodies together in
   --  various fashions. Some joints also feature limits and motors.
   --
   --  class b2Joint

   type b2Joint     is abstract tagged private;
   type b2Joint_ptr is access all b2Joint'Class;



   --  class b2Body;
   --  class b2Draw;
   --  class b2Joint;
   --  struct b2SolverData;
   --  class b2BlockAllocator;



   --  enum b2JointType
   --  {
   --    e_unknownJoint,
   --    e_revoluteJoint,
   --    e_prismaticJoint,
   --    e_distanceJoint,
   --    e_pulleyJoint,
   --    e_mouseJoint,
   --    e_gearJoint,
   --    e_wheelJoint,
   --    e_weldJoint,
   --    e_frictionJoint,
   --    e_ropeJoint,
   --    e_motorJoint
   --  };
   --

   type b2JointType is (e_unknownJoint,
                        e_revoluteJoint,
                        e_prismaticJoint,
                        e_distanceJoint,
                        e_pulleyJoint,
                        e_mouseJoint,
                        e_gearJoint,
                        e_wheelJoint,
                        e_weldJoint,
                        e_frictionJoint,
                        e_ropeJoint,
                        e_motorJoint);


   --  struct b2Jacobian
   --  {
   --    b2Vec2 linear;
   --    float angularA;
   --    float angularB;
   --  };
   --

   type b2Jacobian is
      record
         linear    : b2Vec2;
         angularA,
         angularB  : Real;
      end record;





   --  A joint edge is used to connect bodies and joints together
   --  in a joint graph where each body is a node and each joint
   --  is an edge. A joint edge belongs to a doubly linked list
   --  maintained in each attached body. Each joint has two joint
   --  nodes, one for each attached body.
   --
   --  struct b2JointEdge
   --  {
   --    b2Body* other;       ///< provides quick access to the other body attached.
   --    b2Joint* joint;         ///< the joint
   --    b2JointEdge* prev;      ///< the previous joint edge in the body's joint list
   --    b2JointEdge* next;      ///< the next joint edge in the body's joint list
   --  };
   --

   type b2JointEdge is
      record
         other : access b2_Body.b2Body;     -- Provides quick access to the other body attached.
         joint : access b2Joint;            -- The joint.
         prev,                              -- The previous joint edge in the body's joint list.
         next  : access b2JointEdge;        -- The next joint edge in the body's joint list.
      end record;






   --  Joint definitions are used to construct joints.
   --
   --  struct b2JointDef
   --  {
   --    b2JointDef()
   --    {
   --       type = e_unknownJoint;
   --       bodyA = nullptr;
   --       bodyB = nullptr;
   --       collideConnected = false;
   --    }
   --
   --    The joint type is set automatically for concrete joint types.
   --    b2JointType type;
   --
   --    Use this to attach application specific data to your joints.
   --    b2JointUserData userData;
   --
   --    The first attached body.
   --    b2Body* bodyA;
   --
   --    The second attached body.
   --    b2Body* bodyB;
   --
   --    Set this flag to true if the attached bodies should collide.
   --    bool collideConnected;
   --  };
   --

   type b2JointDef is tagged
      record
         Kind             :        b2JointType    := e_unknownJoint;     -- The joint type is set automatically for concrete joint types.
         userData         :        b2JointUserData;                      -- Use this to attach application specific data to your joints.
         bodyA,                                                          -- The first attached body.
         bodyB            : access b2_Body.b2Body;                       -- The second attached body.
         collideConnected :        Boolean        := False;              -- Set this flag to true if the attached bodies should collide.
      end record;





   --  Utility to compute linear stiffness values from frequency and damping ratio.
   --
   --  void b2LinearStiffness (float& stiffness, float& damping,
   --                          float frequencyHertz, float dampingRatio,
   --                          const b2Body* bodyA, const b2Body* bodyB);
   --

   procedure b2LinearStiffness (stiffness      :    out Real;
                                damping        :    out Real;
                                frequencyHertz : in     Real;
                                dampingRatio   : in     Real;
                                bodyA,
                                bodyB          : in     b2_Body.b2Body);



   --  Utility to compute rotational stiffness values frequency and damping ratio.
   --
   --  void b2AngularStiffness (float& stiffness, float& damping,
   --                           float frequencyHertz, float dampingRatio,
   --                           const b2Body* bodyA, const b2Body* bodyB);
   --

   procedure b2AngularStiffness (stiffness      :    out Real;
                                 damping        :    out Real;
                                 frequencyHertz : in     Real;
                                 dampingRatio   : in     Real;
                                 bodyA,
                                 bodyB          : in     b2_Body.b2Body);




   --  public:
   --


   --    Get the type of the concrete joint.
   --    b2JointType GetType() const;
   --

   function getType (Self : in b2Joint) return b2JointType
     with inline;



   --    Get the first body attached to this joint.
   --    b2Body* GetBodyA();
   --

   function getBodyA (Self : in b2Joint) return access b2_Body.b2Body
     with inline;



   --    Get the second body attached to this joint.
   --    b2Body* GetBodyB();
   --

   function getBodyB (Self : in b2Joint) return access b2_Body.b2Body
     with inline;



   --    Get the anchor point on bodyA in world coordinates.
   --    virtual b2Vec2 GetAnchorA() const = 0;
   --

   function getAnchorA (Self : in b2Joint) return b2Vec2
                        is abstract;




   --    Get the anchor point on bodyB in world coordinates.
   --    virtual b2Vec2 GetAnchorB() const = 0;
   --

   function getAnchorB (Self : in b2Joint) return b2Vec2
                        is abstract;



   --    Get the reaction force on bodyB at the joint anchor in Newtons.
   --    virtual b2Vec2 GetReactionForce(float inv_dt) const = 0;
   --

   function getReactionForce (Self : in b2Joint;   inv_dt : in Real) return b2Vec2
                              is abstract;




   --    Get the reaction torque on bodyB in N*m.
   --    virtual float GetReactionTorque(float inv_dt) const = 0;
   --

   function getReactionTorque (Self : in b2Joint;   inv_dt : in Real) return Real
                               is abstract;




   --    Get the next joint the world joint list.
   --    b2Joint* GetNext();
   --    const b2Joint* GetNext() const;
   --

   function getNext (Self : in b2Joint) return access b2Joint'Class
     with inline;




   --    Get the user data pointer.
   --    b2JointUserData& GetUserData();
   --

   function getUserData (Self : in b2Joint) return b2JointUserData
     with inline;




   --    Short-cut function to determine if either body is enabled.
   --    bool IsEnabled() const;
   --

   function isEnabled (Self : in b2Joint) return Boolean;




   --    Get collide connected.
   --    Note: modifying the collide connect flag won't work correctly because
   --    the flag is only checked when fixture AABBs begin to overlap.
   --    bool GetCollideConnected() const;
   --

   function getCollideConnected (Self : in b2Joint) return Boolean
     with inline;




   --    Dump this joint to the log file.
   --    virtual void Dump() { b2Dump("// Dump is not supported for this joint type.\n"); }
   --

   procedure dump (Self : in b2Joint);




   --    Shift the origin for any points stored in world coordinates.
   --    virtual void ShiftOrigin(const b2Vec2& newOrigin) { B2_NOT_USED(newOrigin);  }
   --

   procedure shiftOrigin (Self : in out b2Joint;   newOrigin : in b2Vec2) is null;



   --    Debug draw this joint
   --    virtual void Draw(b2Draw* draw) const;
   --

   procedure draw_any (Self : in b2Joint'Class;   draw : access b2Draw'Class);
   procedure draw     (Self : in b2Joint;         draw : access b2Draw'Class) is null;





   --------------
   --  protected:


   --    friend class b2World;
   --    friend class b2Body;
   --    friend class b2Island;
   --    friend class b2GearJoint;
   --


   --    static b2Joint* Create(const b2JointDef* def, b2BlockAllocator* allocator);
   --

   function create (def : in b2JointDef'Class) return access b2Joint'Class;



   --    static void Destroy(b2Joint* joint, b2BlockAllocator* allocator);
   --

   procedure destroy (joint : access b2Joint'Class);



   --    b2Joint(const b2JointDef* def);
   --

   package Forge
   is
      procedure define (Self : out b2Joint'Class;   def : in b2JointDef'Class);
   end Forge;




   --    virtual ~b2Joint() {}
   --

   procedure destruct (Self : in out b2Joint) is null;



   --    virtual void InitVelocityConstraints(const b2SolverData& data) = 0;
   --

   procedure initVelocityConstraints (Self : in out b2Joint;   data : in b2SolverData)
   is abstract;



   --    virtual void SolveVelocityConstraints(const b2SolverData& data) = 0;
   --

   procedure solveVelocityConstraints (Self : in out b2Joint;   data : in b2SolverData)
   is abstract;



   --    // This returns true if the position errors are within tolerance.
   --
   --    virtual bool SolvePositionConstraints(const b2SolverData& data) = 0;
   --

   function solvePositionConstraints (Self : in out b2Joint;   data : in b2SolverData) return Boolean
   is abstract;



   procedure m_prev_is (Self : in out b2Joint;   Now : in b2Joint_ptr)
     with inline;

   procedure m_next_is (Self : in out b2Joint;   Now : in b2Joint_ptr)
     with inline;


   function  m_prev    (Self : access b2Joint) return b2Joint_ptr
     with inline;

   function  m_next    (Self : access b2Joint) return b2Joint_ptr
     with inline;


   function  m_edgeA   (Self : access b2Joint) return access b2JointEdge
     with inline;

   function  m_edgeB   (Self : access b2Joint) return access b2JointEdge
     with inline;


   function  m_islandFlag    (Self : in     b2Joint)     return Boolean
     with inline;

   procedure m_islandFlag_is (Self : in out b2Joint;   Now : in Boolean)
     with inline;



   --  inline b2JointType b2Joint::GetType() const
   --  {
   --    return m_type;
   --  }
   --
   --  inline b2Body* b2Joint::GetBodyA()
   --  {
   --    return m_bodyA;
   --  }
   --
   --  inline b2Body* b2Joint::GetBodyB()
   --  {
   --    return m_bodyB;
   --  }
   --
   --  inline b2Joint* b2Joint::GetNext()
   --  {
   --    return m_next;
   --  }
   --
   --  inline const b2Joint* b2Joint::GetNext() const
   --  {
   --    return m_next;
   --  }
   --
   --  inline b2JointUserData& b2Joint::GetUserData()
   --  {
   --    return m_userData;
   --  }
   --
   --  inline bool b2Joint::GetCollideConnected() const
   --  {
   --    return m_collideConnected;
   --  }
   --




private

   --    b2JointType m_type;
   --    b2Joint* m_prev;
   --    b2Joint* m_next;
   --    b2JointEdge m_edgeA;
   --    b2JointEdge m_edgeB;
   --    b2Body* m_bodyA;
   --    b2Body* m_bodyB;
   --
   --    int32 m_index;
   --
   --    bool m_islandFlag;
   --    bool m_collideConnected;
   --
   --    b2JointUserData m_userData;
   --

   type b2Joint is abstract tagged
      record
         m_type             : b2JointType;

         m_prev,
         m_next             : access  b2Joint;

         m_edgeA,
         m_edgeB            : aliased b2JointEdge;

         m_bodyA,
         m_bodyB            : access  b2_Body.b2Body;

         m_index            : Natural;

         m_islandFlag       : Boolean;
         m_collideConnected : Boolean;

         m_userData         : b2JointUserData;
      end record;


end box2d.b2_Joint;
