with
     box2d.b2_Collision,
     box2d.b2_Fixture,
     box2d.b2_Body,
     box2d.b2_world_Callbacks,
     box2d.b2_Math,
     box2d.b2_Shape,
     box2d.b2_Settings,

     Interfaces;


package box2d.b2_Contact
is
   use b2_Collision,
       b2_Body,
       b2_world_Callbacks,
       b2_Shape,
       b2_Fixture,
       b2_Math,
       b2_Settings,

       Interfaces;

   --  class b2Body;
   --  class b2Contact;
   --  class b2Fixture;
   --  class b2World;
   --  class b2BlockAllocator;
   --  class b2StackAllocator;
   --  class b2ContactListener;
   --


   type b2Contact     is abstract tagged private;
   type b2Contact_ptr is access all b2Contact'Class;


   --  Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
   --  For example, anything slides on ice.
   --
   --  inline float b2MixFriction(float friction1, float friction2)
   --  {
   --    return b2Sqrt(friction1 * friction2);
   --  }
   --

   function b2MixFriction (friction1,
                           friction2 : in Real) return Real
     with inline;




   --  Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
   --  For example, a superball bounces on anything.
   --
   --  inline float b2MixRestitution(float restitution1, float restitution2)
   --  {
   --    return restitution1 > restitution2 ? restitution1 : restitution2;
   --  }
   --

   function b2MixRestitution (restitution1,
                              restitution2 : in Real) return Real
     with inline;





   --  Restitution mixing law. This picks the lowest value.
   --

   function b2mixRestitutionThreshold (threshold1,
                                       threshold2 : in Real) return Real
     with inline;





   --  typedef b2Contact* b2ContactCreateFcn(   b2Fixture* fixtureA, int32 indexA,
   --                               b2Fixture* fixtureB, int32 indexB,
   --                               b2BlockAllocator* allocator);
   --

   type b2ContactCreateFcn is access function (fixtureA : in out b2Fixture;   indexA : in Natural;
                                               fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class;



   --  typedef void b2ContactDestroyFcn(b2Contact* contact, b2BlockAllocator* allocator);
   --

   type b2ContactDestroyFcn is access procedure (Contact : access b2Contact'Class);





   --  struct b2ContactRegister
   --  {
   --    b2ContactCreateFcn* createFcn;
   --    b2ContactDestroyFcn* destroyFcn;
   --    bool primary;
   --  };
   --

   type b2ContactRegister is
      record
         createFcn  : b2ContactCreateFcn;
         destroyFcn : b2ContactDestroyFcn;
         primary    : Boolean;
      end record;



   --  A contact edge is used to connect bodies and contacts together
   --  in a contact graph where each body is a node and each contact
   --  is an edge. A contact edge belongs to a doubly linked list
   --  maintained in each attached body. Each contact has two contact
   --  nodes, one for each attached body.
   --
   --  struct b2ContactEdge
   --  {
   --    b2Body* other;       ///< provides quick access to the other body attached.
   --    b2Contact* contact;  ///< the contact
   --    b2ContactEdge* prev; ///< the previous contact edge in the body's contact list
   --    b2ContactEdge* next; ///< the next contact edge in the body's contact list
   --  };
   --

   type b2ContactEdge is
      record
         Other   : access b2Body;            -- Provides quick access to the other body attached.
         Contact : access b2Contact;         -- The contact.
         prev,                               -- The previous contact edge in the body's contact list.
         next    : access b2ContactEdge;     -- The next contact edge in the body's contact list.
      end record;





   ------------
   -- b2Contact
   --

   --  The class manages contact between two shapes. A contact exists for each overlapping
   --  AABB in the broad-phase (except if filtered). Therefore a contact object may exist
   --  that has no contact points.
   --
   --  class b2Contact
   --  {
   --  public:
   --


   --    Get the contact manifold. Do not modify the manifold unless you understand the
   --    internals of Box2D.
   --
   --    b2Manifold* GetManifold();
   --    const b2Manifold* GetManifold() const;
   --

   function getManifold (Self : access b2Contact) return access b2Manifold   with inline;
   function getManifold (Self : in     b2Contact) return        b2Manifold   with inline;



   --    Get the world manifold.
   --    void GetWorldManifold(b2WorldManifold* worldManifold) const;
   --

   procedure getWorldManifold (Self : access b2Contact;   worldManifold : access b2WorldManifold)
     with inline;



   --    Is this contact touching?
   --    bool IsTouching() const;
   --

   function isTouching (Self : in b2Contact) return Boolean
     with inline;



   --    Enable/disable this contact. This can be used inside the pre-solve
   --    contact listener. The contact is only disabled for the current
   --    time step (or sub-step in continuous collisions).
   --    void SetEnabled(bool flag);
   --

   procedure setEnabled (Self : in out b2Contact;   Flag : in Boolean)
     with inline;



   --    Has this contact been disabled?
   --    bool IsEnabled() const;
   --

   function isEnabled (Self : in b2Contact) return Boolean
     with inline;



   --    Get the next contact in the world's contact list.
   --    b2Contact* GetNext();
   --    const b2Contact* GetNext() const;
   --

   function getNext (Self : access b2Contact) return access          b2Contact'Class   with inline;
   --  function getNext (Self : in     b2Contact) return access constant b2Contact'Class   with inline;



   --    Get fixture A in this contact.
   --
   --    b2Fixture* GetFixtureA();
   --    const b2Fixture* GetFixtureA() const;
   --

   function getFixtureA (Self : access b2Contact) return access b2Fixture   with inline;
   function getFixtureA (Self : in     b2Contact) return        b2Fixture   with inline;



   --    Get the child primitive index for fixture A.
   --    int32 GetChildIndexA() const;
   --

   function getChildIndexA (Self : in  b2Contact) return Natural
     with inline;




   --    Get fixture B in this contact.
   --    b2Fixture* GetFixtureB();
   --    const b2Fixture* GetFixtureB() const;
   --

   function getFixtureB (Self : access b2Contact) return access b2Fixture   with inline;
   function getFixtureB (Self : in     b2Contact) return        b2Fixture   with inline;



   --    Get the child primitive index for fixture B.
   --    int32 GetChildIndexB() const;
   --

   function getChildIndexB (Self : in b2Contact) return Natural
     with inline;



   --    Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
   --    This value persists until set or reset.
   --
   --    void SetFriction(float friction);
   --

   procedure setFriction (Self : in out b2Contact;   friction : in Real)
     with inline;



   --    Get the friction.
   --    float GetFriction() const;
   --

   function getFriction (Self : in b2Contact) return Real
     with inline;



   --    Reset the friction mixture to the default value.
   --    void ResetFriction();
   --

   procedure resetFriction (Self : in out b2Contact)
     with inline;



   --    Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
   --    The value persists until you set or reset.
   --
   --    void SetRestitution(float restitution);
   --

   procedure setRestitution (Self : in out b2Contact;   restitution : in Real)
     with inline;



   --    Get the restitution.
   --    float GetRestitution() const;
   --

   function getRestitution (Self : in b2Contact) return Real
     with inline;



   --    Reset the restitution to the default value.
   --    void ResetRestitution();
   --

   procedure resetRestitution (Self : in out b2Contact)
     with inline;



   --    Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
   --    The value persists until you set or reset.
   --
   --    void SetRestitutionThreshold(float threshold);
   --

   procedure setRestitutionThreshold (Self : in out b2Contact;   threshold : in Real);



   --    Get the restitution threshold.
   --    float GetRestitutionThreshold() const;
   --

   function getRestitutionThreshold (Self : in b2Contact) return Real
     with inline;



   --    Reset the restitution threshold to the default value.
   --    void ResetRestitutionThreshold();
   --

   procedure resetRestitutionThreshold (Self : in out b2Contact)
     with inline;



   --    Set the desired tangent speed for a conveyor belt behavior. In meters per second.
   --    void SetTangentSpeed(float speed);
   --

   procedure setTangentSpeed (Self : in out b2Contact;   speed : in Real)
     with inline;



   --    Get the desired tangent speed. In meters per second.
   --    float GetTangentSpeed() const;
   --

   function getTangentSpeed (Self : in b2Contact) return Real
     with inline;



   --    Evaluate this contact with your own manifold and transforms.
   --
   --    virtual void Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB) = 0;
   --

   procedure evaluate (Self : in out b2Contact;   manifold : in out b2Manifold;
                                                  xfA, xfB : in     b2Transform) is abstract;




   ------------------------------
   -- Protected C++ declarations.
   --

   --  protected:
   --    friend class b2ContactManager;
   --    friend class b2World;
   --    friend class b2ContactSolver;
   --    friend class b2Body;
   --    friend class b2Fixture;
   --



   --    // Flags stored in m_flags
   --    enum
   --    {
   --       // Used when crawling contact graph when forming islands.
   --       e_islandFlag      = 0x0001,
   --
   --       // Set when the shapes are touching.
   --       e_touchingFlag    = 0x0002,
   --
   --       // This contact can be disabled (by user)
   --       e_enabledFlag     = 0x0004,
   --
   --       // This contact needs filtering because a fixture filter was changed.
   --       e_filterFlag      = 0x0008,
   --
   --       // This bullet contact had a TOI event
   --       e_bulletHitFlag      = 0x0010,
   --
   --       // This contact has a valid TOI in m_toi
   --       e_toiFlag         = 0x0020
   --    };
   --

   -- Flags stored in m_flags.
   --

   subtype Flag is Unsigned_32;

   e_islandFlag    : constant Flag := 16#0001#;     -- Used when crawling contact graph when forming islands.
   e_touchingFlag  : constant Flag := 16#0002#;     -- Set when the shapes are touching.
   e_enabledFlag   : constant Flag := 16#0004#;     -- This contact can be disabled (by user).
   e_filterFlag    : constant Flag := 16#0008#;     -- This contact needs filtering because a fixture filter was changed.
   e_bulletHitFlag : constant Flag := 16#0010#;     -- This bullet contact had a TOI event.
   e_toiFlag       : constant Flag := 16#0020#;     -- This contact has a valid TOI in m_toi.

   subtype flag_Set is Unsigned_32;




   --    Flag this contact for filtering. Filtering will occur the next time step.
   --
   --    void FlagForFiltering();
   --

   procedure flagForFiltering (Self : in out b2Contact)
     with inline;



   --    static void AddType(b2ContactCreateFcn* createFcn, b2ContactDestroyFcn* destroyFcn,
   --                   b2Shape::Type typeA, b2Shape::Type typeB);

   procedure addType (createFcn  : b2ContactCreateFcn;
                      destroyFcn : b2ContactDestroyFcn;
                      typeA      : b2_Shape.shape_Type;
                      typeB      : b2_Shape.shape_Type);


   --    static void InitializeRegisters();
   --

   procedure initializeRegisters;



   --    static b2Contact* Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator);
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class;



   --    static void Destroy(b2Contact* contact, b2Shape::Type typeA, b2Shape::Type typeB, b2BlockAllocator* allocator);
   --

   procedure destroy (contact      : access b2Contact;
                      typeA, typeB : in     b2_Shape.shape_Type);



   --    static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);
   --

   procedure destroy (contact : access b2Contact);




   --    b2Contact() : m_fixtureA(nullptr), m_fixtureB(nullptr) {}
   --
   -- Setting fixtures to null is Ada default.



   --    b2Contact(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB);
   --

   procedure define (Self : out b2Contact;   fixtureA : access b2Fixture;   indexA : in Natural;
                                             fixtureB : access b2Fixture;   indexB : in Natural);



   --    void Update(b2ContactListener* listener);
   --

   procedure update (Self : in out b2Contact'Class;   listener : access b2ContactListener'Class);






   ------------------------------------
   --- Protected C++ members/functions.
   --

   function  m_Prev    (Self : in     b2Contact)  return access b2Contact'Class;
   procedure m_Prev_is (Self : in out b2Contact;   Now : in     b2Contact_ptr);

   function  m_Next    (Self : in     b2Contact)  return access b2Contact'Class;
   procedure m_Next_is (Self : in out b2Contact;   Now : in     b2Contact_ptr);

   function  m_NodeA    (Self : access b2Contact) return access b2ContactEdge;
   procedure m_NodeA_is (Self : in out b2Contact;  Now : in     b2ContactEdge);

   function  m_NodeB    (Self : access b2Contact) return access b2ContactEdge;
   procedure m_NodeB_is (Self : in out b2Contact;  Now : in     b2ContactEdge);

   function  m_Flags    (Self : in     b2Contact)    return flag_Set;
   procedure m_Flags_is (Self : in out b2Contact;  Now : in flag_Set);

   function  m_toi         (Self : in     b2Contact)    return Real;
   procedure m_toi_is      (Self : in out b2Contact;  Now : in Real);


   function  m_toiCount    (Self : in     b2Contact)    return Natural;
   procedure m_toiCount_is (Self : in out b2Contact;  Now : in Natural);



private

   --    static b2ContactRegister s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
   --    static bool s_initialized;
   --
   s_registers   : array (0 .. Shape_Type'Pos (e_typeCount) - 1,
                          0 .. Shape_Type'Pos (e_typeCount) - 1) of b2ContactRegister;
   s_initialized : Boolean := False;



   --    uint32 m_flags;
   --
   --    // World pool and list pointers.
   --    b2Contact* m_prev;
   --    b2Contact* m_next;
   --
   --    // Nodes for connecting bodies.
   --    b2ContactEdge m_nodeA;
   --    b2ContactEdge m_nodeB;
   --
   --    b2Fixture* m_fixtureA;
   --    b2Fixture* m_fixtureB;
   --
   --    int32 m_indexA;
   --    int32 m_indexB;
   --
   --    b2Manifold m_manifold;
   --
   --    int32 m_toiCount;
   --    float m_toi;
   --
   --    float m_friction;
   --    float m_restitution;
   --    float m_restitutionThreshold;
   --
   --    float m_tangentSpeed;

   type b2Contact is abstract tagged
      record
         m_flags : flag_Set;

         -- World pool and list pointers.
         --
         m_prev,
         m_Next  : access b2Contact;

         -- Nodes for connecting bodies.
         --
         m_nodeA,
         m_nodeB : aliased b2ContactEdge;

         m_fixtureA,
         m_fixtureB : access b2Fixture;

         m_indexA,
         m_indexB   : Natural;

         m_manifold : aliased b2Manifold;

         m_toiCount : Natural;
         m_toi      : Real;

         m_friction,
         m_restitution,
         m_restitutionThreshold,
         m_tangentSpeed        : Real;
      end record;


end box2d.b2_Contact;
