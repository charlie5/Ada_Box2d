with
     b2_chain_circle_Contact,
     b2_chain_polygon_Contact,
     b2_circle_Contact,
     b2_contact_Solver,
     b2_edge_circle_Contact,
     b2_edge_polygon_Contact,
     b2_polygon_circle_Contact,
     b2_polygon_Contact,
     b2_Contact,
     b2_Collision.overlap,
     b2_Time_of_impact,
     b2_World;


package body b2_Contact
is
   --  b2ContactRegister b2Contact::s_registers[b2Shape::e_typeCount][b2Shape::e_typeCount];
   --  bool b2Contact::s_initialized = false;
   --





   --  Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
   --  For example, anything slides on ice.
   --
   --  inline float b2MixFriction(float friction1, float friction2)
   --  {
   --    return b2Sqrt(friction1 * friction2);
   --  }
   --

   function b2MixFriction (friction1, friction2 : in Real) return Real
   is
   begin
      return b2Sqrt (friction1 * friction2);
   end b2MixFriction;



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
   is
   begin
      return (if restitution1 > restitution2 then restitution1
                                             else restitution2);
   end b2MixRestitution;



   --  Restitution mixing law. This picks the lowest value.
   --  inline float b2MixRestitutionThreshold(float threshold1, float threshold2)
   --  {
   --    return threshold1 < threshold2 ? threshold1 : threshold2;
   --  }
   --

   function b2mixRestitutionThreshold (threshold1,
                                       threshold2 : in Real) return Real
   is
   begin
      return (if threshold1 < threshold2 then threshold1 else threshold2);
   end b2mixRestitutionThreshold;





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



   --  inline b2Manifold* b2Contact::GetManifold()
   --  {
   --    return &m_manifold;
   --  }
   --

   function getManifold (Self : access b2Contact) return access b2Manifold
   is
   begin
      return Self.m_manifold'Access;
   end getManifold;



   --  inline const b2Manifold* b2Contact::GetManifold() const
   --  {
   --    return &m_manifold;
   --  }
   --

   function getManifold (Self : in b2Contact) return b2Manifold
   is
   begin
      return Self.m_manifold;
   end getManifold;



   --  inline void b2Contact::GetWorldManifold(b2WorldManifold* worldManifold) const
   --  {
   --    const b2Body* bodyA = m_fixtureA->GetBody();
   --    const b2Body* bodyB = m_fixtureB->GetBody();
   --    const b2Shape* shapeA = m_fixtureA->GetShape();
   --    const b2Shape* shapeB = m_fixtureB->GetShape();
   --
   --    worldManifold->Initialize(&m_manifold, bodyA->GetTransform(), shapeA->m_radius, bodyB->GetTransform(), shapeB->m_radius);
   --  }
   --

   procedure getWorldManifold (Self : access b2Contact;   worldManifold : access b2WorldManifold)
   is
      bodyA  : constant b2Body      := Self.m_fixtureA.getBody;
      bodyB  : constant b2Body      := Self.m_fixtureB.getBody;
      shapeA : constant b2Shape_ptr := Self.m_fixtureA.getShape;
      shapeB : constant b2Shape_ptr := Self.m_fixtureB.getShape;
   begin
      initialize (worldManifold.all, Self.m_manifold,
                                     bodyA.getTransform, shapeA.m_radius,
                                     bodyB.getTransform, shapeB.m_radius);
   end getWorldManifold;




   --  inline bool b2Contact::IsTouching() const
   --  {
   --    return (m_flags & e_touchingFlag) == e_touchingFlag;
   --  }
   --

   function isTouching (Self : in b2Contact) return Boolean
   is
   begin
      return (Self.m_flags and e_touchingFlag) = e_touchingFlag;
   end isTouching;




   --    Enable/disable this contact. This can be used inside the pre-solve
   --    contact listener. The contact is only disabled for the current
   --    time step (or sub-step in continuous collisions).
   --    void SetEnabled(bool flag);
   --
   --  inline void b2Contact::SetEnabled(bool flag)
   --  {
   --    if (flag)
   --    {
   --       m_flags |= e_enabledFlag;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_enabledFlag;
   --    }
   --  }
   --

   procedure setEnabled (Self : in out b2Contact;   Flag : in Boolean)
   is
   begin
      if flag
      then
         Self.m_flags := Self.m_flags or e_enabledFlag;
      else
         Self.m_flags := Self.m_flags and (not e_enabledFlag);
      end if;
   end setEnabled;




   --    Has this contact been disabled?
   --
   --  inline bool b2Contact::IsEnabled() const
   --  {
   --    return (m_flags & e_enabledFlag) == e_enabledFlag;
   --  }
   --

   function isEnabled (Self : in b2Contact) return Boolean
   is
   begin
      return (Self.m_flags and e_enabledFlag) = e_enabledFlag;
   end isEnabled;




   --    Get the next contact in the world's contact list.
   --

   --  inline b2Contact* b2Contact::GetNext()
   --  {
   --    return m_next;
   --  }
   --

   function getNext (Self : access b2Contact) return access b2Contact'Class
   is
   begin
      return Self.m_next;
   end getNext;




   --  inline const b2Contact* b2Contact::GetNext() const
   --  {
   --    return m_next;
   --  }
   --

   --  function getNext (Self : in b2Contact) return access constant b2Contact'Class
   --  is
   --  begin
   --     return Self.m_next;
   --  end getNext;



   --    Get fixture A in this contact.
   --

   --  inline b2Fixture* b2Contact::GetFixtureA()
   --  {
   --    return m_fixtureA;
   --  }
   --

   function getFixtureA (Self : access b2Contact) return access b2Fixture
   is
   begin
      return Self.m_fixtureA;
   end getFixtureA;



   --  inline const b2Fixture* b2Contact::GetFixtureA() const
   --  {
   --    return m_fixtureA;
   --  }
   --

   function getFixtureA (Self : in     b2Contact) return        b2Fixture
   is
   begin
      return Self.m_fixtureA.all;
   end getFixtureA;



   --    Get the child primitive index for fixture A.
   --
   --  inline int32 b2Contact::GetChildIndexA() const
   --  {
   --    return m_indexA;
   --  }
   --

   function getChildIndexA (Self : in  b2Contact) return Natural
   is
   begin
      return Self.m_indexA;
   end getChildIndexA;





   --    Get fixture B in this contact.
   --

   --  inline b2Fixture* b2Contact::GetFixtureB()
   --  {
   --    return m_fixtureB;
   --  }
   --

   function getFixtureB (Self : access b2Contact) return access b2Fixture
   is
   begin
      return Self.m_fixtureB;
   end getFixtureB;



   --  inline const b2Fixture* b2Contact::GetFixtureB() const
   --  {
   --    return m_fixtureB;
   --  }
   --

   function getFixtureB (Self : in b2Contact) return b2Fixture
   is
   begin
      return Self.m_fixtureB.all;
   end getFixtureB;



   --    Get the child primitive index for fixture B.
   --
   --  inline int32 b2Contact::GetChildIndexB() const
   --  {
   --    return m_indexB;
   --  }
   --

   function getChildIndexB (Self : in b2Contact) return Natural
   is
   begin
      return Self.m_indexB;
   end getChildIndexB;




   --    Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
   --    This value persists until set or reset.
   --
   --  inline void b2Contact::SetFriction(float friction)
   --  {
   --    m_friction = friction;
   --  }
   --

   procedure setFriction (Self : in out b2Contact;   friction : in Real)
   is
   begin
      Self.m_friction := friction;
   end setFriction;




   --    Get the friction.
   --
   --  inline float b2Contact::GetFriction() const
   --  {
   --    return m_friction;
   --  }
   --

   function getFriction (Self : in b2Contact) return Real
   is
   begin
      return Self.m_friction;
   end getFriction;




   --    Reset the friction mixture to the default value.
   --
   --  inline void b2Contact::ResetFriction()
   --  {
   --    m_friction = b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
   --  }
   --

   procedure resetFriction (Self : in out b2Contact)
   is
   begin
      Self.m_friction := b2mixFriction (Self.m_fixtureA.getFriction,
                                        Self.m_fixtureB.getFriction);
   end resetFriction;




   --    Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
   --    The value persists until you set or reset.
   --
   --  inline void b2Contact::SetRestitution(float restitution)
   --  {
   --    m_restitution = restitution;
   --  }
   --

   procedure setRestitution (Self : in out b2Contact;   restitution : in Real)
   is
   begin
      Self.m_restitution := restitution;
   end setRestitution;




   --    Get the restitution.
   --
   --  inline float b2Contact::GetRestitution() const
   --  {
   --    return m_restitution;
   --  }
   --

   function getRestitution (Self : in b2Contact) return Real
   is
   begin
      return Self.m_restitution;
   end getRestitution;




   --    Reset the restitution to the default value.
   --
   --  inline void b2Contact::ResetRestitution()
   --  {
   --    m_restitution = b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
   --  }
   --

   procedure resetRestitution (Self : in out b2Contact)
   is
   begin
      Self.m_restitution := b2mixRestitution (Self.m_fixtureA.getRestitution,
                                              Self.m_fixtureB.getRestitution);
   end resetRestitution;




   --    Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
   --    The value persists until you set or reset.
   --

   --  inline void b2Contact::SetRestitutionThreshold(float threshold)
   --  {
   --    m_restitutionThreshold = threshold;
   --  }
   --

   procedure setRestitutionThreshold (Self : in out b2Contact;   threshold : in Real)
   is
   begin
      Self.m_restitutionThreshold := threshold;
   end setRestitutionThreshold;




   --    Get the restitution threshold.
   --
   --  inline float b2Contact::GetRestitutionThreshold() const
   --  {
   --    return m_restitutionThreshold;
   --  }
   --

   function getRestitutionThreshold (Self : in b2Contact) return Real
   is
   begin
      return Self.m_restitutionThreshold;
   end getRestitutionThreshold;




   --    Reset the restitution threshold to the default value.
   --
   --  inline void b2Contact::ResetRestitutionThreshold()
   --  {
   --    m_restitutionThreshold = b2MixRestitutionThreshold(m_fixtureA->m_restitutionThreshold, m_fixtureB->m_restitutionThreshold);
   --  }
   --

   procedure resetRestitutionThreshold (Self : in out b2Contact)
   is
   begin
      Self.m_restitutionThreshold := b2mixRestitutionThreshold (Self.m_fixtureA.getRestitutionThreshold,
                                                                Self.m_fixtureB.getRestitutionThreshold);
   end resetRestitutionThreshold;




   --    Set the desired tangent speed for a conveyor belt behavior. In meters per second.
   --

   --  inline void b2Contact::SetTangentSpeed(float speed)
   --  {
   --    m_tangentSpeed = speed;
   --  }
   --

   procedure setTangentSpeed (Self : in out b2Contact;   speed : in Real)
   is
   begin
      Self.m_tangentSpeed := speed;
   end setTangentSpeed;




   --    Get the desired tangent speed. In meters per second.
   --
   --  inline float b2Contact::GetTangentSpeed() const
   --  {
   --    return m_tangentSpeed;
   --  }
   --

   function getTangentSpeed (Self : in b2Contact) return Real
   is
   begin
      return Self.m_tangentSpeed;
   end getTangentSpeed;





   ------------------------------
   -- Protected C++ declarations.
   --


   --    Flag this contact for filtering. Filtering will occur the next time step.
   --
   --  inline void b2Contact::FlagForFiltering()
   --  {
   --    m_flags |= e_filterFlag;
   --  }
   --

   procedure flagForFiltering (Self : in out b2Contact)
   is
   begin
      Self.m_flags := Self.m_flags or e_filterFlag;
   end flagForFiltering;




   --  void b2Contact::AddType (b2ContactCreateFcn*  createFcn,
   --                           b2ContactDestroyFcn* destoryFcn,
   --                           b2Shape::Type type1, b2Shape::Type type2)
   --  {
   --    b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
   --    b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
   --
   --    s_registers[type1][type2].createFcn = createFcn;
   --    s_registers[type1][type2].destroyFcn = destoryFcn;
   --    s_registers[type1][type2].primary = true;
   --
   --    if (type1 != type2)
   --    {
   --       s_registers[type2][type1].createFcn = createFcn;
   --       s_registers[type2][type1].destroyFcn = destoryFcn;
   --       s_registers[type2][type1].primary = false;
   --    }
   --  }
   --

   procedure addType (createFcn  : b2ContactCreateFcn;
                      destroyFcn : b2ContactDestroyFcn;
                      typeA      : b2_Shape.shape_Type;
                      typeB      : b2_Shape.shape_Type)
   is
      typeA_Pos : constant Natural := b2_Shape.shape_Type'Pos (typeA);
      typeB_Pos : constant Natural := b2_Shape.shape_Type'Pos (typeB);

      pragma assert (0 <= typeA_Pos and typeA < b2_Shape.e_typeCount);
      pragma assert (0 <= typeB_Pos and typeB < b2_Shape.e_typeCount);
   begin
      s_registers (typeA_Pos, typeB_Pos).createFcn  := createFcn;
      s_registers (typeA_Pos, typeB_Pos).destroyFcn := destroyFcn;
      s_registers (typeA_Pos, typeB_Pos).primary    := true;

      if typeA /= typeB
      then
         s_registers (typeB_Pos, typeA_Pos).createFcn  := createFcn;
         s_registers (typeB_Pos, typeA_Pos).destroyFcn := destroyFcn;
         s_registers (typeB_Pos, typeA_Pos).primary    := False;
      end if;
   end addType;




   --  void b2Contact::InitializeRegisters()
   --  {
   --    AddType(b2CircleContact::Create, b2CircleContact::Destroy, b2Shape::e_circle, b2Shape::e_circle);
   --    AddType(b2PolygonAndCircleContact::Create, b2PolygonAndCircleContact::Destroy, b2Shape::e_polygon, b2Shape::e_circle);
   --    AddType(b2PolygonContact::Create, b2PolygonContact::Destroy, b2Shape::e_polygon, b2Shape::e_polygon);
   --    AddType(b2EdgeAndCircleContact::Create, b2EdgeAndCircleContact::Destroy, b2Shape::e_edge, b2Shape::e_circle);
   --    AddType(b2EdgeAndPolygonContact::Create, b2EdgeAndPolygonContact::Destroy, b2Shape::e_edge, b2Shape::e_polygon);
   --    AddType(b2ChainAndCircleContact::Create, b2ChainAndCircleContact::Destroy, b2Shape::e_chain, b2Shape::e_circle);
   --    AddType(b2ChainAndPolygonContact::Create, b2ChainAndPolygonContact::Destroy, b2Shape::e_chain, b2Shape::e_polygon);
   --  }
   --

   procedure initializeRegisters
   is
   begin
      addType (b2_circle_Contact        .create'Access,  b2_circle_Contact        .destroy'Access,  b2_Shape.e_circle,   b2_Shape.e_circle);
      addType (b2_polygon_circle_Contact.create'Access,  b2_polygon_circle_Contact.destroy'Access,  b2_Shape.e_polygon,  b2_Shape.e_circle);
      addType (b2_polygon_Contact       .create'Access,  b2_polygon_Contact       .destroy'Access,  b2_Shape.e_polygon,  b2_Shape.e_polygon);
      addType (b2_edge_circle_Contact   .create'Access,  b2_edge_circle_Contact   .destroy'Access,  b2_Shape.e_edge,     b2_Shape.e_circle);
      addType (b2_edge_polygon_Contact  .create'Access,  b2_edge_polygon_Contact  .destroy'Access,  b2_Shape.e_edge,     b2_Shape.e_polygon);
      addType (b2_chain_circle_Contact  .create'Access,  b2_chain_circle_Contact  .destroy'Access,  b2_Shape.e_chain,    b2_Shape.e_circle);
      addType (b2_chain_polygon_Contact .create'Access,  b2_chain_polygon_Contact .destroy'Access,  b2_Shape.e_chain,    b2_Shape.e_polygon);
   end initializeRegisters;




   --  b2Contact* b2Contact::Create(b2Fixture* fixtureA, int32 indexA, b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator)
   --  {
   --    if (s_initialized == false)
   --    {
   --       InitializeRegisters();
   --       s_initialized = true;
   --    }
   --
   --    b2Shape::Type type1 = fixtureA->GetType();
   --    b2Shape::Type type2 = fixtureB->GetType();
   --
   --    b2Assert(0 <= type1 && type1 < b2Shape::e_typeCount);
   --    b2Assert(0 <= type2 && type2 < b2Shape::e_typeCount);
   --
   --    b2ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
   --    if (createFcn)
   --    {
   --       if (s_registers[type1][type2].primary)
   --       {
   --          return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
   --       }
   --       else
   --       {
   --          return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
   --       }
   --    }
   --    else
   --    {
   --       return nullptr;
   --    }
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      if not s_initialized
      then
         initializeRegisters;
         s_initialized := True;
      end if;

      declare
         type1     : constant b2_Shape.shape_Type := fixtureA.getType;
         type2     : constant b2_Shape.shape_Type := fixtureB.getType;

         type1_Pos : constant Natural := b2_Shape.shape_Type'Pos (type1);
         type2_Pos : constant Natural := b2_Shape.shape_Type'Pos (type2);

         pragma assert (0 <= type1_Pos and type1 < b2_Shape.e_typeCount);
         pragma assert (0 <= type2_Pos and type2 < b2_Shape.e_typeCount);

         createFcn : constant b2ContactCreateFcn := s_registers (type1_Pos, type2_Pos).createFcn;
      begin
         if createFcn /= null
         then
            if s_registers (type1_Pos, type2_Pos).primary
            then
               return createFcn (fixtureA,  indexA,
                                 fixtureB,  indexB);
            else
               return createFcn (fixtureB,  indexB,
                                 fixtureA,  indexA);
            end if;
         else
            return null;
         end if;
      end;
   end create;




   procedure destroy (contact      : access b2Contact;
                      typeA, typeB : in     b2_Shape.shape_Type)
   is
   begin
      raise program_Error;     -- TODO: Unable to find the code for this function, atm.
   end destroy;




   --  void b2Contact::Destroy (b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    b2Assert(s_initialized == true);
   --
   --    b2Fixture* fixtureA = contact->m_fixtureA;
   --    b2Fixture* fixtureB = contact->m_fixtureB;
   --
   --    if (contact->m_manifold.pointCount > 0 &&
   --       fixtureA->IsSensor() == false &&
   --       fixtureB->IsSensor() == false)
   --    {
   --       fixtureA->GetBody()->SetAwake(true);
   --       fixtureB->GetBody()->SetAwake(true);
   --    }
   --
   --    b2Shape::Type typeA = fixtureA->GetType();
   --    b2Shape::Type typeB = fixtureB->GetType();
   --
   --    b2Assert(0 <= typeA && typeA < b2Shape::e_typeCount);
   --    b2Assert(0 <= typeB && typeB < b2Shape::e_typeCount);
   --
   --    b2ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
   --    destroyFcn(contact, allocator);
   --  }
   --

   procedure destroy (contact : access b2Contact)
   is
     pragma assert (s_initialized);

      fixtureA : constant access b2Fixture := contact.m_fixtureA;
      fixtureB : constant access b2Fixture := contact.m_fixtureB;

   begin
      if    contact.m_manifold.pointCount > 0
        and fixtureA.isSensor = False
        and fixtureB.isSensor = False
      then
         fixtureA.getBody.setAwake (True);
         fixtureB.getBody.setAwake (True);
      end if;

      declare
         typeA : constant b2_Shape.shape_Type := fixtureA.getType;
         typeB : constant b2_Shape.shape_Type := fixtureB.getType;

         typeA_Pos : constant Natural := b2_Shape.shape_Type'Pos (typeA);
         typeB_Pos : constant Natural := b2_Shape.shape_Type'Pos (typeB);

         pragma assert (0 <= typeA_Pos and typeA < b2_Shape.e_typeCount);
         pragma assert (0 <= typeB_Pos and typeB < b2_Shape.e_typeCount);

         destroyFcn : constant b2ContactDestroyFcn := s_registers (typeA_Pos, typeB_Pos).destroyFcn;
      begin
         destroyFcn (contact);
      end;
   end destroy;




   --    b2Contact() : m_fixtureA(nullptr), m_fixtureB(nullptr) {}
   --
   -- Setting fixtures to null is Ada default.




   --  b2Contact::b2Contact(b2Fixture* fA, int32 indexA, b2Fixture* fB, int32 indexB)
   --  {
   --    m_flags = e_enabledFlag;
   --
   --    m_fixtureA = fA;
   --    m_fixtureB = fB;
   --
   --    m_indexA = indexA;
   --    m_indexB = indexB;
   --
   --    m_manifold.pointCount = 0;
   --
   --    m_prev = nullptr;
   --    m_next = nullptr;
   --
   --    m_nodeA.contact = nullptr;
   --    m_nodeA.prev = nullptr;
   --    m_nodeA.next = nullptr;
   --    m_nodeA.other = nullptr;
   --
   --    m_nodeB.contact = nullptr;
   --    m_nodeB.prev = nullptr;
   --    m_nodeB.next = nullptr;
   --    m_nodeB.other = nullptr;
   --
   --    m_toiCount = 0;
   --
   --    m_friction = b2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
   --    m_restitution = b2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
   --    m_restitutionThreshold = b2MixRestitutionThreshold(m_fixtureA->m_restitutionThreshold, m_fixtureB->m_restitutionThreshold);
   --
   --    m_tangentSpeed = 0.0f;
   --  }
   --

   procedure define (Self : out b2Contact;   fixtureA : access b2Fixture;   indexA : in Natural;
                                             fixtureB : access b2Fixture;   indexB : in Natural)
   is
   begin
      Self.m_flags    := e_enabledFlag;

      Self.m_fixtureA := fixtureA.all'unchecked_Access;
      Self.m_fixtureB := fixtureB.all'unchecked_Access;

      Self.m_indexA   := indexA;
      Self.m_indexB   := indexB;

      Self.m_manifold.pointCount := 0;

      Self.m_prev := null;
      Self.m_next := null;

      Self.m_nodeA.contact := null;
      Self.m_nodeA.prev    := null;
      Self.m_nodeA.next    := null;
      Self.m_nodeA.other   := null;

      Self.m_nodeB.contact := null;
      Self.m_nodeB.prev    := null;
      Self.m_nodeB.next    := null;
      Self.m_nodeB.other   := null;

      Self.m_toiCount := 0;

      Self.m_friction             := b2MixFriction             (Self.m_fixtureA.getFriction,             Self.m_fixtureB.getFriction);
      Self.m_restitution          := b2MixRestitution          (Self.m_fixtureA.getRestitution,          Self.m_fixtureB.getRestitution);
      Self.m_restitutionThreshold := b2MixRestitutionThreshold (Self.m_fixtureA.getRestitutionThreshold, Self.m_fixtureB.getRestitutionThreshold);

      Self.m_tangentSpeed := 0.0;
   end define;




   --  // Update the contact manifold and touching status.
   --  // Note: do not assume the fixture AABBs are overlapping or are valid.
   --
   --  void b2Contact::Update(b2ContactListener* listener)
   --  {
   --    b2Manifold oldManifold = m_manifold;
   --
   --    // Re-enable this contact.
   --    m_flags |= e_enabledFlag;
   --
   --    bool touching = false;
   --    bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;
   --
   --    bool sensorA = m_fixtureA->IsSensor();
   --    bool sensorB = m_fixtureB->IsSensor();
   --    bool sensor = sensorA || sensorB;
   --
   --    b2Body* bodyA = m_fixtureA->GetBody();
   --    b2Body* bodyB = m_fixtureB->GetBody();
   --    const b2Transform& xfA = bodyA->GetTransform();
   --    const b2Transform& xfB = bodyB->GetTransform();
   --
   --    // Is this contact a sensor?
   --    if (sensor)
   --    {
   --       const b2Shape* shapeA = m_fixtureA->GetShape();
   --       const b2Shape* shapeB = m_fixtureB->GetShape();
   --       touching = b2TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);
   --
   --       // Sensors don't generate manifolds.
   --       m_manifold.pointCount = 0;
   --    }
   --    else
   --    {
   --       Evaluate(&m_manifold, xfA, xfB);
   --       touching = m_manifold.pointCount > 0;
   --
   --       // Match old contact ids to new contact ids and copy the
   --       // stored impulses to warm start the solver.
   --       for (int32 i = 0; i < m_manifold.pointCount; ++i)
   --       {
   --          b2ManifoldPoint* mp2 = m_manifold.points + i;
   --          mp2->normalImpulse = 0.0f;
   --          mp2->tangentImpulse = 0.0f;
   --          b2ContactID id2 = mp2->id;
   --
   --          for (int32 j = 0; j < oldManifold.pointCount; ++j)
   --          {
   --             b2ManifoldPoint* mp1 = oldManifold.points + j;
   --
   --             if (mp1->id.key == id2.key)
   --             {
   --                mp2->normalImpulse = mp1->normalImpulse;
   --                mp2->tangentImpulse = mp1->tangentImpulse;
   --                break;
   --             }
   --          }
   --       }
   --
   --       if (touching != wasTouching)
   --       {
   --          bodyA->SetAwake(true);
   --          bodyB->SetAwake(true);
   --       }
   --    }
   --
   --    if (touching)
   --    {
   --       m_flags |= e_touchingFlag;
   --    }
   --    else
   --    {
   --       m_flags &= ~e_touchingFlag;
   --    }
   --
   --    if (wasTouching == false && touching == true && listener)
   --    {
   --       listener->BeginContact(this);
   --    }
   --
   --    if (wasTouching == true && touching == false && listener)
   --    {
   --       listener->EndContact(this);
   --    }
   --
   --    if (sensor == false && touching && listener)
   --    {
   --       listener->PreSolve(this, &oldManifold);
   --    }
   --  }

   procedure update (Self : in out b2Contact'Class;   listener : access b2ContactListener'Class)
   is
      oldManifold : b2Manifold := Self.m_manifold;
   begin
      -- Re-enable this contact.
      --
      Self.m_flags := Self.m_flags or e_enabledFlag;

      declare
         touching    :          Boolean := False;
         wasTouching : constant Boolean := (Self.m_flags and e_touchingFlag) = e_touchingFlag;

         sensorA : constant Boolean := Self.m_fixtureA.IsSensor;
         sensorB : constant Boolean := Self.m_fixtureB.IsSensor;
         sensor  : constant Boolean := sensorA or sensorB;

         bodyA   : constant access b2Body := Self.m_fixtureA.getBody;
         bodyB   : constant access b2Body := Self.m_fixtureB.getBody;

         xfA     : constant b2Transform := bodyA.getTransform;
         xfB     : constant b2Transform := bodyB.getTransform;

      begin
         -- Is this contact a sensor?
         --
         if sensor
         then
            declare
               shapeA : constant access b2Shape := Self.m_fixtureA.getShape;
               shapeB : constant access b2Shape := Self.m_fixtureB.getShape;
            begin
               touching := b2_Collision.overlap.b2testOverlap (shapeA, Self.m_indexA,
                                                               shapeB, Self.m_indexB,
                                                               xfA,    xfB);
            end;

            -- Sensors don't generate manifolds.
            --
            Self.m_manifold.pointCount := 0;

         else
            Self.evaluate (Self.m_manifold, xfA, xfB);
            touching := Self.m_manifold.pointCount > 0;

            -- Match old contact ids to new contact ids and copy the
            -- stored impulses to warm start the solver.
            --
            for i in 0 .. Self.m_manifold.pointCount - 1
            loop
               declare
                  id2 :                 b2ContactID;
                  mp1 : access          b2ManifoldPoint;
                  mp2 : constant access b2ManifoldPoint := Self.m_manifold.points (i)'Access;
               begin
                  mp2.normalImpulse  := 0.0;
                  mp2.tangentImpulse := 0.0;

                  id2 := mp2.id;

                  for j in 0 .. oldManifold.pointCount - 1
                  loop
                     mp1 := oldManifold.points (j)'Access;

                     if mp1.id.key = id2.key
                     then
                        mp2.normalImpulse  := mp1.normalImpulse;
                        mp2.tangentImpulse := mp1.tangentImpulse;
                        exit;
                     end if;
                  end loop;
               end;
            end loop;

            if touching /= wasTouching
            then
               bodyA.setAwake (True);
               bodyB.setAwake (True);
            end if;
         end if;

         if touching
         then
            Self.m_flags := Self.m_flags or e_touchingFlag;
         else
            Self.m_flags := Self.m_flags and (not e_touchingFlag);
         end if;

         if    wasTouching = False
           and touching
           and listener   /= null
         then
            listener.beginContact (Self);
         end if;

         if    wasTouching
           and touching    = false
           and listener   /= null
         then
            listener.endContact (Self);
         end if;

         if    sensor    = false
           and touching
           and listener /= null
         then
            listener.preSolve (Self, oldManifold);
         end if;
      end;
   end update;




   function m_Prev (Self : in b2Contact) return access b2Contact'Class
   is
   begin
      return Self.m_prev;
   end m_Prev;

   procedure m_Prev_is (Self : in out b2Contact;   Now : in b2Contact_ptr)
   is
   begin
      Self.m_Prev := Now;
   end m_Prev_is;



   function m_Next (Self : in b2Contact) return access b2Contact'Class
   is
   begin
      return Self.m_next;
   end m_Next;

   procedure m_Next_is (Self : in out b2Contact;   Now : in b2Contact_ptr)
   is
   begin
      Self.m_Next := Now;
   end m_Next_is;



   function m_NodeA (Self : access b2Contact) return access b2ContactEdge
   is
   begin
      return Self.m_NodeA'Access;
   end m_NodeA;

   function m_NodeA (Self : in b2Contact) return b2ContactEdge
   is
   begin
      return Self.m_NodeA;
   end m_NodeA;

   procedure m_NodeA_is (Self : in out b2Contact;   Now : in b2ContactEdge)
   is
   begin
      Self.m_NodeA := Now;
   end m_NodeA_is;



   function m_NodeB (Self : access b2Contact) return access b2ContactEdge
   is
   begin
      return Self.m_NodeB'Access;
   end m_NodeB;

   procedure m_NodeB_is (Self : in out b2Contact;   Now : in b2ContactEdge)
   is
   begin
      Self.m_NodeB := Now;
   end m_NodeB_is;



   function m_Flags (Self : in b2Contact) return flag_Set
   is
   begin
      return Self.m_Flags;
   end m_Flags;

   procedure m_Flags_is (Self : in out b2Contact;   Now : in flag_Set)
   is
   begin
      Self.m_flags := Now;
   end m_Flags_is;



   function m_toiCount (Self : in b2Contact) return Natural
   is
   begin
      return Self.m_toiCount;
   end m_toiCount;

   procedure m_toiCount_is (Self : in out b2Contact;  Now : in Natural)
   is
   begin
      Self.m_toiCount := Now;
   end m_toiCount_is;



   function m_toi (Self : in b2Contact) return Real
   is
   begin
      return Self.m_toi;
   end m_toi;

   procedure m_toi_is (Self : in out b2Contact;  Now : in Real)
   is
   begin
      Self.m_toi := Now;
   end m_toi_is;


end b2_Contact;
