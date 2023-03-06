with
     b2_chain_Shape,
     b2_circle_Shape,
     b2_Contact,
     b2_edge_Shape,
     b2_polygon_Shape,
     b2_World,
     b2_Body,
     b2_Common,
     ada.unchecked_Deallocation;


package body b2_Fixture
is
   use b2_Contact,
       b2_World;



   ------------------
   -- class b2Fixture
   ------------------


   ---------------------
   --  public functions.
   --


   --  inline b2Shape::Type b2Fixture::GetType() const
   --  {
   --    return m_shape->GetType();
   --  }
   --

   function getType (Self : in b2Fixture) return b2_Shape.shape_Type
   is
   begin
      return Self.m_shape.getType;
   end getType;




   --    Get the child shape. You can modify the child shape, however you should not change the
   --    number of vertices because this will crash some collision caching mechanisms.
   --    Manipulating the shape may lead to non-physical behavior.
   --
   --  inline b2Shape* b2Fixture::GetShape()
   --  {
   --    return m_shape;
   --  }
   --

   function getShape (Self : in b2Fixture) return access b2_Shape.b2Shape'Class
   is
   begin
      return Self.m_shape;
   end getShape;




   --  inline const b2Shape* b2Fixture::GetShape() const
   --  {
   --    return m_shape;
   --  }
   --

   --  function getShape (Self : in b2Fixture) return b2_Shape.b2Shape'Class
   --  is
   --  begin
   --     return Self.m_shape.all;
   --  end getShape;





   --  void b2Fixture::SetSensor(bool sensor)
   --  {
   --    if (sensor != m_isSensor)
   --    {
   --       m_body->SetAwake(true);
   --       m_isSensor = sensor;
   --    }
   --  }
   --

   procedure setSensor (Self : in out b2Fixture;   Sensor : in Boolean)
   is
   begin
      if sensor /= Self.m_isSensor
      then
         Self.m_body.setAwake (True);
         Self.m_isSensor := sensor;
      end if;
   end setSensor;




   --    Set if this fixture is a sensor.
   --
   --  inline bool b2Fixture::IsSensor() const
   --  {
   --    return m_isSensor;
   --  }
   --

   function isSensor (Self : in b2Fixture) return Boolean
   is
   begin
      return Self.m_isSensor;
   end isSensor;




   --  void b2Fixture::SetFilterData(const b2Filter& filter)
   --  {
   --    m_filter = filter;
   --
   --    Refilter();
   --  }
   --

   procedure setFilterData (Self : in out b2Fixture;   filter : in b2Filter)
   is
   begin
      Self.m_filter := filter;
      Self.refilter;
   end setFilterData;



   --  inline const b2Filter& b2Fixture::GetFilterData() const
   --  {
   --    return m_filter;
   --  }
   --

   function getFilterData (Self : in b2Fixture) return b2Filter
   is
   begin
      return Self.m_filter;
   end getFilterData;




   --  void b2Fixture::Refilter()
   --  {
   --    if (m_body == nullptr)
   --    {
   --       return;
   --    }
   --
   --    // Flag associated contacts for filtering.
   --    b2ContactEdge* edge = m_body->GetContactList();
   --    while (edge)
   --    {
   --       b2Contact* contact = edge->contact;
   --       b2Fixture* fixtureA = contact->GetFixtureA();
   --       b2Fixture* fixtureB = contact->GetFixtureB();
   --       if (fixtureA == this || fixtureB == this)
   --       {
   --          contact->FlagForFiltering();
   --       }
   --
   --       edge = edge->next;
   --    }
   --
   --    b2World* world = m_body->GetWorld();
   --
   --    if (world == nullptr)
   --    {
   --       return;
   --    }
   --
   --    // Touch each proxy so that new pairs may be created
   --    b2BroadPhase* broadPhase = &world->m_contactManager.m_broadPhase;
   --    for (int32 i = 0; i < m_proxyCount; ++i)
   --    {
   --       broadPhase->TouchProxy(m_proxies[i].proxyId);
   --    }
   --  }
   --

   procedure refilter (Self : access b2Fixture)
   is
   begin
      if Self.m_body = null
      then
         return;
      end if;

      -- Flag associated contacts for filtering.
      --
      declare
         edge : access b2ContactEdge := Self.m_body.getContactList;
      begin
        while edge /= null
         loop
            declare
               contact  : constant access b2Contact := edge.contact;
               fixtureA : constant access b2Fixture := contact.getFixtureA;
               fixtureB : constant access b2Fixture := contact.getFixtureB;
            begin
               if fixtureA = Self or fixtureB = Self
               then
                  contact.flagForFiltering;
               end if;
            end;

            edge := edge.next;
         end loop;
      end;


      declare
         world      : constant access b2World     := Self.m_body.getWorld;
         broadPhase :          access b2BroadPhase;
      begin

         if world = null
         then
            return;
         end if;

         -- Touch each proxy so that new pairs may be created.
         --
         broadPhase := world.m_contactManager.m_broadPhase'Access;

         for i in 0 .. Self.m_proxyCount - 1
         loop
            broadPhase.touchProxy (Self.m_proxies (i).proxyId);
         end loop;
      end;
   end refilter;




   --  inline b2Body* b2Fixture::GetBody()
   --  {
   --    return m_body;
   --  }
   --

   function getBody (Self : in out b2Fixture) return access b2_Body.b2Body
   is
   begin
      return Self.m_body;
   end getBody;




   --  inline const b2Body* b2Fixture::GetBody() const
   --  {
   --    return m_body;
   --  }
   --

   function getBody (Self : in b2Fixture) return b2_Body.b2Body
   is
   begin
      return Self.m_body.all;
   end getBody;




   --  inline b2Fixture* b2Fixture::GetNext()
   --  {
   --    return m_next;
   --  }
   --

   function getNext (Self : in out b2Fixture) return access b2Fixture
   is
   begin
      return Self.m_next;
   end getNext;




   --  inline const b2Fixture* b2Fixture::GetNext() const
   --  {
   --    return m_next;
   --  }
   --

   function getNext (Self : in b2Fixture) return b2Fixture
   is
   begin
      return Self.m_next.all;
   end getNext;




   --  inline b2FixtureUserData& b2Fixture::GetUserData()
   --  {
   --    return m_userData;
   --  }
   --

   function getUserData (Self : in b2Fixture) return b2FixtureUserData
   is
   begin
      return Self.m_userData;
   end getUserData;




   --  inline bool b2Fixture::TestPoint(const b2Vec2& p) const
   --  {
   --    return m_shape->TestPoint(m_body->GetTransform(), p);
   --  }
   --

   function testPoint (Self : in b2Fixture;   p : in b2Vec2) return Boolean
   is
   begin
      return Self.m_shape.testPoint (Self.m_body.getTransform, p);
   end testPoint;




   --    Cast a ray against this shape.
   --    @param output the ray-cast results.
   --    @param input the ray-cast input parameters.
   --    @param childIndex the child shape index (e.g. edge index)
   --
   --  inline bool b2Fixture::RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const
   --  {
   --    return m_shape->RayCast(output, input, m_body->GetTransform(), childIndex);
   --  }
   --

   function raycast (Self : in b2Fixture;   output     :    out b2RayCastOutput;
                                            input      : in     b2RayCastInput;
                                            childIndex : in     Natural) return Boolean
   is
   begin
      return Self.m_shape.raycast (output,
                                   input,
                                   Self.m_body.getTransform,
                                   childIndex);
   end raycast;



   --    Get the mass data for this fixture. The mass data is based on the density and
   --    the shape. The rotational inertia is about the shape's origin. This operation
   --    may be expensive.
   --
   --  inline void b2Fixture::GetMassData(b2MassData* massData) const
   --  {
   --    m_shape->ComputeMass(massData, m_density);
   --  }
   --

   procedure getMassData (Self : in b2Fixture;   massData : out b2MassData)
   is
   begin
      Self.m_shape.computeMass (massData, Self.m_density);
   end getMassData;




   --    Set the density of this fixture. This will _not_ automatically adjust the mass
   --    of the body. You must call b2Body::ResetMassData to update the body's mass.
   --
   --  inline void b2Fixture::SetDensity(float density)
   --  {
   --    b2Assert(b2IsValid(density) && density >= 0.0f);
   --    m_density = density;
   --  }
   --

   procedure setDensity (Self : in out b2Fixture;   density : in Real)
   is
      pragma assert (b2IsValid (density) and density >= 0.0);
   begin
      Self.m_density := density;
   end setDensity;




   --    Get the density of this fixture.
   --
   --  inline float b2Fixture::GetDensity() const
   --  {
   --    return m_density;
   --  }
   --

   function getDensity (Self : in b2Fixture) return Real
   is
   begin
      return Self.m_density;
   end getDensity;




   --    Get the coefficient of friction.
   --
   --  inline float b2Fixture::GetFriction() const
   --  {
   --    return m_friction;
   --  }
   --

   function getFriction (Self : in b2Fixture) return Real
   is
   begin
      return Self.m_friction;
   end getFriction;




   --    Set the coefficient of friction. This will _not_ change the friction of
   --    existing contacts.
   --

   --  inline void b2Fixture::SetFriction(float friction)
   --  {
   --    m_friction = friction;
   --  }
   --

   procedure setFriction (Self : in out b2Fixture;   friction : in Real)
   is
   begin
      Self.m_friction := friction;
   end setFriction;




   --    Get the coefficient of restitution.
   --
   --  inline float b2Fixture::GetRestitution() const
   --  {
   --    return m_restitution;
   --  }
   --

   function getRestitution (Self : in b2Fixture) return Real
   is
   begin
      return Self.m_restitution;
   end getRestitution;




   --    Set the coefficient of restitution. This will _not_ change the restitution of
   --    existing contacts.
   --
   --  inline void b2Fixture::SetRestitution(float restitution)
   --  {
   --    m_restitution = restitution;
   --  }
   --

   procedure setRestitution (Self : in out b2Fixture;   restitution : in Real)
   is
   begin
      Self.m_restitution := restitution;
   end setRestitution;




   --    Get the restitution velocity threshold.
   --
   --  inline float b2Fixture::GetRestitutionThreshold() const
   --  {
   --    return m_restitutionThreshold;
   --  }
   --

   function getRestitutionThreshold (Self : in b2Fixture) return Real
   is
   begin
      return Self.m_restitutionThreshold;
   end getRestitutionThreshold;




   --    Set the restitution threshold. This will _not_ change the restitution threshold of
   --    existing contacts.
   --
   --  inline void b2Fixture::SetRestitutionThreshold(float threshold)
   --  {
   --    m_restitutionThreshold = threshold;
   --  }
   --

   procedure setRestitutionThreshold (Self : in out b2Fixture;   threshold : in Real)
   is
   begin
      Self.m_restitutionThreshold := threshold;
   end setRestitutionThreshold;




   --    Get the fixture's AABB. This AABB may be enlarge and/or stale.
   --    If you need a more accurate AABB, compute it using the shape and
   --    the body transform.
   --
   --  inline const b2AABB& b2Fixture::GetAABB(int32 childIndex) const
   --  {
   --    b2Assert(0 <= childIndex && childIndex < m_proxyCount);
   --    return m_proxies[childIndex].aabb;
   --  }
   --

   function getAABB (Self : in b2Fixture;   childIndex : in Natural) return b2AABB
   is
      pragma assert (0 <= childIndex and childIndex < Self.m_proxyCount);
   begin
      return Self.m_proxies (childIndex).aabb;
   end getAABB;



   --  End of public functions.
   ----------------------------




   ---------------------------------
   -- Protected b2Fixture functions.
   --

   --  protected:
   --
   --    friend class b2Body;
   --    friend class b2World;
   --    friend class b2Contact;
   --    friend class b2ContactManager;
   --



   --  b2Fixture::b2Fixture()
   --  {
   --    m_body = nullptr;
   --    m_next = nullptr;
   --    m_proxies = nullptr;
   --    m_proxyCount = 0;
   --    m_shape = nullptr;
   --    m_density = 0.0f;
   --  }
   --

   function to_b2Fixture return b2Fixture
   is
      Self : b2Fixture;
   begin
      Self.m_proxyCount := 0;
      Self.m_density    := 0.0;

      return Self;
   end to_b2Fixture;




   --    // We need separation create/destroy functions from the constructor/destructor because
   --    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
   --
   --  void b2Fixture::Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def)
   --  {
   --    m_userData = def->userData;
   --    m_friction = def->friction;
   --    m_restitution = def->restitution;
   --    m_restitutionThreshold = def->restitutionThreshold;
   --
   --    m_body = body;
   --    m_next = nullptr;
   --
   --    m_filter = def->filter;
   --
   --    m_isSensor = def->isSensor;
   --
   --    m_shape = def->shape->Clone(allocator);
   --
   --    // Reserve proxy space
   --    int32 childCount = m_shape->GetChildCount();
   --    m_proxies = (b2FixtureProxy*)allocator->Allocate(childCount * sizeof(b2FixtureProxy));
   --    for (int32 i = 0; i < childCount; ++i)
   --    {
   --       m_proxies[i].fixture = nullptr;
   --       m_proxies[i].proxyId = b2BroadPhase::e_nullProxy;
   --    }
   --    m_proxyCount = 0;
   --
   --    m_density = def->density;
   --  }
   --

   procedure create (Self : in out b2Fixture;   the_body : access b2_Body.b2Body;
                                                def      : in     b2FixtureDef'Class)
   is
   begin
      Self.m_userData             := def.userData;
      Self.m_friction             := def.friction;
      Self.m_restitution          := def.restitution;
      Self.m_restitutionThreshold := def.restitutionThreshold;

      Self.m_body := the_body;
      Self.m_next := null;

      Self.m_filter   := def.filter;
      Self.m_isSensor := def.isSensor;
      Self.m_shape    := def.shape.clone;

      -- Reserve proxy space.
      --
      declare
         childCount : constant Natural := Self.m_shape.getChildCount;
      begin
         Self.m_proxies := new b2FixtureProxies (0 .. childCount - 1);

         for i in 0 .. childCount - 1
         loop
            Self.m_proxies (i).fixture := null;
            Self.m_proxies (i).proxyId := b2_Broad_Phase.e_nullProxy;
         end loop;
      end;

      Self.m_proxyCount := 0;
      Self.m_density    := def.density;
   end create;




   --  void b2Fixture::Destroy(b2BlockAllocator* allocator)
   --  {
   --    // The proxies must be destroyed before calling this.
   --    b2Assert(m_proxyCount == 0);
   --
   --    // Free the proxy array.
   --    int32 childCount = m_shape->GetChildCount();
   --    allocator->Free(m_proxies, childCount * sizeof(b2FixtureProxy));
   --    m_proxies = nullptr;
   --
   --    // Free the child shape.
   --    switch (m_shape->m_type)
   --    {
   --    case b2Shape::e_circle:
   --       {
   --          b2CircleShape* s = (b2CircleShape*)m_shape;
   --          s->~b2CircleShape();
   --          allocator->Free(s, sizeof(b2CircleShape));
   --       }
   --       break;
   --
   --    case b2Shape::e_edge:
   --       {
   --          b2EdgeShape* s = (b2EdgeShape*)m_shape;
   --          s->~b2EdgeShape();
   --          allocator->Free(s, sizeof(b2EdgeShape));
   --       }
   --       break;
   --
   --    case b2Shape::e_polygon:
   --       {
   --          b2PolygonShape* s = (b2PolygonShape*)m_shape;
   --          s->~b2PolygonShape();
   --          allocator->Free(s, sizeof(b2PolygonShape));
   --       }
   --       break;
   --
   --    case b2Shape::e_chain:
   --       {
   --          b2ChainShape* s = (b2ChainShape*)m_shape;
   --          s->~b2ChainShape();
   --          allocator->Free(s, sizeof(b2ChainShape));
   --       }
   --       break;
   --
   --    default:
   --       b2Assert(false);
   --       break;
   --    }
   --
   --    m_shape = nullptr;
   --  }
   --

   procedure destroy (Self : in out b2Fixture)
   is
      procedure free is new ada.unchecked_Deallocation (b2FixtureProxies, b2FixtureProxies_ptr);
      procedure free is new ada.unchecked_Deallocation (b2Shape'Class,    b2Shape_ptr);


      -- The proxies must be destroyed before calling this.
      --
      pragma assert (Self.m_proxyCount = 0);

      -- Free the proxy array.
      --
      childCount : Natural := Self.m_shape.getChildCount;

   begin
      free (Self.m_proxies);

      -- Free the child shape.
      --
        case Self.m_shape.m_type     -- TODO: Destruction can probly be done in one line with a virtual shape destructor.
        is
        when b2_Shape.e_circle =>
            declare
               use  b2_circle_Shape;
               type b2CircleShape_ptr is access all b2circleShape;

               s : constant b2CircleShape_ptr := b2CircleShape_ptr (Self.m_shape);
            begin
               s.destruct;
            end;

        when b2_Shape.e_edge =>
            declare
               use  b2_edge_Shape;
               type b2EdgeShape_ptr is access all b2edgeShape;

               s : constant b2EdgeShape_ptr := b2EdgeShape_ptr (Self.m_shape);
            begin
               s.destruct;
            end;

        when b2_Shape.e_polygon =>
           declare
               use  b2_polygon_Shape;
               type b2polygonShape_ptr is access all b2polygonShape;

               s : constant b2PolygonShape_ptr := b2PolygonShape_ptr (Self.m_shape);
            begin
               s.destruct;
            end;

        when b2_Shape.e_chain =>
           declare
               use  b2_chain_Shape;
               type b2chainShape_ptr is access all b2chainShape;

               s : constant b2chainShape_ptr := b2chainShape_ptr (Self.m_shape);
            begin
               s.destruct;
            end;

         when others =>
            pragma assert (False);
      end case;

      free (Self.m_shape);
   end destroy;




   -- These support body activation/deactivation.
   --

   --  void b2Fixture::CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf)
   --  {
   --    b2Assert(m_proxyCount == 0);
   --
   --    // Create proxies in the broad-phase.
   --    m_proxyCount = m_shape->GetChildCount();
   --
   --    for (int32 i = 0; i < m_proxyCount; ++i)
   --    {
   --       b2FixtureProxy* proxy = m_proxies + i;
   --       m_shape->ComputeAABB(&proxy->aabb, xf, i);
   --       proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy);
   --       proxy->fixture = this;
   --       proxy->childIndex = i;
   --    }
   --  }
   --

   procedure createProxies  (Self : access b2Fixture;   broadPhase : access b2BroadPhase;
                                                        xf         : in     b2Transform)
   is
      pragma assert (Self.m_proxyCount = 0);
   begin
      -- Create proxies in the broad-phase.
      --
      Self.m_proxyCount := Self.m_shape.getChildCount;

     for i in 0 .. Self.m_proxyCount - 1
      loop
         declare
            proxy : b2FixtureProxy renames Self.m_proxies (i);
         begin
            Self.m_shape.computeAABB (proxy.aabb, xf, i);

            proxy.proxyId    := broadPhase.createProxy (proxy.aabb, proxy'Address);
            proxy.fixture    := Self.all'Access;
            proxy.childIndex := i;
         end;
      end loop;
   end createProxies;




   --  void b2Fixture::DestroyProxies(b2BroadPhase* broadPhase)
   --  {
   --    // Destroy proxies in the broad-phase.
   --    for (int32 i = 0; i < m_proxyCount; ++i)
   --    {
   --       b2FixtureProxy* proxy = m_proxies + i;
   --       broadPhase->DestroyProxy(proxy->proxyId);
   --       proxy->proxyId = b2BroadPhase::e_nullProxy;
   --    }
   --
   --    m_proxyCount = 0;
   --  }
   --

   procedure destroyProxies (Self : in out b2Fixture;   broadPhase : access b2BroadPhase)
   is
   begin
      -- Destroy proxies in the broad-phase.
      --
      for i in 0 .. Self.m_proxyCount - 1
      loop
         declare
            proxy : b2FixtureProxy renames Self.m_proxies (i);
         begin
            broadPhase.destroyProxy (proxy.proxyId);
            proxy.proxyId := b2_Broad_Phase.e_nullProxy;
         end;
      end loop;

      Self.m_proxyCount := 0;
   end destroyProxies;




   --  void b2Fixture::Synchronize(b2BroadPhase* broadPhase, const b2Transform& transform1, const b2Transform& transform2)
   --  {
   --    if (m_proxyCount == 0)
   --    {
   --       return;
   --    }
   --
   --    for (int32 i = 0; i < m_proxyCount; ++i)
   --    {
   --       b2FixtureProxy* proxy = m_proxies + i;
   --
   --       // Compute an AABB that covers the swept shape (may miss some rotation effect).
   --       b2AABB aabb1, aabb2;
   --       m_shape->ComputeAABB(&aabb1, transform1, proxy->childIndex);
   --       m_shape->ComputeAABB(&aabb2, transform2, proxy->childIndex);
   --
   --       proxy->aabb.Combine(aabb1, aabb2);
   --
   --       b2Vec2 displacement = aabb2.GetCenter() - aabb1.GetCenter();
   --
   --       broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
   --    }
   --  }
   --

   procedure synchronize    (Self : in out b2Fixture;   broadPhase : access b2BroadPhase;
                                                        xf1, xf2   : in     b2Transform)
   is
   begin
      if Self.m_proxyCount = 0
      then
         return;
      end if;

      for i in 0 .. Self.m_proxyCount - 1
      loop
         declare
            proxy : b2FixtureProxy renames Self.m_proxies (i);

            -- Compute an AABB that covers the swept shape (may miss some rotation effect).
            --
            aabb1, aabb2 : b2AABB;
            displacement : b2Vec2;
         begin
            Self.m_shape.computeAABB (aabb1, xf1, proxy.childIndex);
            Self.m_shape.computeAABB (aabb2, xf2, proxy.childIndex);

            combine (proxy.aabb,  aabb1, aabb2);

            displacement := getCenter (aabb2) - getCenter (aabb1);

            broadPhase.moveProxy (proxy.proxyId, proxy.aabb, displacement);
         end;
      end loop;
   end synchronize;




   function m_Next (Self : access b2Fixture)  return access b2Fixture
   is
   begin
      return Self.m_Next;
   end m_Next;




   procedure m_Next_is (Self : in out b2Fixture;   Now : access b2Fixture)
   is
   begin
      Self.m_Next := Now;
   end m_Next_is;




   function m_Body (Self : access b2Fixture) return b2_Body.b2Body_ptr
   is
   begin
      return Self.m_body;
   end m_Body;




   procedure m_Body_is (Self : in out b2Fixture;   Now : access b2_Body.b2Body)
   is
   begin
      Self.m_body := Now;
   end m_Body_is;




   function m_Density (Self : in b2Fixture) return Real
   is
   begin
      return Self.m_density;
   end m_Density;




   procedure m_Density_is (Self : in out b2FixtureDef;   Now : in Real)
   is
   begin
      Self.density := Now;
   end m_Density_is;




   procedure m_Shape_is (Self : in out b2FixtureDef;   Now : access b2Shape'Class)
   is
   begin
      Self.shape := Now;
   end m_Shape_is;


   --
   -- End of protected b2Fixture functions.
   ----------------------------------------



--  void b2Fixture::Dump(int32 bodyIndex)
--  {
--    b2Dump("    b2FixtureDef fd;\n");
--    b2Dump("    fd.friction = %.9g;\n", m_friction);
--    b2Dump("    fd.restitution = %.9g;\n", m_restitution);
--    b2Dump("    fd.restitutionThreshold = %.9g;\n", m_restitutionThreshold);
--    b2Dump("    fd.density = %.9g;\n", m_density);
--    b2Dump("    fd.isSensor = bool(%d);\n", m_isSensor);
--    b2Dump("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
--    b2Dump("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
--    b2Dump("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);
--
--    switch (m_shape->m_type)
--    {
--    case b2Shape::e_circle:
--       {
--          b2CircleShape* s = (b2CircleShape*)m_shape;
--          b2Dump("    b2CircleShape shape;\n");
--          b2Dump("    shape.m_radius = %.9g;\n", s->m_radius);
--          b2Dump("    shape.m_p.Set(%.9g, %.9g);\n", s->m_p.x, s->m_p.y);
--       }
--       break;
--
--    case b2Shape::e_edge:
--       {
--          b2EdgeShape* s = (b2EdgeShape*)m_shape;
--          b2Dump("    b2EdgeShape shape;\n");
--          b2Dump("    shape.m_radius = %.9g;\n", s->m_radius);
--          b2Dump("    shape.m_vertex0.Set(%.9g, %.9g);\n", s->m_vertex0.x, s->m_vertex0.y);
--          b2Dump("    shape.m_vertex1.Set(%.9g, %.9g);\n", s->m_vertex1.x, s->m_vertex1.y);
--          b2Dump("    shape.m_vertex2.Set(%.9g, %.9g);\n", s->m_vertex2.x, s->m_vertex2.y);
--          b2Dump("    shape.m_vertex3.Set(%.9g, %.9g);\n", s->m_vertex3.x, s->m_vertex3.y);
--          b2Dump("    shape.m_oneSided = bool(%d);\n", s->m_oneSided);
--       }
--       break;
--
--    case b2Shape::e_polygon:
--       {
--          b2PolygonShape* s = (b2PolygonShape*)m_shape;
--          b2Dump("    b2PolygonShape shape;\n");
--          b2Dump("    b2Vec2 vs[%d];\n", b2_maxPolygonVertices);
--          for (int32 i = 0; i < s->m_count; ++i)
--          {
--             b2Dump("    vs[%d].Set(%.9g, %.9g);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
--          }
--          b2Dump("    shape.Set(vs, %d);\n", s->m_count);
--       }
--       break;
--
--    case b2Shape::e_chain:
--       {
--          b2ChainShape* s = (b2ChainShape*)m_shape;
--          b2Dump("    b2ChainShape shape;\n");
--          b2Dump("    b2Vec2 vs[%d];\n", s->m_count);
--          for (int32 i = 0; i < s->m_count; ++i)
--          {
--             b2Dump("    vs[%d].Set(%.9g, %.9g);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
--          }
--          b2Dump("    shape.CreateChain(vs, %d);\n", s->m_count);
--          b2Dump("    shape.m_prevVertex.Set(%.9g, %.9g);\n", s->m_prevVertex.x, s->m_prevVertex.y);
--          b2Dump("    shape.m_nextVertex.Set(%.9g, %.9g);\n", s->m_nextVertex.x, s->m_nextVertex.y);
--       }
--       break;
--
--    default:
--       return;
--    }
--
--    b2Dump("\n");
--    b2Dump("    fd.shape = &shape;\n");
--    b2Dump("\n");
--    b2Dump("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
--  }
--

   procedure dump (Self : in b2Fixture;   bodyIndex : in Natural)
   is
      use b2_Common;
   begin
      b2Dump ("    b2FixtureDef fd;");
      b2Dump ("    fd.friction             = " & Self.m_friction            'Image);
      b2Dump ("    fd.restitution          = " & Self.m_restitution         'Image);
      b2Dump ("    fd.restitutionThreshold = " & Self.m_restitutionThreshold'Image);
      b2Dump ("    fd.density              = " & Self.m_density             'Image);
      b2Dump ("    fd.isSensor             = " & Self.m_isSensor            'Image);
      b2Dump ("    fd.filter.categoryBits  = " & Self.m_filter.categoryBits 'Image);
      b2Dump ("    fd.filter.maskBits      = " & Self.m_filter.maskBits     'Image);
      b2Dump ("    fd.filter.groupIndex    = " & Self.m_filter.groupIndex   'Image);

      case Self.m_shape.m_type
      is
         when b2_Shape.e_circle =>
            declare
               use  b2_circle_Shape;
               type b2CircleShape_ptr is access all b2CircleShape;

               s : constant access b2CircleShape := b2CircleShape_ptr (Self.m_shape);
            begin
               b2Dump ("    b2CircleShape shape;");
               b2Dump ("    shape.m_radius = " & s.m_radius'Image);
               b2Dump ("    shape.m_p      = (" & s.m_p.x'Image & ", " & s.m_p.y'Image & ")");
            end;


         when b2_Shape.e_edge =>
            declare
               use  b2_edge_Shape;
               type b2EdgeShape_ptr is access all b2EdgeShape;

               s : constant access b2EdgeShape := b2EdgeShape_ptr (Self.m_shape);
            begin
               b2Dump ("    b2EdgeShape shape;");
               b2Dump ("    shape.m_radius   = "  & s.m_radius'Image);
               b2Dump ("    shape.m_vertex0  = (" & s.m_vertex0.x'Image & ", " & s.m_vertex0.y'Image & ")");
               b2Dump ("    shape.m_vertex1  = (" & s.m_vertex1.x'Image & ", " & s.m_vertex1.y'Image & ")");
               b2Dump ("    shape.m_vertex2  = (" & s.m_vertex2.x'Image & ", " & s.m_vertex2.y'Image & ")");
               b2Dump ("    shape.m_vertex3  = (" & s.m_vertex3.x'Image & ", " & s.m_vertex3.y'Image & ")");
               b2Dump ("    shape.m_oneSided = "  & s.m_oneSided'Image);
            end;


         when b2_Shape.e_polygon =>
            declare
               use  b2_polygon_Shape;
               type b2PolygonShape_ptr is access all b2PolygonShape;

               s : constant access b2PolygonShape := b2PolygonShape_ptr (Self.m_shape);
            begin
               b2Dump ("    b2PolygonShape shape;\n");
               b2Dump ("    b2Vec2 vs [" & b2_maxPolygonVertices'Image & "]");

               for i in 0 .. s.m_count - 1
               loop
                  b2Dump ("    vs [" & i'Image & "] = (" & s.m_vertices (i).x'Image & ", " & s.m_vertices (i).y'Image & ")");
               end loop;

               b2Dump ("    shape.m_count = " & s.m_count'Image);
            end;


         when b2_Shape.e_chain =>
            declare
               use  b2_chain_Shape;
               type b2ChainShape_ptr is access all b2ChainShape;

               s : constant access b2ChainShape := b2ChainShape_ptr (Self.m_shape);
            begin
               b2Dump ("    b2ChainShape shape;");
               b2Dump ("    b2Vec2 vs [" & s.m_count'Image & "]");

               for i in 0 .. s.m_count - 1
               loop
                  b2Dump ("    vs [" & i'Image & "] = (" & s.m_vertices (i).x'Image & ", " & s.m_vertices (i).y'Image & ")");
               end loop;

               b2Dump ("    shape.m_count      = "  & s.m_Count'Image);
               b2Dump ("    shape.m_prevVertex = (" & s.m_prevVertex.x'Image & ", " & s.m_prevVertex.y'Image & ")");
               b2Dump ("    shape.m_nextVertex = (" & s.m_nextVertex.x'Image & ", " & s.m_nextVertex.y'Image & ")");
            end;


         when others =>
            return;

      end case;


      b2Dump ("");
      b2Dump ("    fd.shape = &shape;");
      b2Dump ("");
      b2Dump ("    bodies [" & bodyIndex'Image & "]->CreateFixture(&fd);");
   end dump;


end b2_Fixture;
