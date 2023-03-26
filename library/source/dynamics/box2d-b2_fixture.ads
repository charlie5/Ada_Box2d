with
     --  b2_Body,
     box2d.b2_Collision,
     box2d.b2_broad_Phase,
     box2d.b2_Shape,
     box2d.b2_Math,
     box2d.b2_Settings,

     Interfaces;

limited
with
     box2d.b2_Body;


package box2d.b2_Fixture
is
   use b2_Collision,
       b2_broad_Phase,
       b2_Shape,
       b2_Math,
       b2_Settings,
       Interfaces;



   --  class b2BlockAllocator;
   --  class b2Body;
   --  class b2BroadPhase;
   --  class b2Fixture;
   --





   --  This holds contact filtering data.
   --
   --
   --  struct b2Filter
   --  {
   --    b2Filter()
   --    {
   --       categoryBits = 0x0001;
   --       maskBits = 0xFFFF;
   --       groupIndex = 0;
   --    }
   --
   --    The collision category bits. Normally you would just set one bit.
   --    uint16 categoryBits;
   --
   --    The collision mask bits. This states the categories that this
   --    shape would accept for collision.
   --    uint16 maskBits;
   --
   --    Collision groups allow a certain group of objects to never collide (negative)
   --    or always collide (positive). Zero means no collision group. Non-zero group
   --    filtering always wins against the mask bits.
   --    int16 groupIndex;
   --  };
   --

   type b2Filter is
      record
         -- The collision category bits. Normally you would just set one bit.
         --
         categoryBits : Unsigned_16 := 16#0001#;

         -- The collision mask bits. This states the categories that this
         -- shape would accept for collision.
         --
         maskBits     : Unsigned_16 := 16#FFFF#;

         -- Collision groups allow a certain group of objects to never collide (negative)
         -- or always collide (positive). Zero means no collision group. Non-zero group
         -- filtering always wins against the mask bits.
         --
         groupIndex   : Unsigned_16 := 0;
      end record;







   --  A fixture definition is used to create a fixture. This class defines an
   --  abstract fixture definition. You can reuse fixture definitions safely.
   --
   --  struct b2FixtureDef
   --  {
   --    The constructor sets the default fixture definition values.
   --    b2FixtureDef()
   --    {
   --       shape = nullptr;
   --       friction = 0.2f;
   --       restitution = 0.0f;
   --       restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
   --       density = 0.0f;
   --       isSensor = false;
   --    }
   --
   --    The shape, this must be set. The shape will be cloned, so you
   --    can create the shape on the stack.
   --    const b2Shape* shape;
   --
   --    Use this to store application specific fixture data.
   --    b2FixtureUserData userData;
   --
   --    The friction coefficient, usually in the range [0,1].
   --    float friction;
   --
   --    The restitution (elasticity) usually in the range [0,1].
   --    float restitution;
   --
   --    Restitution velocity threshold, usually in m/s. Collisions above this
   --    speed have restitution applied (will bounce).
   --    float restitutionThreshold;
   --
   --    The density, usually in kg/m^2.
   --    float density;
   --
   --    A sensor shape collects contact information but never generates a collision
   --    response.
   --    bool isSensor;
   --
   --    Contact filtering data.
   --    b2Filter filter;
   --  };
   --

   type b2FixtureDef is tagged
      record
           -- The shape, this must be set. The shape will be cloned, so you
           -- can create the shape on the stack.
           shape : access constant b2Shape'Class;

           -- Use this to store application specific fixture data.
           userData : b2FixtureUserData;

           -- The friction coefficient, usually in the range [0,1].
           friction : Real := 0.2;

           -- The restitution (elasticity) usually in the range [0,1].
           restitution : Real := 0.0;

           -- Restitution velocity threshold, usually in m/s. Collisions above this
           -- speed have restitution applied (will bounce).
           restitutionThreshold : Real := 1.0 * b2_lengthUnitsPerMeter;

           -- The density, usually in kg/m^2.
           density : Real := 0.0;

           -- A sensor shape collects contact information but never generates a collision response.
           isSensor : Boolean := False;

           -- Contact filtering data.
           filter : b2Filter;
      end record;





   ------------------
   -- class b2Fixture
   --

   --  A fixture is used to attach a shape to a body for collision detection. A fixture
   --  inherits its transform from its parent. Fixtures hold additional non-geometric data
   --  such as friction, collision filters, etc.
   --  Fixtures are created via b2Body::CreateFixture.
   --  @warning you cannot reuse fixtures.
   --

   type b2Fixture     is tagged private;
   type b2Fixture_ptr is access all b2Fixture'Class;

   procedure free (the_Fixture : in out b2Fixture_ptr);



   ---------------------
   --  public functions.
   --

   procedure destruct (Self : in out b2Fixture) is null;



   --    Get the type of the child shape. You can use this to down cast to the concrete shape.
   --    @return the shape type.
   --
   --    b2Shape::Type GetType() const;
   --

   function getType (Self : in b2Fixture) return b2_Shape.shape_Type
     with inline;



   --    Get the child shape. You can modify the child shape, however you should not change the
   --    number of vertices because this will crash some collision caching mechanisms.
   --    Manipulating the shape may lead to non-physical behavior.
   --
   --    b2Shape* GetShape();
   --

   function getShape (Self : in b2Fixture) return access b2_Shape.b2Shape'Class
     with inline;


   --    const b2Shape* GetShape() const;
   --

   --  function getShape (Self : in b2Fixture) return b2_Shape.b2Shape'Class
   --    with inline;



   --    Set if this fixture is a sensor.
   --
   --    void SetSensor(bool sensor);
   --

   procedure setSensor (Self : in out b2Fixture;   Sensor : in Boolean);



   --    Is this fixture a sensor (non-solid)?
   --    @return the true if the shape is a sensor.
   --
   --    bool IsSensor() const;
   --

   function isSensor (Self : in b2Fixture) return Boolean
     with inline;



   --    Set the contact filtering data. This will not update contacts until the next time
   --    step when either parent body is active and awake.
   --    This automatically calls Refilter.
   --
   --    void SetFilterData(const b2Filter& filter);
   --

   procedure setFilterData (Self : in out b2Fixture;   filter : in b2Filter);



   --    Get the contact filtering data.
   --
   --    const b2Filter& GetFilterData() const;
   --

   function getFilterData (Self : in b2Fixture) return b2Filter
     with inline;



   --    Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
   --
   --    void Refilter();
   --

   procedure refilter (Self : access b2Fixture);



   --    Get the parent body of this fixture. This is nullptr if the fixture is not attached.
   --    @return the parent body.
   --
   --    b2Body* GetBody();
   --

   function getBody (Self : in out b2Fixture) return access b2_Body.b2Body
     with inline;



   --    const b2Body* GetBody() const;
   --

   function getBody (Self : in b2Fixture) return b2_Body.b2Body
     with inline;



   --    Get the next fixture in the parent body's fixture list.
   --    @return the next shape.
   --
   --    b2Fixture* GetNext();
   --

   function getNext (Self : in out b2Fixture) return access b2Fixture
     with inline;



   --    const b2Fixture* GetNext() const;
   --

   function getNext (Self : in b2Fixture) return b2Fixture
     with inline;



   --    Get the user data that was assigned in the fixture definition. Use this to
   --    store your application specific data.
   --
   --    b2FixtureUserData& GetUserData();
   --

   function getUserData (Self : in b2Fixture) return b2FixtureUserData
     with inline;



   --    Test a point for containment in this fixture.
   --    @param p a point in world coordinates.
   --
   --    bool TestPoint(const b2Vec2& p) const;
   --

   function testPoint (Self : in b2Fixture;   p : in b2Vec2) return Boolean
     with inline;



   --    Cast a ray against this shape.
   --    @param output the ray-cast results.
   --    @param input the ray-cast input parameters.
   --    @param childIndex the child shape index (e.g. edge index)
   --
   --    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const;
   --

   function raycast (Self : in b2Fixture;   output     :    out b2RayCastOutput;
                                            input      : in     b2RayCastInput;
                                            childIndex : in     Natural) return Boolean
     with inline;


   --    Get the mass data for this fixture. The mass data is based on the density and
   --    the shape. The rotational inertia is about the shape's origin. This operation
   --    may be expensive.
   --
   --    void GetMassData(b2MassData* massData) const;
   --

   procedure getMassData (Self : in b2Fixture;   massData : out b2MassData)
     with inline;



   --    Set the density of this fixture. This will _not_ automatically adjust the mass
   --    of the body. You must call b2Body::ResetMassData to update the body's mass.
   --
   --    void SetDensity(float density);
   --

   procedure setDensity (Self : in out b2Fixture;   density : in Real)
     with inline;



   --    Get the density of this fixture.
   --
   --    float GetDensity() const;
   --

   function getDensity (Self : in b2Fixture) return Real
     with inline;



   --    Get the coefficient of friction.
   --
   --    float GetFriction() const;
   --

   function getFriction (Self : in b2Fixture) return Real
     with inline;



   --    Set the coefficient of friction. This will _not_ change the friction of
   --    existing contacts.
   --
   --    void SetFriction(float friction);
   --

   procedure setFriction (Self : in out b2Fixture;   friction : in Real)
     with inline;



   --    Get the coefficient of restitution.
   --
   --    float GetRestitution() const;
   --

   function getRestitution (Self : in b2Fixture) return Real
     with inline;



   --    Set the coefficient of restitution. This will _not_ change the restitution of
   --    existing contacts.
   --
   --    void SetRestitution(float restitution);
   --

   procedure setRestitution (Self : in out b2Fixture;   restitution : in Real)
     with inline;



   --    Get the restitution velocity threshold.
   --
   --    float GetRestitutionThreshold() const;
   --

   function getRestitutionThreshold (Self : in b2Fixture) return Real
     with inline;



   --    Set the restitution threshold. This will _not_ change the restitution threshold of
   --    existing contacts.
   --
   --    void SetRestitutionThreshold(float threshold);
   --

   procedure setRestitutionThreshold (Self : in out b2Fixture;   threshold : in Real)
     with inline;



   --    Get the fixture's AABB. This AABB may be enlarge and/or stale.
   --    If you need a more accurate AABB, compute it using the shape and
   --    the body transform.
   --
   --    const b2AABB& GetAABB(int32 childIndex) const;
   --

   function getAABB (Self : in b2Fixture;   childIndex : in Natural) return b2AABB
     with inline;



   --    Dump this fixture to the log file.
   --
   --    void Dump(int32 bodyIndex);
   --

   procedure dump (Self : in b2Fixture;   bodyIndex : in Natural);



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


   --    b2Fixture();
   --

   function to_b2Fixture return b2Fixture;



   --    // We need separation create/destroy functions from the constructor/destructor because
   --    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
   --

   --    void Create(b2BlockAllocator* allocator, b2Body* body, const b2FixtureDef* def);
   --

   procedure create (Self : in out b2Fixture;   the_body : access b2_Body.b2Body;
                                                def      : in     b2FixtureDef'Class);



   --    void Destroy(b2BlockAllocator* allocator);
   --

   procedure destroy (Self : in out b2Fixture);




   -- These support body activation/deactivation.
   --

   --    void CreateProxies(b2BroadPhase* broadPhase, const b2Transform& xf);
   --

   procedure createProxies (Self : access b2Fixture;   broadPhase : access b2BroadPhase;
                            xf         : in     b2Transform);


   --    void DestroyProxies(b2BroadPhase* broadPhase);
   --

   procedure destroyProxies (Self : in out b2Fixture;   broadPhase : access b2BroadPhase);



   --    void Synchronize(b2BroadPhase* broadPhase, const b2Transform& xf1, const b2Transform& xf2);
   --

   procedure synchronize    (Self : in out b2Fixture;   broadPhase : access b2BroadPhase;
                                                        xf1, xf2   : in     b2Transform);



   function  m_Next    (Self : access b2Fixture)  return access b2Fixture;
   procedure m_Next_is (Self : in out b2Fixture;   Now :        b2Fixture_ptr);

   function  m_Body    (Self : access b2Fixture)         return b2_Body.b2Body_ptr;
   procedure m_Body_is (Self : in out b2Fixture;   Now : access b2_Body.b2Body);

   function  m_Density    (Self : in     b2Fixture)        return Real;
   procedure m_Density_is (Self : in out b2FixtureDef;   Now : in Real);

   procedure m_Shape_is (Self : in out b2FixtureDef;   Now : access b2Shape'Class);


   --  This proxy is used internally to connect fixtures to the broad-phase.
   --
   --  struct b2FixtureProxy
   --  {
   --    b2AABB aabb;
   --    b2Fixture* fixture;
   --    int32 childIndex;
   --    int32 proxyId;
   --  };
   --

   type b2FixtureProxy is
      record
         aabb       :        b2AABB;
         fixture    : access b2Fixture;
         childIndex :        Natural;
         proxyId    :        Integer;
      end record;


   type b2FixtureProxies     is array (Natural range <>) of b2FixtureProxy;
   type b2FixtureProxies_ptr is access all b2FixtureProxies;


   function  m_proxies       (Self : in out b2Fixture) return b2FixtureProxies;


   function  m_proxyCount    (Self : in out b2Fixture)     return Natural;
   procedure m_proxyCount_is (Self : in out b2Fixture;   Now : in Natural);


   --
   -- End of protected b2Fixture functions.
   ----------------------------------------



   private

   --    float m_density;
   --
   --    b2Fixture* m_next;
   --    b2Body* m_body;
   --
   --    b2Shape* m_shape;
   --
   --    float m_friction;
   --    float m_restitution;
   --    float m_restitutionThreshold;
   --
   --    b2FixtureProxy* m_proxies;
   --    int32 m_proxyCount;
   --
   --    b2Filter m_filter;
   --
   --    bool m_isSensor;
   --
   --    b2FixtureUserData m_userData;


   type b2Fixture is tagged
      record
         m_density : Real;

         m_next    : access b2Fixture;
         m_body    : access b2_Body.b2Body;

         m_shape   : access b2Shape'Class;

         m_friction,
         m_restitution,
         m_restitutionThreshold : Real;

         m_proxies    : b2FixtureProxies_ptr;
         m_proxyCount : Natural;

         m_filter   : b2Filter;
         m_isSensor : Boolean;
         m_userData : b2FixtureUserData;
      end record;


end box2d.b2_Fixture;
