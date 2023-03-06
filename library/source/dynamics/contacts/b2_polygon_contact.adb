with
     b2_polygon_Shape,
     b2_collide_Polygon,
     b2_Shape,
     ada.unchecked_Deallocation;


package body b2_polygon_Contact
is

   --  b2Contact* b2PolygonContact::Create (b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2PolygonContact));
   --    return new (mem) b2PolygonContact(fixtureA, fixtureB);
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2PolygonContact' (to_b2PolygonContact (fixtureA'Access,
                                                         fixtureB'Access));
   end create;



   --  void b2PolygonContact::Destroy (b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2PolygonContact*)contact)->~b2PolygonContact();
   --    allocator->Free(contact, sizeof(b2PolygonContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2PolygonContact_ptr is access all b2PolygonContact;

      procedure free is new ada.unchecked_Deallocation (b2PolygonContact, b2PolygonContact_ptr);

      Self : b2PolygonContact_ptr := b2PolygonContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2PolygonContact::b2PolygonContact (b2Fixture* fixtureA, b2Fixture* fixtureB)
   --    : b2Contact(fixtureA, 0, fixtureB, 0)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_polygon);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_polygon);
   --  }
   --

   function to_b2PolygonContact (fixtureA : access b2Fixture;
                                 fixtureB : access b2Fixture) return b2PolygonContact
   is
      use b2_Shape;

      Self : aliased b2PolygonContact;
   begin
      define (Self, fixtureA, 0,
                    fixtureB, 0);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_polygon);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_polygon);

      return Self;
   end to_b2PolygonContact;



   --  void b2PolygonContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2CollidePolygons (manifold,
   --                       (b2PolygonShape*)m_fixtureA->GetShape(), xfA,
   --                       (b2PolygonShape*)m_fixtureB->GetShape(), xfB);
   --  }
   --

   overriding
   procedure evaluate (Self : in out b2PolygonContact;   manifold : in out b2Manifold;
                                                         xfA, xfB : in     b2Transform)
   is
      use b2_polygon_Shape,
          b2_collide_Polygon;

      type b2polygonShape_ptr is access all b2polygonShape;

      polygonA : constant access b2polygonShape := b2polygonShape_ptr (Self.getFixtureA.getShape);
      polygonB : constant access b2polygonShape := b2polygonShape_ptr (Self.getFixtureB.getShape);
   begin
      b2collidePolygons (manifold, polygonA.all, xfA,
                                   polygonB.all, xfB);
   end evaluate;


end b2_polygon_Contact;
