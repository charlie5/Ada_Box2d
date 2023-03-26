with
     box2d.b2_polygon_Shape,
     box2d.b2_circle_Shape,
     box2d.b2_collide_Circle,
     box2d.b2_Shape,

     ada.unchecked_Deallocation;


package body box2d.b2_polygon_circle_Contact
is

   --  b2Contact* b2PolygonAndCircleContact::Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2PolygonAndCircleContact));
   --    return new (mem) b2PolygonAndCircleContact(fixtureA, fixtureB);
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2PolygonAndCircleContact' (to_b2PolygonAndCircleContact (fixtureA'Access,
                                                                           fixtureB'Access));
   end create;



   --  void b2PolygonAndCircleContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2PolygonAndCircleContact*)contact)->~b2PolygonAndCircleContact();
   --    allocator->Free(contact, sizeof(b2PolygonAndCircleContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2PolygonAndCircleContact_ptr is access all b2PolygonAndCircleContact;

      procedure free is new ada.unchecked_Deallocation (b2PolygonAndCircleContact, b2PolygonAndCircleContact_ptr);

      Self : b2PolygonAndCircleContact_ptr := b2PolygonAndCircleContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2PolygonAndCircleContact::b2PolygonAndCircleContact(b2Fixture* fixtureA, b2Fixture* fixtureB)
   --  : b2Contact(fixtureA, 0, fixtureB, 0)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_polygon);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_circle);
   --  }
   --

   function to_b2PolygonAndCircleContact (fixtureA : access b2Fixture;
                                          fixtureB : access b2Fixture) return b2PolygonAndCircleContact
   is
      use b2_Shape;

      Self : aliased b2PolygonAndCircleContact;
   begin
      define (Self, fixtureA, 0,
                    fixtureB, 0);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_polygon);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_circle);

      return Self;
   end to_b2PolygonAndCircleContact;



   --  void b2PolygonAndCircleContact::Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2CollidePolygonAndCircle (manifold,
   --                               (b2PolygonShape*)m_fixtureA->GetShape(), xfA,
   --                               (b2CircleShape*)m_fixtureB->GetShape(), xfB);
   --  }

   overriding
   procedure evaluate (Self : in out b2PolygonAndCircleContact;   manifold : in out b2Manifold;
                                                                  xfA, xfB : in     b2Transform)
   is
      use b2_polygon_Shape,
          b2_circle_Shape,
          b2_collide_Circle;

      type b2polygonShape_ptr is access all b2polygonShape;
      type b2circleShape_ptr  is access all b2circleShape;

      polygon : constant access b2polygonShape := b2polygonShape_ptr (Self.getFixtureA.getShape);
      circle  : constant access b2circleShape  :=  b2circleShape_ptr (Self.getFixtureB.getShape);
   begin
      b2CollidePolygonAndCircle (manifold, polygon.all, xfA,
                                           circle .all, xfB);
   end evaluate;


end box2d.b2_polygon_circle_Contact;
