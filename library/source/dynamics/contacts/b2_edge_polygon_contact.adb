with
     b2_polygon_Shape,
     b2_edge_Shape,
     b2_collide_Edge,
     b2_Shape,
     ada.unchecked_Deallocation;


package body b2_edge_polygon_Contact
is

   --  b2Contact* b2EdgeAndPolygonContact::Create(b2Fixture* fixtureA, int32, b2Fixture* fixtureB, int32, b2BlockAllocator* allocator)
   --  {
   --    void* mem = allocator->Allocate(sizeof(b2EdgeAndPolygonContact));
   --    return new (mem) b2EdgeAndPolygonContact(fixtureA, fixtureB);
   --  }
   --

   function create (fixtureA : in out b2Fixture;   indexA : in Natural;
                    fixtureB : in out b2Fixture;   indexB : in Natural) return access b2Contact'Class
   is
   begin
      return new b2EdgeAndPolygonContact' (to_b2EdgeAndPolygonContact (fixtureA'Access,
                                                                       fixtureB'Access));
   end create;




   --  void b2EdgeAndPolygonContact::Destroy(b2Contact* contact, b2BlockAllocator* allocator)
   --  {
   --    ((b2EdgeAndPolygonContact*)contact)->~b2EdgeAndPolygonContact();
   --    allocator->Free(contact, sizeof(b2EdgeAndPolygonContact));
   --  }
   --

   procedure destroy (contact : access b2Contact'Class)
   is
      type b2EdgeAndPolygonContact_ptr is access all b2EdgeAndPolygonContact;

      procedure free is new ada.unchecked_Deallocation (b2EdgeAndPolygonContact, b2EdgeAndPolygonContact_ptr);

      Self : b2EdgeAndPolygonContact_ptr := b2EdgeAndPolygonContact_ptr (contact);
   begin
      Self.destruct;
      free (Self);
   end destroy;



   --  b2EdgeAndPolygonContact::b2EdgeAndPolygonContact(b2Fixture* fixtureA, b2Fixture* fixtureB)
   --  : b2Contact(fixtureA, 0, fixtureB, 0)
   --  {
   --    b2Assert(m_fixtureA->GetType() == b2Shape::e_edge);
   --    b2Assert(m_fixtureB->GetType() == b2Shape::e_polygon);
   --  }
   --

   function to_b2EdgeAndPolygonContact (fixtureA : access b2Fixture;
                                        fixtureB : access b2Fixture) return b2EdgeAndPolygonContact
   is
      use b2_Shape;

      Self : aliased b2EdgeAndPolygonContact;
   begin
      define (Self, fixtureA, 0,
                    fixtureB, 0);

      pragma assert (Self.getFixtureA.getType = b2_Shape.e_edge);
      pragma assert (Self.getFixtureB.getType = b2_Shape.e_polygon);

      return Self;
   end to_b2EdgeAndPolygonContact;




   --  void b2EdgeAndPolygonContact::Evaluate (b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB)
   --  {
   --    b2CollideEdgeAndPolygon (manifold,
   --                            (b2EdgeShape*)m_fixtureA->GetShape(),    xfA,
   --                            (b2PolygonShape*)m_fixtureB->GetShape(), xfB);
   --  }
   --

   overriding
   procedure evaluate (Self : in out b2EdgeAndPolygonContact;   manifold : in out b2Manifold;
                                                                xfA, xfB : in     b2Transform)
   is
      use b2_polygon_Shape,
          b2_edge_Shape,
          b2_collide_Edge;

      type b2edgeShape_ptr    is access all b2edgeShape;
      type b2polygonShape_ptr is access all b2polygonShape;

      edge    : constant access b2EdgeShape    := b2EdgeShape_ptr    (Self.getFixtureA.getShape);
      polygon : constant access b2polygonShape := b2polygonShape_ptr (Self.getFixtureB.getShape);
   begin
      b2CollideEdgeAndPolygon (manifold, edge   .all, xfA,
                                         polygon.all, xfB);
   end evaluate;


end b2_edge_polygon_Contact;
