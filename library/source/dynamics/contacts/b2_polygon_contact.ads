with
     b2_Contact;


package b2_polygon_Contact
is
   procedure dummy;


   --
--  class b2BlockAllocator;
--
--  class b2PolygonContact : public b2Contact
--  {
--  public:
--    static b2Contact* Create(  b2Fixture* fixtureA, int32 indexA,
--                         b2Fixture* fixtureB, int32 indexB, b2BlockAllocator* allocator);
--    static void Destroy(b2Contact* contact, b2BlockAllocator* allocator);
--
--    b2PolygonContact(b2Fixture* fixtureA, b2Fixture* fixtureB);
--    ~b2PolygonContact() {}
--
--    void Evaluate(b2Manifold* manifold, const b2Transform& xfA, const b2Transform& xfB) override;
--  };
--
end b2_polygon_Contact;