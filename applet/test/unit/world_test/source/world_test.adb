with
     b2_world_Callbacks,
     b2_Contact,
     b2_Fixture,
     b2_circle_Shape,
     b2_Body,
     b2_World,
     b2_Math,
     b2_Settings,

     ada.Text_IO;


procedure world_Test
is
   use b2_world_Callbacks,
       b2_Contact,
       ada.Text_IO;


   begin_contact : Boolean := False;

   type myContactListener is new b2ContactListener with null record;

   overriding
   procedure beginContact (Self : in out myContactListener;   contact : in out b2_Contact.b2Contact'Class)
   is
   begin
      begin_contact := True;
   end beginContact;


begin
   put_Line ("world test");
   put_Line ("begin contact");

   declare
      use b2_circle_Shape,
          b2_Body,
          b2_Fixture,
          b2_World,
          b2_Math,
          b2_Settings;

      world    : aliased b2World          := to_b2World (Gravity => b2Vec2' (0.0, -10.0));
      listener : aliased myContactListener;

      circle   : aliased b2CircleShape := to_b2circleShape;
      bodyDef  :         b2BodyDef     := to_b2BodyDef;

      bodyA : access b2Body;
      bodyB : access b2Body;

      timeStep           : Real;
      velocityIterations : Natural;
      positionIterations : Natural;

      fA, fB : b2Fixture_ptr
        with unreferenced;

   begin
      world.setContactListener (listener'unchecked_Access);

      circle.m_radius := 5.0;
      bodyDef.Kind    := b2_dynamicBody;

      bodyA := world.createBody (bodyDef);
      bodyB := world.createBody (bodyDef);

      fA := bodyA.createFixture (circle'unchecked_Access, 0.0);
      fB := bodyB.createFixture (circle'unchecked_Access, 0.0);

      bodyA.setTransform (b2Vec2' (  0.0, 0.0),  0.0);
      bodyB.setTransform (b2Vec2' (100.0, 0.0),  0.0);

      timeStep           := 1.0 / 60.0;
      velocityIterations := 6;
      positionIterations := 2;

      world.step (timeStep, velocityIterations,
                            positionIterations);

      pragma assert (world.getContactList = null);
      pragma assert (not begin_contact);

      bodyB.setTransform (b2Vec2' (1.0, 0.0),  0.0);

      world.step (timeStep, velocityIterations,
                            positionIterations);

      pragma assert (world.getContactList /= null);
      pragma assert (begin_contact);
   end;

   put_Line ("Success");
end world_Test;



--  static bool begin_contact = false;
--
--  class MyContactListener : public b2ContactListener
--  {
--  public:
--    void BeginContact(b2Contact* contact)
--    {
--       begin_contact = true;
--    }
--  };
--
--  DOCTEST_TEST_CASE ("begin contact")
--  {
--    b2World world = b2World(b2Vec2(0.0f, -10.0f));
--    MyContactListener listener;
--    world.SetContactListener(&listener);
--
--    b2CircleShape circle;
--    circle.m_radius = 5.f;
--
--    b2BodyDef bodyDef;
--    bodyDef.type = b2_dynamicBody;
--
--    b2Body* bodyA = world.CreateBody(&bodyDef);
--    b2Body* bodyB = world.CreateBody(&bodyDef);
--    bodyA->CreateFixture(&circle, 0.0f);
--    bodyB->CreateFixture(&circle, 0.0f);
--
--    bodyA->SetTransform(b2Vec2(0.f, 0.f), 0.f);
--    bodyB->SetTransform(b2Vec2(100.f, 0.f), 0.f);
--
--    const float timeStep = 1.f / 60.f;
--    const int32 velocityIterations = 6;
--    const int32 positionIterations = 2;
--
--    world.Step(timeStep, velocityIterations, positionIterations);
--
--    CHECK(world.GetContactList() == nullptr);
--    CHECK(begin_contact == false);
--
--    bodyB->SetTransform(b2Vec2(1.f, 0.f), 0.f);
--
--    world.Step(timeStep, velocityIterations, positionIterations);
--
--    CHECK(world.GetContactList() != nullptr);
--    CHECK(begin_contact == true);
--  }
