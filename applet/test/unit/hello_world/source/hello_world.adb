with
     b2_Contact,
     b2_Fixture,
     b2_polygon_Shape,
     b2_Body,
     b2_World,
     b2_Math,
     b2_Settings,

     ada.Text_IO;


procedure hello_World
--
--  This is a simple example of building and running a simulation
--  using Box2D. Here we create a large ground box and a small dynamic box.
--  There are no graphics for this example. Box2D is meant to be used
--  with your rendering engine in your game engine.
--
is
   use b2_Contact,
       ada.Text_IO;

begin
   put_Line ("hello world");
   put_Line ("begin contact");

   declare
      use b2_polygon_Shape,
          b2_Body,
          b2_Fixture,
          b2_World,
          b2_Math,
          b2_Settings;

      --    // Define the gravity vector.
      --
      --    b2Vec2 gravity(0.0f, -10.0f);
      --

      gravity : constant b2Vec2 := (0.0, -10.0);


      --    // Construct a world object, which will hold and simulate the rigid bodies.
      --
      --    b2World world(gravity);
      --

      world : b2World := to_b2World (gravity);



      --    // Define the ground body.
      --
      --    b2BodyDef groundBodyDef;
      --    groundBodyDef.position.Set(0.0f, -10.0f);

      groundBodyDef : b2BodyDef := to_b2BodyDef;


      --    // Call the body factory which allocates memory for the ground body
      --    // from a pool and creates the ground box shape (also from a pool).
      --    // The body is also added to the world.
      --
      --    b2Body* groundBody = world.CreateBody(&groundBodyDef);
      --

      groundBody : access b2Body;


      --    // Define the ground box shape.
      --
      --    b2PolygonShape groundBox;
      --

      groundBox  : aliased b2PolygonShape := to_b2polygonShape;


      bodyDef    :         b2BodyDef := to_b2BodyDef;
      the_Body   : access  b2Body;

      dynamicBox : aliased b2PolygonShape := to_b2polygonShape;
      fixtureDef :         b2FixtureDef;

      Fixture    :         b2Fixture_ptr with unreferenced;

   begin
      groundBodyDef.Position := (0.0, -10.0);
      groundBody             := world.createBody (groundBodyDef);


      --    // The extents are the half-widths of the box.
      --
      --    groundBox.SetAsBox(50.0f, 10.0f);
      --

      groundBox.setAsBox (50.0, 10.0);


      --    // Add the ground fixture to the ground body.
      --
      --    groundBody->CreateFixture(&groundBox, 0.0f);
      --

      Fixture := groundBody.createFixture (groundBox'unchecked_Access, 0.0);


      --    // Define the dynamic body. We set its position and call the body factory.
      --
      --    bodyDef.type = b2_dynamicBody;
      --    bodyDef.position.Set(0.0f, 4.0f);
      --    b2Body* body = world.CreateBody(&bodyDef);
      --

      bodyDef.Kind     := b2_dynamicBody;
      bodyDef.position := (0.0, 4.0);
      the_body         := World.createBody (bodyDef);


      --    // Define another box shape for our dynamic body.
      --
      --    dynamicBox.SetAsBox(1.0f, 1.0f);
      --

      dynamicBox.setAsBox (1.0, 1.0);

      --    // Define the dynamic body fixture.
      --
      --    fixtureDef.shape = &dynamicBox;
      --

      fixtureDef.shape := dynamicBox'unchecked_Access;

      --    // Set the box density to be non-zero, so it will be dynamic.
      --
      --    fixtureDef.density = 1.0f;
      --

      fixtureDef.density := 1.0;

      --    // Override the default friction.
      --
      --    fixtureDef.friction = 0.3f;
      --

      fixtureDef.friction := 0.3;

      --    // Add the shape to the body.
      --
      --    body->CreateFixture(&fixtureDef);
      --

      Fixture := the_Body.createFixture (fixtureDef);

      --    // Prepare for simulation. Typically we use a time step of 1/60 of a
      --    // second (60Hz) and 10 iterations. This provides a high quality simulation
      --    // in most game scenarios.
      --
      declare
         --    float timeStep = 1.0f / 60.0f;
         --    int32 velocityIterations = 6;
         --    int32 positionIterations = 2;
         --

         timeStep           : constant Real    := 1.0 / 60.0;
         velocityIterations : constant Natural := 6;
         positionIterations : constant Natural := 2;

         --    b2Vec2 position = body->GetPosition();
         --    float angle = body->GetAngle();
         --

         position : b2Vec2 := the_Body.getPosition;
         angle    : Real   := the_Body.getAngle;

      begin
         --    // This is our little game loop.
         --
         --    for (int32 i = 0; i < 60; ++i)
         --    {
         --       // Instruct the world to perform a single step of simulation.
         --       // It is generally best to keep the time step and iterations fixed.
         --       world.Step(timeStep, velocityIterations, positionIterations);
         --
         --       // Now print the position and angle of the body.
         --       position = body->GetPosition();
         --       angle = body->GetAngle();
         --
         --       printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
         --    }
         --

         for i in 0 .. 59
         loop
            -- Instruct the world to perform a single step of simulation.
            -- It is generally best to keep the time step and iterations fixed.
            --
            World.step (timeStep, velocityIterations,
                                  positionIterations);


            -- Now print the position and angle of the body.
            --
            position := the_Body.getPosition;
            angle    := the_Body.getAngle;

            put_Line (  "("  & position.x'Image
                      & ","  & position.y'Image
                      & ") " & angle     'Image);
         end loop;



         -- When the world destructor is called, all bodies and joints are freed. This can
         -- create orphaned pointers, so be careful about your world management.
         --
         pragma assert (abs position.x        < 0.01);
         pragma assert (abs position.y - 1.01 < 0.01);
         pragma assert (abs angle             < 0.01);
      end;
   end;

   put_Line ("Success");
end hello_World;



--  #include "box2d/box2d.h"
--  #define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
--  #include "doctest.h"
--  #include <stdio.h>
--
--  // This is a simple example of building and running a simulation
--  // using Box2D. Here we create a large ground box and a small dynamic
--  // box.
--  // There are no graphics for this example. Box2D is meant to be used
--  // with your rendering engine in your game engine.
--  DOCTEST_TEST_CASE("hello world")
--  {
--    // Define the gravity vector.
--    b2Vec2 gravity(0.0f, -10.0f);
--
--    // Construct a world object, which will hold and simulate the rigid bodies.
--    b2World world(gravity);
--
--    // Define the ground body.
--    b2BodyDef groundBodyDef;
--    groundBodyDef.position.Set(0.0f, -10.0f);
--
--    // Call the body factory which allocates memory for the ground body
--    // from a pool and creates the ground box shape (also from a pool).
--    // The body is also added to the world.
--    b2Body* groundBody = world.CreateBody(&groundBodyDef);
--
--    // Define the ground box shape.
--    b2PolygonShape groundBox;
--
--    // The extents are the half-widths of the box.
--    groundBox.SetAsBox(50.0f, 10.0f);
--
--    // Add the ground fixture to the ground body.
--    groundBody->CreateFixture(&groundBox, 0.0f);
--
--    // Define the dynamic body. We set its position and call the body factory.
--    b2BodyDef bodyDef;
--    bodyDef.type = b2_dynamicBody;
--    bodyDef.position.Set(0.0f, 4.0f);
--    b2Body* body = world.CreateBody(&bodyDef);
--
--    // Define another box shape for our dynamic body.
--    b2PolygonShape dynamicBox;
--    dynamicBox.SetAsBox(1.0f, 1.0f);
--
--    // Define the dynamic body fixture.
--    b2FixtureDef fixtureDef;
--    fixtureDef.shape = &dynamicBox;
--
--    // Set the box density to be non-zero, so it will be dynamic.
--    fixtureDef.density = 1.0f;
--
--    // Override the default friction.
--    fixtureDef.friction = 0.3f;
--
--    // Add the shape to the body.
--    body->CreateFixture(&fixtureDef);
--
--    // Prepare for simulation. Typically we use a time step of 1/60 of a
--    // second (60Hz) and 10 iterations. This provides a high quality simulation
--    // in most game scenarios.
--    float timeStep = 1.0f / 60.0f;
--    int32 velocityIterations = 6;
--    int32 positionIterations = 2;
--
--    b2Vec2 position = body->GetPosition();
--    float angle = body->GetAngle();
--
--    // This is our little game loop.
--    for (int32 i = 0; i < 60; ++i)
--    {
--       // Instruct the world to perform a single step of simulation.
--       // It is generally best to keep the time step and iterations fixed.
--       world.Step(timeStep, velocityIterations, positionIterations);
--
--       // Now print the position and angle of the body.
--       position = body->GetPosition();
--       angle = body->GetAngle();
--
--       printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
--    }
--
--    // When the world destructor is called, all bodies and joints are freed. This can
--    // create orphaned pointers, so be careful about your world management.
--
--    CHECK(b2Abs(position.x) < 0.01f);
--    CHECK(b2Abs(position.y - 1.01f) < 0.01f);
--    CHECK(b2Abs(angle) < 0.01f);
--  }
