with
     b2_Shape,
     b2_polygon_Shape,
     b2_Math,
     b2_Settings,
     b2_Common,

     ada.Text_IO;


procedure collision_Test
is
   use b2_polygon_Shape,
       b2_Shape,
       b2_Math,
       b2_Settings,
       b2_Common,
       ada.Text_IO;

   center : constant b2Vec2 := (100.0, -50.0);
   hx     : constant Real   :=    0.5;
   hy     : constant Real   :=    1.5;
   angle1 : constant Real   :=    0.25;


   --       // Data from issue #422. Not used because the data exceeds accuracy limits.
   --
   --       //const b2Vec2 center(-15000.0f, -15000.0f);
   --       //const float hx = 0.72f, hy = 0.72f;
   --       //const float angle1 = 0.0f;

   polygon1 : b2PolygonShape;
   polygon2 : b2PolygonShape;

   absTol   : constant Real := 2.0 * b2_epsilon;
   relTol   : constant Real := 2.0 * b2_epsilon;

   mass     : constant Real := 4.0 * hx * hy;
   inertia  : constant Real :=  (mass / 3.0) * (hx * hx + hy * hy)
                               + mass * b2Dot (center, center);


begin
   put_Line ("collision test");
   put_Line ("polygon mass data");

   declare
   begin
      polygon1.setAsBox (hx, hy, center, angle1);

      pragma assert (  abs (polygon1.m_centroid.x - center.x)
                     < absTol + relTol * abs center.x);

      pragma assert (  abs (polygon1.m_centroid.y - center.y)
                     < absTol + relTol * abs center.y);
   end;


   declare
      vertices : b2Vec2s (0 .. 3);
   begin
      vertices (0) := (center.x - hx,  center.y - hy);
      vertices (1) := (center.x + hx,  center.y - hy);
      vertices (2) := (center.x - hx,  center.y + hy);
      vertices (3) := (center.x + hx,  center.y + hy);

      polygon2.set (vertices, 4);

      pragma assert (abs (polygon2.m_centroid.x - center.x) < absTol + relTol * abs center.x);
      pragma assert (abs (polygon2.m_centroid.y - center.y) < absTol + relTol * abs center.y);
   end;


   declare
      massData1 : b2MassData;
   begin
      polygon1.ComputeMass(massData1, 1.0);

      pragma assert (abs (massData1.center.x - center.x) < absTol + relTol * abs center.x);
      pragma assert (abs (massData1.center.y - center.y) < absTol + relTol * abs center.y);
      pragma assert (abs (massData1.mass - mass)         < 20.0 * (absTol + relTol * mass));
      pragma assert (abs (massData1.I - inertia)         < 40.0 * (absTol + relTol * inertia));
   end;


   -- TODO: This appears identical to the prior test.
   --
   declare
      massData2 : b2MassData;
   begin
      polygon2.ComputeMass(massData2, 1.0);

      pragma assert ((abs (massData2.center.x - center.x) < absTol + relTol * abs center.x));
      pragma assert ((abs (massData2.center.y - center.y) < absTol + relTol * abs center.y));
      pragma assert ((abs (massData2.mass - mass)         < 20.0 * (absTol + relTol * mass)));
      pragma assert ((abs (massData2.I - inertia)         < 40.0 * (absTol + relTol * inertia)));
   end;

   put_Line ("Success");
end collision_Test;





--  #include "box2d/box2d.h"
--  #include "doctest.h"
--  #include <stdio.h>
--
--  // Unit tests for collision algorithms
--  DOCTEST_TEST_CASE("collision test")
--  {
--    SUBCASE("polygon mass data")
--    {
--       const b2Vec2 center(100.0f, -50.0f);
--       const float hx = 0.5f, hy = 1.5f;
--       const float angle1 = 0.25f;
--
--       // Data from issue #422. Not used because the data exceeds accuracy limits.
--       //const b2Vec2 center(-15000.0f, -15000.0f);
--       //const float hx = 0.72f, hy = 0.72f;
--       //const float angle1 = 0.0f;
--
--       b2PolygonShape polygon1;
--       polygon1.SetAsBox(hx, hy, center, angle1);
--
--       const float absTol = 2.0f * b2_epsilon;
--       const float relTol = 2.0f * b2_epsilon;
--
--       CHECK(b2Abs(polygon1.m_centroid.x - center.x) < absTol + relTol * b2Abs(center.x));
--       CHECK(b2Abs(polygon1.m_centroid.y - center.y) < absTol + relTol * b2Abs(center.y));
--
--       b2Vec2 vertices[4];
--       vertices[0].Set(center.x - hx, center.y - hy);
--       vertices[1].Set(center.x + hx, center.y - hy);
--       vertices[2].Set(center.x - hx, center.y + hy);
--       vertices[3].Set(center.x + hx, center.y + hy);
--
--       b2PolygonShape polygon2;
--       polygon2.Set(vertices, 4);
--
--       CHECK(b2Abs(polygon2.m_centroid.x - center.x) < absTol + relTol * b2Abs(center.x));
--       CHECK(b2Abs(polygon2.m_centroid.y - center.y) < absTol + relTol * b2Abs(center.y));
--
--       const float mass = 4.0f * hx * hy;
--       const float inertia = (mass / 3.0f) * (hx * hx + hy * hy) + mass * b2Dot(center, center);
--
--       b2MassData massData1;
--       polygon1.ComputeMass(&massData1, 1.0f);
--
--       CHECK(b2Abs(massData1.center.x - center.x) < absTol + relTol * b2Abs(center.x));
--       CHECK(b2Abs(massData1.center.y - center.y) < absTol + relTol * b2Abs(center.y));
--       CHECK(b2Abs(massData1.mass - mass) < 20.0f * (absTol + relTol * mass));
--       CHECK(b2Abs(massData1.I - inertia) < 40.0f * (absTol + relTol * inertia));
--
--       b2MassData massData2;
--       polygon2.ComputeMass(&massData2, 1.0f);
--
--       CHECK(b2Abs(massData2.center.x - center.x) < absTol + relTol * b2Abs(center.x));
--       CHECK(b2Abs(massData2.center.y - center.y) < absTol + relTol * b2Abs(center.y));
--       CHECK(b2Abs(massData2.mass - mass) < 20.0f * (absTol + relTol * mass));
--       CHECK(b2Abs(massData2.I - inertia) < 40.0f * (absTol + relTol * inertia));
--    }
--  }
