with
     b2_Math,
     ada.Text_IO;


procedure math_Test
is
   use ada.Text_IO;
begin
   put_Line ("math test");
   put_Line ("sweep");

   declare
      use b2_Math,
          b2_Math.Functions;

      -- From issue #447
      --
      sweep     : b2Sweep;
      transform : b2Transform;
   begin
      setZero (sweep.localCenter);
      set (sweep.c0, -2.0, 4.0);
      set (sweep.c,   3.0, 8.0);
      sweep.a0     := 0.5;
      sweep.a      := 5.0;
      sweep.alpha0 := 0.0;


      sweep.getTransform (transform, 0.0);

      pragma assert (transform.p.x = sweep.c0.x);
      pragma assert (transform.p.y = sweep.c0.y);
      pragma assert (transform.q.c = cos (sweep.a0));
      pragma assert (transform.q.s = sin (sweep.a0));


      sweep.getTransform (transform, 1.0);

      pragma assert (transform.p.x = sweep.c.x);
      pragma assert (transform.p.y = sweep.c.y);
      pragma assert (transform.q.c = cos (sweep.a));
      pragma assert (transform.q.s = sin (sweep.a));
   end;

   put_Line ("Success");
end math_Test;
