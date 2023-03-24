with
     SDL.Events.Events,
     SDL.Log,
     SDL.Video.Pixel_Formats,
     SDL.Video.Textures.Makers,
     SDL.Video.Windows.Makers,
     SDL.Video.Palettes,

     ada.Finalization;


--  with ada.Text_IO;
--  use  ada.Text_IO;


package body testbed.SDL_Drawer
is
   use sdl.Video.Rectangles;



   W                :          SDL.Video.Windows.Window;
   W_Size           : constant SDL.Positive_Sizes          := (800, 640);
   the_Renderer     : aliased  SDL.Video.Renderers.Renderer;

   use type SDL.Events.Event_Types,
            Interfaces.C.int;



   function "+" (Color : in b2Color) return SDL.Video.Palettes.Colour
   is
      use sdl.Video.Palettes;
   begin
      return (Colour_Component (Color.r),
              Colour_Component (Color.g),
              Colour_Component (Color.b),
              Colour_Component (Color.a));
   end "+";



   -- Flip 'y' coord to make the origin at the bottom left.
   --

   function "+" (y : in C.int) return C.int
   is
      Height : constant C.int := W.get_Size.Height;
   begin
      return Height - y - 1;
   end "+";



   overriding
   procedure drawPolygon (Self : in out sdlDrawer;   vertices    : in b2Vec2s;
                                                     --  vertexCount : in Natural;
                                                     Color       : in b2Color)
   is
      use SDL.Video.Renderers;

      Start,
      Finish : b2Vec2;

   begin
      set_draw_Colour (the_Renderer, +Color);

      for i in Vertices'Range
      loop
         if i /= Vertices'Last
         then
            Start  := (Vertices (i + 0).x,
                       Vertices (i + 0).y);

            Finish := (Vertices (i + 1).x,
                       Vertices (i + 1).y);
         else
            Start  := (Vertices (i).x,
                       Vertices (i).y);

            Finish := (Vertices (0).x,
                       Vertices (0).y);
         end if;

         Self.drawSegment (Start, Finish, Color);
      end loop;
   end drawPolygon;

   --  procedure drawPolygon (Renderer : in out SDL.Video.Renderers.Renderer;
   --                         Vertices : in     b2Vec2s)
   --  is
   --     use SDL.Video.Renderers;
   --     Line : line_Segment;
   --  begin
   --     for i in Vertices'Range
   --     loop
   --        if i /= Vertices'Last
   --        then
   --           Line := (Start  => (int (Vertices (i + 0).x),
   --                               int (Vertices (i + 0).y)),
   --                    Finish => (int (Vertices (i + 1).x),
   --                               int (Vertices (i + 1).y)));
   --        else
   --           Line := (Start  => (int (Vertices (i).x),
   --                               int (Vertices (i).y)),
   --                    Finish => (int (Vertices (0).x),
   --                               int (Vertices (0).y)));
   --        end if;
   --
   --        drawSegment (Renderer, Line);
   --     end loop;
   --  end drawPolygon;






   --    Draw a solid closed polygon provided in CCW order.
   --    virtual void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;
   --

   overriding
   procedure drawSolidPolygon (Self : in out sdlDrawer;   vertices    : in b2Vec2s;
                                                          --  vertexCount : in Natural;
                                                          Color       : in b2Color)
   is
      use b2_Math.Functions;

      Center  : b2Vec2  := (0.0, 0.0);
      Offsets : b2Vec2s (Vertices'Range);

      min_Offset : b2Vec2 := (0.0, 0.0);
      max_Offset : b2Vec2 := (0.0, 0.0);

      Iterations : Integer; --  := 50;

   begin
      for j in Vertices'Range
      loop
         Center := Center + Vertices (j);
      end loop;

      Center.x := Center.x / Real (Vertices'Length);
      Center.y := Center.y / Real (Vertices'Length);

      for j in Vertices'Range
      loop
         Offsets (j) := Vertices (j) - Center;

         min_Offset.x := Real'min (min_Offset.x, Offsets (j).x);
         min_Offset.y := Real'min (min_Offset.y, Offsets (j).y);

         max_Offset.x := Real'max (max_Offset.x, Offsets (j).x);
         max_Offset.y := Real'max (max_Offset.y, Offsets (j).y);
      end loop;

      --  put_Line (Center'Image);


      --  Iterations :=   Integer (  (max_Offset.x - min_Offset.x)
      --                             * (max_Offset.y - min_Offset.y))
      --                / 150;

      --  Iterations :=   Integer (SqRt (  (max_Offset.x - min_Offset.x)
      --                                 * (max_Offset.y - min_Offset.y))
      --                                 / 0.5);
      --  Iterations := Integer'max (Iterations, 5);
      Iterations := 5;

      --  put_Line (Iterations'Image);

      for i in 1 .. Iterations
      loop
         declare
            scaled_Vertices : b2Vec2s (Vertices'Range);
         begin
            for j in Vertices'Range
            loop
               scaled_Vertices (j) := (Center.x + Offsets (j).x * Real (i) / Real (Iterations),
                                       Center.y + Offsets (j).y * Real (i) / Real (Iterations));

            end loop;

            Self.drawPolygon (scaled_Vertices, Color);
         end;
      end loop;
   end drawSolidPolygon;



   --  procedure drawSolidPolygon (Renderer : in out SDL.Video.Renderers.Renderer;
   --                              Vertices : in     b2Vec2s)
   --  is
   --     use b2_Math.Functions;
   --
   --     Center  : b2Vec2  := (0.0, 0.0);
   --     Offsets : b2Vec2s (Vertices'Range);
   --
   --     min_Offset : b2Vec2 := (0.0, 0.0);
   --     max_Offset : b2Vec2 := (0.0, 0.0);
   --
   --     Iterations : Integer; --  := 50;
   --
   --  begin
   --     for j in Vertices'Range
   --     loop
   --        Center := Center + Vertices (j);
   --     end loop;
   --
   --     Center.x := Center.x / Real (Vertices'Length);
   --     Center.y := Center.y / Real (Vertices'Length);
   --
   --     for j in Vertices'Range
   --     loop
   --        Offsets (j) := Vertices (j) - Center;
   --
   --        min_Offset.x := Real'min (min_Offset.x, Offsets (j).x);
   --        min_Offset.y := Real'min (min_Offset.y, Offsets (j).y);
   --
   --        max_Offset.x := Real'max (max_Offset.x, Offsets (j).x);
   --        max_Offset.y := Real'max (max_Offset.y, Offsets (j).y);
   --     end loop;
   --
   --     --  put_Line (Center'Image);
   --
   --
   --     --  Iterations :=   Integer (  (max_Offset.x - min_Offset.x)
   --     --                             * (max_Offset.y - min_Offset.y))
   --     --                / 150;
   --
   --     --  Iterations :=   Integer (SqRt (  (max_Offset.x - min_Offset.x)
   --     --                                 * (max_Offset.y - min_Offset.y))
   --     --                                 / 0.5);
   --     --  Iterations := Integer'max (Iterations, 5);
   --     Iterations := 5;
   --
   --     --  put_Line (Iterations'Image);
   --
   --     for i in 1 .. Iterations
   --     loop
   --        declare
   --           scaled_Vertices : b2Vec2s (Vertices'Range);
   --        begin
   --           for j in Vertices'Range
   --           loop
   --              scaled_Vertices (j) := (Center.x + Offsets (j).x * Real (i) / Real (Iterations),
   --                                      Center.y + Offsets (j).y * Real (i) / Real (Iterations));
   --
   --           end loop;
   --
   --           Self.drawPolygon (scaled_Vertices, Color => );
   --        end;
   --     end loop;
   --  end drawSolidPolygon;









   --    Draw a circle.
   --    virtual void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) = 0;
   --

   overriding
   procedure drawCircle (Self : in out sdlDrawer;   Center : in b2Vec2;
                                                    Radius : in Real;
                                                    Color  : in b2Color)
   is
      use SDL.Video.Renderers;

      diameter : constant C.int := C.int (radius * 2.0);

      CenterX  : constant c.int := C.int (Center.x);
      CenterY  : constant c.int := C.int (Center.y);

      -- Point : SDL.Video.Rectangles.Point;
      x  : C.int := C.int (Radius) - 1;
      y  : C.int := 0;

      tx : C.int := 1;
      ty : C.int := 1;

      error : C.int := tx - diameter;

   begin
      set_draw_Colour (the_Renderer, +Color);

      while x >= y
      loop
         --  Each of the following renders an octant of the circle
         --
         draw (Renderer.all, Point' (CenterX + x,  +CenterY - y));
         draw (Renderer.all, Point' (CenterX + x,  +CenterY + y));
         draw (Renderer.all, Point' (CenterX - x,  +CenterY - y));
         draw (Renderer.all, Point' (CenterX - x,  +CenterY + y));
         draw (Renderer.all, Point' (CenterX + y,  +CenterY - x));
         draw (Renderer.all, Point' (CenterX + y,  +CenterY + x));
         draw (Renderer.all, Point' (CenterX - y,  +CenterY - x));
         draw (Renderer.all, Point' (CenterX - y,  +CenterY + x));

         if error <= 0
         then
            y     := y + 1;
            error := error + ty;
            ty    := ty + 2;
         end if;

         if error > 0
         then
            x     := x - 1;
            tx    := tx + 2;
            error := error + (tx - diameter);
         end if;
      end loop;
   end drawCircle;



   --  procedure drawCircle (renderer : in out SDL.Video.Renderers.Renderer;
   --                        centreX  : in     int;
   --                        centreY  : in     int;
   --                        radius   : in     int)
   --  is
   --     use SDL.Video.Renderers;
   --
   --     diameter : constant C.int := radius * 2;
   --
   --     -- Point : SDL.Video.Rectangles.Point;
   --     x  : C.int := radius - 1;
   --     y  : C.int := 0;
   --
   --     tx : C.int := 1;
   --     ty : C.int := 1;
   --
   --     error : C.int := tx - diameter;
   --
   --  begin
   --     while x >= y
   --     loop
   --        --  Each of the following renders an octant of the circle
   --        --
   --        draw (Renderer, Point' (centreX + x, centreY - y));
   --        draw (Renderer, Point' (centreX + x, centreY + y));
   --        draw (Renderer, Point' (centreX - x, centreY - y));
   --        draw (Renderer, Point' (centreX - x, centreY + y));
   --        draw (Renderer, Point' (centreX + y, centreY - x));
   --        draw (Renderer, Point' (centreX + y, centreY + x));
   --        draw (Renderer, Point' (centreX - y, centreY - x));
   --        draw (Renderer, Point' (centreX - y, centreY + x));
   --
   --        if error <= 0
   --        then
   --           y     := y + 1;
   --           error := error + ty;
   --           ty    := ty + 2;
   --        end if;
   --
   --        if error > 0
   --        then
   --           x     := x - 1;
   --           tx    := tx + 2;
   --           error := error + (tx - diameter);
   --        end if;
   --     end loop;
   --  end drawCircle;








   --    Draw a solid circle.
   --    virtual void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) = 0;
   --

   overriding
   procedure drawSolidCircle (Self : in out sdlDrawer;   Center : in b2Vec2;
                                                         Radius : in Real;
                                                         Axis   : in b2Vec2;
                                                         Color  : in b2Color)
   is
      radius_Delta : Real := Radius / 5.0;
      R            : Real := Radius;
   begin
      Self.drawCircle (Center => Center,
                       Radius => 2.0,
                       Color  => Color);

      --  for r in 0 .. radius
      for i in int' (1) .. 5
      loop
         Self.drawCircle (Center => Center,
                          Radius => R, --Radius / Real (i),
                          Color  => Color);

         R := R - radius_Delta;
         --  Self.drawCircle (renderer,
         --                   centreX,
         --                   centreY,
         --                   radius => radius / i);
         --  drawCircle (renderer,
         --              centreX + 1,
         --              centreY,
         --              radius => r);
         --  drawCircle (renderer,
         --              centreX,
         --              centreY + 1,
         --              radius => r);
      end loop;

   end drawSolidCircle;



   --  procedure drawSolidCircle (renderer : in out SDL.Video.Renderers.Renderer;
   --                             centreX  : in     int;
   --                             centreY  : in     int;
   --                             radius   : in     int)
   --  is
   --  begin
   --     drawCircle (renderer,
   --                 centreX,
   --                 centreY,
   --                 radius => 2);
   --
   --     --  for r in 0 .. radius
   --     for i in int' (1) .. 5
   --     loop
   --        drawCircle (renderer,
   --                    centreX,
   --                    centreY,
   --                    radius => radius / i);
   --        --  drawCircle (renderer,
   --        --              centreX + 1,
   --        --              centreY,
   --        --              radius => r);
   --        --  drawCircle (renderer,
   --        --              centreX,
   --        --              centreY + 1,
   --        --              radius => r);
   --     end loop;
   --  end drawSolidCircle;







   --    Draw a line segment.
   --    virtual void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) = 0;
   --

   overriding
   procedure drawSegment (Self : in out sdlDrawer;   p1, p2 : in b2Vec2;
                                                     Color  : in b2Color)
   is
      use SDL.Video.Renderers;

      Line : constant line_Segment := (Start  => (C.int (p1.x),  +C.int (p1.y)),
                                       Finish => (C.int (p2.x),  +C.int (p2.y)));
   begin
      set_draw_Colour (the_Renderer, +Color);
      draw            (Renderer.all, Line);
   end drawSegment;


   --  procedure drawSegment (renderer : in out SDL.Video.Renderers.Renderer;
   --                         Line     : in     line_Segment)
   --  is
   --     use SDL.Video.Renderers;
   --  begin
   --     draw (Renderer, Line);
   --  end drawSegment;









   --    Draw a transform. Choose your own length scale.
   --    @param xf a transform.
   --    virtual void DrawTransform(const b2Transform& xf) = 0;
   --

   overriding
   procedure drawTransform (Self : in out sdlDrawer;   xf : in b2Transform)
   is
   begin
      raise Program_Error with "TODO";
   end drawTransform;






   --    Draw a point.
   --    virtual void DrawPoint(const b2Vec2& p, float size, const b2Color& color) = 0;
   --

   overriding
   procedure drawPoint (Self : in out sdlDrawer;   p     : in b2Vec2;
                                                   Size  : in Real;
                                                   Color : in b2Color)
   is
      use SDL.Video.Renderers;
   begin
      set_draw_Colour (the_Renderer, +Color);
      draw            (Renderer.all, Point' ( C.int (p.x),
                                             +C.int (p.y)));
   end drawPoint;






   function Renderer return access SDL.Video.Renderers.Renderer
   is
   begin
      return the_Renderer'Access;
   end Renderer;






   Colour : constant SDL.Video.Palettes.Colour := (255, 255, 255, 255);

   offX       : int := 0;
   offY       : int := 0;
   offX_Delta : int := 1;
   offY_Delta : int := 1;

   Line       : constant Line_Segment := (Start  => ( 0,  0),
                                          Finish => (50, 50));

   --  Polygon    : b2Vec2s := ((100.0,   0.0) * 4.5,
   --                           (200.0, 100.0) * 4.5,
   --                           (  0.0, 100.0) * 4.5);

   Polygon    : constant b2Vec2s := [(  0.0,   0.0) * 1.5,
                                     (100.0,   0.0) * 1.5,
                                     (100.0,   5.0) * 1.5,
                                     (  0.0,   5.0) * 1.5];




   ---------------------
   -- Clean up and exit.
   --
   use ada.Finalization;

   type Closure is new ada.Finalization.Controlled with null record;

   overriding
   procedure finalize (Object : in out Closure)
   is
   begin
      W  .finalize;
      SDL.finalise;
   end finalize;

   Closer : Closure;



   -------------
   -- Setup SDL.
   --

   use SDL.Video.Renderers;

begin
   SDL.Log.Set (Category => SDL.Log.Application, Priority => SDL.Log.Debug);

   if SDL.Initialise = True
   then
      SDL.Video.Windows.Makers.Create
        (Win      => W,
         Title    => "Hello, World",
         Position => SDL.Natural_Coordinates' (X => 100, Y => 100),
         Size     => W_Size,
         Flags    => SDL.Video.Windows.Resizable);

      SDL.Video.Renderers.Makers.Create (the_Renderer, W);

      --  Set the texture to the same size as the window, as the window scales, the texture
      --  will also scale, it will *not* be rebuilt to match the new window size.
      --
      SDL.Video.Textures.Makers.Create
        (Tex      => Texture,
         Renderer => the_Renderer,
         Format   => SDL.Video.Pixel_Formats.Pixel_Format_ARGB_8888,
         Kind     => SDL.Video.Textures.Streaming,
         Size     => W_Size);

      set_draw_Colour (the_Renderer, Colour);

   else
      raise program_Error with "Failed to initialize SDL.";
   end if;


end testbed.SDL_Drawer;
