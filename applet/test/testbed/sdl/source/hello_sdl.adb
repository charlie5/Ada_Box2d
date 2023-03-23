with
     testbed.SDL_Drawer,
     b2_Draw,
     b2_Math,
     b2_Settings,

     SDL.Events.Events,
     SDL.Log,
     SDL.Video.Rectangles,

     Interfaces.C;

--  with ada.Text_IO;
--  use  ada.Text_IO;


procedure hello_SDL
is
   use testbed.SDL_Drawer,
       SDL.Video.Rectangles,
       b2_Math,
       b2_Draw,
       b2_Settings,
       Interfaces.C;

   use type SDL.Events.Event_Types,
            Interfaces.C.int;

   Drawer     : sdlDrawer;

   offX       : Real := 0.0;
   offY       : Real := 0.0;
   offX_Delta : Real := 1.0;
   offY_Delta : Real := 1.0;

   Line       : constant Line_Segment := (Start  => ( 0,  0),
                                          Finish => (50, 50));

   --  Polygon    : b2Vec2s := ((100.0,   0.0) * 4.5,
   --                           (200.0, 100.0) * 4.5,
   --                           (  0.0, 100.0) * 4.5);

   Polygon    : constant b2Vec2s := [(  0.0,   0.0) * 1.5,
                                     (100.0,   0.0) * 1.5,
                                     (100.0,   5.0) * 1.5,
                                     (  0.0,   5.0) * 1.5];

   new_Event_detected : Boolean;
   Event              : SDL.Events.Events.Events;

   White : constant b2Color := (255.0, 255.0, 255.0, 255.0);
   Axis  : constant b2Vec2  := (0.0, 0.0);

begin
   Main:
   loop
      new_Event_detected :=SDL.Events.Events.Poll (Event);

      if         new_Event_detected
        and then Event.Common.Event_Type = SDL.Events.Quit
      then
         exit Main;
      end if;

      Renderer.clear;
      Renderer.copy (Texture);

      Drawer.drawCircle (Center => (50.0, 50.0),
                         Radius => 50.0,
                         Color  => White);

      Drawer.drawSolidCircle (Center => (50.0 + offX, 50.0 + offY),
                              Radius => 50.0,
                              Axis   => Axis,
                              Color  => White);

      Drawer.drawSegment (p1    => ( 0.0,  0.0),
                          p2    => (50.0, 50.0),
                          Color => White);

      --  drawPolygon (Renderer, Polygon);
      Drawer.drawSolidPolygon (Vertices => Polygon,
                               Color    => White);

      if    offX <   0.0 then offX_Delta :=  1.0;
      elsif offX > 700.0 then offX_Delta := -1.0;
      end if;

      if    offY <   0.0 then offY_Delta :=  1.0;
      elsif offY > 700.0 then offY_Delta := -1.0;
      end if;

      offX := offX + offX_Delta;
      offY := offY + offY_Delta;

      Renderer.Present;
      delay (1.0 / 60.0) / 8.0;
   end loop Main;

end hello_SDL;
