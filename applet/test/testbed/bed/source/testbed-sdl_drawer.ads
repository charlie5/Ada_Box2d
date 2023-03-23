with
     b2_Math,
     b2_Draw,
     b2_Settings,

     SDL.Video.Rectangles,
     SDL.Video.Renderers.Makers,
     SDL.Video.Textures,

     Interfaces.C;


package testbed.SDL_Drawer
is
   use b2_Math,
       b2_Draw,
       b2_Settings,
       Interfaces.C;

   Texture : SDL.Video.Textures.Texture;


   type sdlDrawer is new b2_Draw.b2Draw with private;


   overriding
   procedure drawPolygon (Self : in out sdlDrawer;   vertices    : in b2Vec2s;
                                                     --  vertexCount : in Natural;
                                                     Color       : in b2Color);


   --    Draw a solid closed polygon provided in CCW order.
   --    virtual void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;
   --

   overriding
   procedure drawSolidPolygon (Self : in out sdlDrawer;   vertices    : in b2Vec2s;
                                                          --  vertexCount : in Natural;
                                                          Color       : in b2Color);


   --    Draw a circle.
   --    virtual void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) = 0;
   --

   overriding
   procedure drawCircle (Self : in out sdlDrawer;   Center : in b2Vec2;
                                                    Radius : in Real;
                                                    Color  : in b2Color);


   --    Draw a solid circle.
   --    virtual void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) = 0;
   --

   overriding
   procedure drawSolidCircle (Self : in out sdlDrawer;   Center : in b2Vec2;
                                                         Radius : in Real;
                                                         Axis   : in b2Vec2;
                                                         Color  : in b2Color);


   --    Draw a line segment.
   --    virtual void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) = 0;
   --

   overriding
   procedure drawSegment (Self : in out sdlDrawer;   p1, p2 : in b2Vec2;
                                                     Color  : in b2Color);


   --    Draw a transform. Choose your own length scale.
   --    @param xf a transform.
   --    virtual void DrawTransform(const b2Transform& xf) = 0;
   --

   overriding
   procedure drawTransform (Self : in out sdlDrawer;   xf : in b2Transform);


   --    Draw a point.
   --    virtual void DrawPoint(const b2Vec2& p, float size, const b2Color& color) = 0;
   --

   overriding
   procedure drawPoint (Self : in out sdlDrawer;   p     : in b2Vec2;
                                                   Size  : in Real;
                                                   Color : in b2Color);







   function  Renderer return access SDL.Video.Renderers.Renderer;


   --  procedure drawCircle (renderer : in out SDL.Video.Renderers.Renderer;
   --                        centreX  : in     int;
   --                        centreY  : in     int;
   --                        radius   : in     int);


   --  procedure drawSolidCircle (renderer : in out SDL.Video.Renderers.Renderer;
   --                             centreX  : in     int;
   --                             centreY  : in     int;
   --                             radius   : in     int);


   --  procedure drawSegment (renderer : in out SDL.Video.Renderers.Renderer;
   --                         Line     : in     SDL.Video.Rectangles.line_Segment);



   --  procedure drawPolygon (Self : in out sdlDrawer;   Renderer : in out SDL.Video.Renderers.Renderer;
   --                         Vertices : in     b2Vec2s);


   --  procedure drawSolidPolygon (Renderer : in out SDL.Video.Renderers.Renderer;
   --                              Vertices : in     b2Vec2s);


private

   type sdlDrawer is new b2_Draw.b2Draw with
      record
         null;
      end record;

end testbed.SDL_Drawer;
