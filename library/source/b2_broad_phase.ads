with
     b2_Collision,
     b2_dynamic_Tree,
     b2_Math,
     b2_Types,
     b2_Settings,
     System;


package b2_broad_Phase
is
   use b2_Collision,
       b2_dynamic_Tree,
       b2_Math,
       b2_Types,
       b2_Settings;



   --    enum
   --    {
   --       e_nullProxy = -1
   --    };
   --

   e_nullProxy : constant Integer; -- := -1;




   --  struct b2Pair
   --  {
   --    int32 proxyIdA;
   --    int32 proxyIdB;
   --  };
   --

   type b2Pair is
      record
         proxyIdA,
         proxyIdB : Integer;
      end record;

   type b2Pairs is array (Natural range <>) of aliased b2Pair;



   --  The broad-phase is used for computing pairs and performing volume queries and ray casts.
   --  This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
   --  It is up to the client to consume the new pairs and to track subsequent overlap.
   --
   --  class b2BroadPhase
   --  {
   --  public:
   --
   --
   --  private:
   --
   --    b2DynamicTree m_tree;
   --
   --    int32 m_proxyCount;
   --
   --    int32* m_moveBuffer;
   --    int32 m_moveCapacity;
   --    int32 m_moveCount;
   --
   --    b2Pair* m_pairBuffer;
   --    int32 m_pairCapacity;
   --    int32 m_pairCount;
   --
   --    int32 m_queryProxyId;
   --  };
   --

   type Naturals_ptr is access all Naturals;
   type Integers_ptr is access all Integers;
   type  b2Pairs_ptr is access all b2Pairs;

   type b2BroadPhase is tagged
      record
         m_tree         : b2DynamicTree;

         m_proxyCount   : Integer;

         m_moveBuffer   : Integers_ptr;
         m_moveCapacity : Integer;
         m_moveCount    : Integer;

         m_pairBuffer   : b2Pairs_ptr;
         m_pairCapacity : Integer;
         m_pairCount    : Integer;

         m_queryProxyId : Integer;
      end record;




   --    b2BroadPhase();
   --

   function to_b2BroadPhase return b2BroadPhase;




   --    ~b2BroadPhase();
   --

   procedure destruct (Self : in out b2BroadPhase);




   --    Create a proxy with an initial AABB. Pairs are not reported until
   --    UpdatePairs is called.
   --
   --    int32 CreateProxy(const b2AABB& aabb, void* userData);
   --

   function createProxy (Self : in out b2BroadPhase;   aabb     : in b2AABB;
                                                       userData : in system.Address) return Natural;




   --    Destroy a proxy. It is up to the client to remove any pairs.
   --
   --    void DestroyProxy(int32 proxyId);
   --

   procedure destroyProxy (Self : in out b2BroadPhase;   proxyId : in Natural);



   --    Call MoveProxy as many times as you like, then when you are done
   --    call UpdatePairs to finalized the proxy pairs (for your time step).
   --
   --    void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);
   --

   procedure moveProxy (Self : in out b2BroadPhase;   proxyId      : in Natural;
                                                      aabb         : in b2AABB;
                                                      displacement : in b2Vec2);




   --    Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
   --
   --    void TouchProxy(int32 proxyId);
   --

   procedure touchProxy (Self : in out b2BroadPhase;   proxyId : in Natural);





   --    Get the fat AABB for a proxy.
   --
   --    const b2AABB& GetFatAABB(int32 proxyId) const;
   --

   function getFatAABB (Self : in out b2BroadPhase;   proxyId : in Natural) return b2AABB;




   --    Get user data from a proxy. Returns nullptr if the id is invalid.
   --
   --    void* GetUserData(int32 proxyId) const;
   --

   function getUserData (Self : in out b2BroadPhase;   proxyId : in Natural) return system.Address;




   --    Test overlap of fat AABBs.
   --
   --    bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const;
   --

   function testOverlap (Self : in out b2BroadPhase;   proxyIdA,
                                                       proxyIdB : in Natural) return Boolean;




   --    Get the number of proxies.
   --
   --    int32 GetProxyCount() const;
   --

   function getProxyCount (Self : in b2BroadPhase) return Natural;





   --    Update the pairs. This results in pair callbacks. This can only add pairs.
   --
   --    template <typename T>
   --    void UpdatePairs(T* callback);
   --

   generic
      type callback_t is private;

      with procedure addPair (Callback : in out callback_t;   userDataA,
                                                              userDataB : in system.Address);

   procedure updatePairs (Self : in out b2BroadPhase;    callback : in out callback_t);




   --    Query an AABB for overlapping proxies. The callback class
   --    is called for each proxy that overlaps the supplied AABB.
   --
   --    template <typename T>
   --    void Query (T* callback, const b2AABB& aabb) const;
   --

   generic
      type callback_t is private;

   procedure query (Self : in out b2BroadPhase;   callback : in out callback_t;
                                                  aabb     : in     b2AABB);





   --    Ray-cast against the proxies in the tree. This relies on the callback
   --    to perform an exact ray-cast in the case were the proxy contains a shape.
   --    The callback also performs the any collision filtering. This has performance
   --    roughly equal to k * log(n), where k is the number of collisions and n is the
   --    number of proxies in the tree.
   --
   --    @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
   --    @param callback a callback class that is called for each proxy that is hit by the ray.
   --
   --    template <typename T>
   --    void RayCast(T* callback, const b2RayCastInput& input) const;
   --

   generic
      type callback_t is private;

      with function raycastCallback (Callback : in out Callback_t;
                                     subInput : in     b2RayCastInput;
                                     nodeId   : in     Natural) return Real;

   procedure raycast (Self : in b2BroadPhase;    callback : in out callback_t;
                                                 input    : in     b2RayCastInput);




   --    Get the height of the embedded tree.
   --
   --    int32 GetTreeHeight() const;
   --

   function getTreeHeight (Self : in b2BroadPhase) return Natural;





   --    Get the balance of the embedded tree.
   --
   --    int32 GetTreeBalance() const;
   --

   function getTreeBalance (Self : in b2BroadPhase) return Natural;





   --    Get the quality metric of the embedded tree.
   --
   --    float GetTreeQuality() const;
   --

   function getTreeQuality (Self : in b2BroadPhase) return Real;




   --    Shift the world origin. Useful for large worlds.
   --    The shift formula is: position -= newOrigin
   --    @param newOrigin the new origin with respect to the old origin
   --
   --    void ShiftOrigin(const b2Vec2& newOrigin);

   procedure shiftOrigin (Self : in out b2BroadPhase;    newOrigin : in b2Vec2);







private
   e_nullProxy : constant Integer := -1;


   --    friend class b2DynamicTree;


   --    void BufferMove   (int32 proxyId);
   --

   procedure bufferMove (Self : in out b2BroadPhase;   proxyId : in Natural);




   --    void UnBufferMove (int32 proxyId);
   --

   procedure unBufferMove (Self : in out b2BroadPhase;   proxyId : in Natural);




   --    bool QueryCallback(int32 proxyId);
   --

   function queryCallback (Self : in out b2BroadPhase;   proxyId : in Natural) return Boolean;




end b2_broad_Phase;
