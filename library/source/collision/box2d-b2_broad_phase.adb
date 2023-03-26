with
     ada.unchecked_Deallocation,
     Interfaces;


package body box2d.b2_broad_Phase
is

   procedure free is new ada.unchecked_Deallocation (Naturals, Naturals_ptr);
   procedure free is new ada.unchecked_Deallocation (Integers, Integers_ptr);
   procedure free is new ada.unchecked_Deallocation (b2Pairs,   b2Pairs_ptr);



   --    b2BroadPhase();
   --
   --  b2BroadPhase::b2BroadPhase()
   --  {
   --    m_proxyCount = 0;
   --
   --    m_pairCapacity = 16;
   --    m_pairCount = 0;
   --    m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));
   --
   --    m_moveCapacity = 16;
   --    m_moveCount = 0;
   --    m_moveBuffer = (int32*)b2Alloc(m_moveCapacity * sizeof(int32));
   --  }
   --

   function to_b2BroadPhase return b2BroadPhase
   is
      Self : b2BroadPhase;
   begin
      Self.m_proxyCount   := 0;

      Self.m_pairCapacity := 16;
      Self.m_pairCount    := 0;
      Self.m_pairBuffer   := new b2Pairs (0 .. Self.m_pairCapacity - 1);

      Self.m_moveCapacity := 16;
      Self.m_moveCount    := 0;
      Self.m_moveBuffer   := new Integers (0 .. Self.m_moveCapacity - 1);

      Self.m_tree         := to_b2DynamicTree;

      return Self;
   end to_b2BroadPhase;




   --    ~b2BroadPhase();
   --
   --  b2BroadPhase::~b2BroadPhase()
   --  {
   --    b2Free(m_moveBuffer);
   --    b2Free(m_pairBuffer);
   --  }
   --

   procedure destruct (Self : in out b2BroadPhase)
   is
   begin
      free (Self.m_moveBuffer);
      free (Self.m_pairBuffer);
   end destruct;






   --    Create a proxy with an initial AABB. Pairs are not reported until
   --    UpdatePairs is called.
   --
   --    int32 CreateProxy(const b2AABB& aabb, void* userData);
   --
   --  int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
   --  {
   --    int32 proxyId = m_tree.CreateProxy(aabb, userData);
   --    ++m_proxyCount;
   --    BufferMove(proxyId);
   --    return proxyId;
   --  }
   --

   function createProxy (Self : in out b2BroadPhase;   aabb     : in b2AABB;
                                                       userData : in system.Address) return Natural
   is
      proxyId : constant Natural := Self.m_tree.createProxy (aabb, userData);
   begin
      Self.m_proxyCount := Self.m_proxyCount + 1;
      Self.bufferMove (proxyId);

      return proxyId;
   end createProxy;






   --    Destroy a proxy. It is up to the client to remove any pairs.
   --
   --    void DestroyProxy(int32 proxyId);
   --
   --  void b2BroadPhase::DestroyProxy(int32 proxyId)
   --  {
   --    UnBufferMove(proxyId);
   --    --m_proxyCount;
   --    m_tree.DestroyProxy(proxyId);
   --  }
   --

   procedure destroyProxy (Self : in out b2BroadPhase;   proxyId : in Natural)
   is
   begin
        Self.unBufferMove (proxyId);
        Self.m_proxyCount := Self.m_proxyCount - 1;
        Self.m_tree.destroyProxy (proxyId);
   end destroyProxy;





   --    Call MoveProxy as many times as you like, then when you are done
   --    call UpdatePairs to finalized the proxy pairs (for your time step).
   --
   --    void MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement);
   --
   --  void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
   --  {
   --    bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
   --    if (buffer)
   --    {
   --       BufferMove(proxyId);
   --    }
   --  }
   --

   procedure moveProxy (Self : in out b2BroadPhase;   proxyId      : in Natural;
                                                      aabb         : in b2AABB;
                                                      displacement : in b2Vec2)
   is
      buffer : constant Boolean := Self.m_tree.moveProxy (proxyId, aabb, displacement);
   begin
      if buffer
      then
         Self.bufferMove (proxyId);
      end if;
   end moveProxy;






   --    Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
   --
   --    void TouchProxy(int32 proxyId);
   --
   --  void b2BroadPhase::TouchProxy(int32 proxyId)
   --  {
   --    BufferMove(proxyId);
   --  }
   --

   procedure touchProxy (Self : in out b2BroadPhase;   proxyId : in Natural)
   is
   begin
       Self.bufferMove (proxyId);
   end touchProxy;






   --    Get the fat AABB for a proxy.
   --
   --    const b2AABB& GetFatAABB(int32 proxyId) const;
   --
   --  inline const b2AABB& b2BroadPhase::GetFatAABB(int32 proxyId) const
   --  {
   --    return m_tree.GetFatAABB(proxyId);
   --  }
   --

   function getFatAABB (Self : in out b2BroadPhase;   proxyId : in Natural) return b2AABB
   is
   begin
      return Self.m_tree.getFatAABB (proxyId);
   end getFatAABB;






   --    Get user data from a proxy. Returns nullptr if the id is invalid.
   --
   --    void* GetUserData(int32 proxyId) const;
   --
   --  inline void* b2BroadPhase::GetUserData(int32 proxyId) const
   --  {
   --    return m_tree.GetUserData(proxyId);
   --  }
   --

   function getUserData (Self : in b2BroadPhase;   proxyId : in Natural) return system.Address
   is
   begin
      return Self.m_tree.getUserData (proxyId);
   end getUserData;






   --    Test overlap of fat AABBs.
   --
   --    bool TestOverlap(int32 proxyIdA, int32 proxyIdB) const;
   --
   --  inline bool b2BroadPhase::TestOverlap(int32 proxyIdA, int32 proxyIdB) const
   --  {
   --    const b2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
   --    const b2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
   --    return b2TestOverlap(aabbA, aabbB);
   --  }
   --

   function testOverlap (Self : in out b2BroadPhase;   proxyIdA,
                                                       proxyIdB : in Natural) return Boolean
   is
      aabbA : constant b2AABB := Self.m_tree.getFatAABB (proxyIdA);
      aabbB : constant b2AABB := Self.m_tree.GetFatAABB (proxyIdB);
   begin
      return b2testOverlap (aabbA, aabbB);
   end testOverlap;





   --    Get the number of proxies.
   --
   --    int32 GetProxyCount() const;
   --
   --  inline int32 b2BroadPhase::GetProxyCount() const
   --  {
   --    return m_proxyCount;
   --  }
   --

   function getProxyCount (Self : in b2BroadPhase) return Natural
   is
   begin
      return Self.m_proxyCount;
   end getProxyCount;





   --    Update the pairs. This results in pair callbacks. This can only add pairs.
   --
   --    template <typename T>
   --    void UpdatePairs(T* callback);
   --
   --  template <typename T>
   --  void b2BroadPhase::UpdatePairs(T* callback)
   --  {
   --    // Reset pair buffer
   --    m_pairCount = 0;
   --
   --    // Perform tree queries for all moving proxies.
   --    for (int32 i = 0; i < m_moveCount; ++i)
   --    {
   --       m_queryProxyId = m_moveBuffer[i];
   --       if (m_queryProxyId == e_nullProxy)
   --       {
   --          continue;
   --       }
   --
   --       // We have to query the tree with the fat AABB so that
   --       // we don't fail to create a pair that may touch later.
   --       const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);
   --
   --       // Query tree, create pairs and add them pair buffer.
   --       m_tree.Query(this, fatAABB);
   --    }
   --
   --    // Send pairs to caller
   --    for (int32 i = 0; i < m_pairCount; ++i)
   --    {
   --       b2Pair* primaryPair = m_pairBuffer + i;
   --       void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
   --       void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);
   --
   --       callback->AddPair(userDataA, userDataB);
   --    }
   --
   --    // Clear move flags
   --    for (int32 i = 0; i < m_moveCount; ++i)
   --    {
   --       int32 proxyId = m_moveBuffer[i];
   --       if (proxyId == e_nullProxy)
   --       {
   --          continue;
   --       }
   --
   --       m_tree.ClearMoved(proxyId);
   --    }
   --
   --    // Reset move buffer
   --    m_moveCount = 0;
   --  }
   --

   procedure updatePairs (Self : in out b2BroadPhase;    callback : access callback_t)
   is
   begin
      -- Reset pair buffer.
      --
      Self.m_pairCount := 0;

      -- Perform tree queries for all moving proxies.
      --
      for i in 0 .. Self.m_moveCount - 1
      loop
         Self.m_queryProxyId := Self.m_moveBuffer (i);

         if Self.m_queryProxyId /= e_nullProxy
         then
            -- We have to query the tree with the fat AABB so that
            -- we don't fail to create a pair that may touch later.
            --
            declare
               --  function queryCallback (Callback : in out Callback_t;
               --                          nodeId   : in     Natural) return Boolean
               --  is
               --  begin
               --     return True;
               --  end queryCallback;

               function queryCallback (Callback : access b2BroadPhase;
                                       nodeId   : in     Natural) return Boolean
               is
               begin
                  return Self.queryCallback (nodeId);
               end queryCallback;


               --  procedure my_Query is new b2_dynamic_Tree.query (Callback_t    =>callback_t,
               --                                                   queryCallback => queryCallback);

               procedure my_Query is new b2_dynamic_Tree.query (Callback_t    => b2BroadPhase,
                                                                queryCallback => queryCallback);

               fatAABB : constant b2AABB := Self.m_tree.getFatAABB (Self.m_queryProxyId);
            begin
               -- Query tree, create pairs and add them to the pair buffer.
               --
               --  Self.m_tree.query (callback'Access, fatAABB);
               --  Self.my_Query (callback'Access, fatAABB);
               --  my_Query (Self.m_tree, callback, fatAABB);
               my_Query (Self.m_tree, Self, fatAABB);
            end;
         end if;
      end loop;


      -- Send pairs to caller.
      --
      for i in 0 .. Self.m_pairCount - 1
      loop
         declare
            primaryPair : constant access b2Pair  := Self.m_pairBuffer (i)'Access;
            userDataA   : constant system.Address := Self.m_tree.GetUserData (primaryPair.proxyIdA);
            userDataB   : constant system.Address := Self.m_tree.GetUserData (primaryPair.proxyIdB);
         begin
            addPair (callback, userDataA,
                               userDataB);
         end;
      end loop;


      -- Clear move flags.
      --
      for i in 0 .. Self.m_moveCount - 1
      loop
         declare
            proxyId : constant Natural := Self.m_moveBuffer (i);
         begin
            if proxyId /= e_nullProxy
            then
               Self.m_tree.clearMoved (proxyId);
            end if;
         end;
      end loop;

      -- Reset move buffer.
      --
      Self.m_moveCount := 0;
   end updatePairs;






   --  Query an AABB for overlapping proxies. The callback class
   --  is called for each proxy that overlaps the supplied AABB.
   --
   --  template <typename T>
   --  void Query (T* callback, const b2AABB& aabb) const;
   --
   --  template <typename T>
   --  inline void b2BroadPhase::Query(T* callback, const b2AABB& aabb) const
   --  {
   --    m_tree.Query(callback, aabb);
   --  }
   --

   procedure query (Self : in b2BroadPhase;    callback : access callback_t;
                                                            aabb     : in     b2AABB)
   is
      --  function queryCallback (Callback : access Callback_t;
      --                          nodeId   : in     Natural) return Boolean
      --  is
      --  begin
      --     return Self.queryCallback (nodeId);
      --  end queryCallback;


      procedure my_Query is new b2_dynamic_Tree.query (callback_t, queryCallback);

   begin
      my_query (Self.m_tree, callback.all, aabb);
   end query;







   --    Ray-cast against the proxies in the tree. This relies on the callback
   --    to perform an exact ray-cast in the case were the proxy contains a shape.
   --    The callback also performs the any collision filtering. This has performance
   --    roughly equal to k * log(n), where k is the number of collisions and n is the
   --    number of proxies in the tree.

   --    @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
   --    @param callback a callback class that is called for each proxy that is hit by the ray.
   --
   --    template <typename T>
   --    void RayCast(T* callback, const b2RayCastInput& input) const;
   --
   --  template <typename T>
   --  inline void b2BroadPhase::RayCast(T* callback, const b2RayCastInput& input) const
   --  {
   --    m_tree.RayCast(callback, input);
   --  }
   --

   procedure raycast (Self : in b2BroadPhase;    callback : in out callback_t;
                                                 input    : in     b2RayCastInput)
   is
      --  function raycastCallback (Callback : in out Callback_t;
      --                            subInput : in     b2RayCastInput;
      --                            nodeId   : in     Natural) return Real
      --  is
      --  begin
      --     --  return Self.raycast (Callback, subInput);
      --     return raycast (Self.m_tree, Callback, subInput);
      --  end raycastCallback;

      procedure my_raycast is new b2_dynamic_Tree.raycast (callback_t, raycastCallback);
   begin
      my_raycast (Self.m_tree, callback, input);
      --  raycast (Self.m_tree, callback, input);
   end raycast;






   --    Get the height of the embedded tree.
   --
   --    int32 GetTreeHeight() const;
   --
   --  inline int32 b2BroadPhase::GetTreeHeight() const
   --  {
   --    return m_tree.GetHeight();
   --  }
   --

   function getTreeHeight (Self : in b2BroadPhase) return Natural
   is
   begin
      return Self.m_tree.getHeight;
   end getTreeHeight;






   --    Get the balance of the embedded tree.
   --
   --    int32 GetTreeBalance() const;
   --
   --  inline int32 b2BroadPhase::GetTreeBalance() const
   --  {
   --    return m_tree.GetMaxBalance();
   --  }
   --

   function getTreeBalance (Self : in b2BroadPhase) return Natural
   is
   begin
      return Self.m_tree.getMaxBalance;
   end getTreeBalance;






   --    Get the quality metric of the embedded tree.
   --
   --    float GetTreeQuality() const;
   --
   --  inline float b2BroadPhase::GetTreeQuality() const
   --  {
   --    return m_tree.GetAreaRatio();
   --  }
   --

   function getTreeQuality (Self : in b2BroadPhase) return Real
   is
   begin
      return Self.m_tree.getAreaRatio;
   end getTreeQuality;





   --    Shift the world origin. Useful for large worlds.
   --    The shift formula is: position -= newOrigin
   --    @param newOrigin the new origin with respect to the old origin
   --
   --    void ShiftOrigin(const b2Vec2& newOrigin);
   --
   --  inline void b2BroadPhase::ShiftOrigin(const b2Vec2& newOrigin)
   --  {
   --    m_tree.ShiftOrigin(newOrigin);
   --  }
   --

   procedure shiftOrigin (Self : in out b2BroadPhase;    newOrigin : in b2Vec2)
   is
   begin
      Self.m_tree.shiftOrigin (newOrigin);
   end shiftOrigin;





   --    void BufferMove   (int32 proxyId);
   --
   --  void b2BroadPhase::BufferMove(int32 proxyId)
   --  {
   --    if (m_moveCount == m_moveCapacity)
   --    {
   --       int32* oldBuffer = m_moveBuffer;
   --       m_moveCapacity *= 2;
   --       m_moveBuffer = (int32*)b2Alloc(m_moveCapacity * sizeof(int32));
   --       memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(int32));
   --       b2Free(oldBuffer);
   --    }
   --
   --    m_moveBuffer[m_moveCount] = proxyId;
   --    ++m_moveCount;
   --  }
   --

   procedure bufferMove (Self : in out b2BroadPhase;   proxyId : in Natural)
   is
   begin
     if Self.m_moveCount = Self.m_moveCapacity
      then
         declare
            oldBuffer : Integers_ptr := Self.m_moveBuffer;
         begin
            Self.m_moveCapacity := Self.m_moveCapacity * 2;
            Self.m_moveBuffer   := new Integers (0 .. Self.m_moveCapacity - 1);

            Self.m_moveBuffer (0 .. Self.m_moveCount - 1) := oldBuffer.all;
            free (oldBuffer);
         end;
     end if;

     Self.m_moveBuffer (Self.m_moveCount) := proxyId;
     Self.m_moveCount                     := Self.m_moveCount + 1;
   end bufferMove;






   --    void UnBufferMove (int32 proxyId);
   --
   --  void b2BroadPhase::UnBufferMove(int32 proxyId)
   --  {
   --    for (int32 i = 0; i < m_moveCount; ++i)
   --    {
   --       if (m_moveBuffer[i] == proxyId)
   --       {
   --          m_moveBuffer[i] = e_nullProxy;
   --       }
   --    }
   --  }
   --

   procedure unBufferMove (Self : in out b2BroadPhase;   proxyId : in Natural)
   is
   begin
      for i in 0 .. Self.m_moveCount - 1
      loop
         if Self.m_moveBuffer (i) = proxyId
         then
            Self.m_moveBuffer (i) := e_nullProxy;
         end if;
      end loop;
   end unBufferMove;






   --    bool QueryCallback(int32 proxyId);
   --
   --  // This is called from b2DynamicTree::Query when we are gathering pairs.
   --
   --  bool b2BroadPhase::QueryCallback(int32 proxyId)
   --  {
   --    // A proxy cannot form a pair with itself.
   --    if (proxyId == m_queryProxyId)
   --    {
   --       return true;
   --    }
   --
   --    const bool moved = m_tree.WasMoved(proxyId);
   --    if (moved && proxyId > m_queryProxyId)
   --    {
   --       // Both proxies are moving. Avoid duplicate pairs.
   --       return true;
   --    }
   --
   --    // Grow the pair buffer as needed.
   --    if (m_pairCount == m_pairCapacity)
   --    {
   --       b2Pair* oldBuffer = m_pairBuffer;
   --       m_pairCapacity = m_pairCapacity + (m_pairCapacity >> 1);
   --       m_pairBuffer = (b2Pair*)b2Alloc(m_pairCapacity * sizeof(b2Pair));
   --       memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(b2Pair));
   --       b2Free(oldBuffer);
   --    }
   --
   --    m_pairBuffer[m_pairCount].proxyIdA = b2Min(proxyId, m_queryProxyId);
   --    m_pairBuffer[m_pairCount].proxyIdB = b2Max(proxyId, m_queryProxyId);
   --    ++m_pairCount;
   --
   --    return true;
   --  }

   function queryCallback (Self : in out b2BroadPhase;   proxyId : in Natural) return Boolean
   is
   begin
     -- A proxy cannot form a pair with itself.
     if proxyId = Self.m_queryProxyId
     then
        return True;
     end if;


      declare
         moved : constant Boolean := Self.m_tree.wasMoved (proxyId);
      begin
         if moved and proxyId > Self.m_queryProxyId
         then
            -- Both proxies are moving. Avoid duplicate pairs.
            --
            return True;
         end if;
      end;


      -- Grow the pair buffer as needed.
      --
      if Self.m_pairCount = Self.m_pairCapacity
      then
         declare
            use Interfaces;

            oldBuffer : b2Pairs_ptr := Self.m_pairBuffer;
         begin
            Self.m_pairCapacity :=   Self.m_pairCapacity
                                   + Integer (shift_Right (Unsigned_64 (Self.m_pairCapacity),
                                                           1));     -- TODO: Divide by 2, instead ?

            Self.m_pairBuffer                             := new b2Pairs (0 .. Self.m_pairCapacity - 1);
            Self.m_pairBuffer (0 .. Self.m_pairCount - 1) := oldBuffer.all;

            free (oldBuffer);
         end;
      end if;

      Self.m_pairBuffer (Self.m_pairCount).proxyIdA := Integer'min (proxyId, Self.m_queryProxyId);
      Self.m_pairBuffer (Self.m_pairCount).proxyIdB := Integer'max (proxyId, Self.m_queryProxyId);

      Self.m_pairCount := Self.m_pairCount + 1;

     return true;
   end queryCallback;



end box2d.b2_broad_Phase;
