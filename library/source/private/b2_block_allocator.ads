with
     b2_Settings,
     b2_Types;


private
with
     interfaces.C.Pointers;


package b2_block_Allocator
is
   use b2_Types,
       b2_Settings;


   --  const int32 b2_blockSizeCount = 14;
   --
   b2_blockSizeCount : constant := 14;



   --  struct b2Block;

   type b2Block is private;



   --  struct b2Chunk;

   type b2Chunk is private;




   -------------------
   -- b2BlockAllocator
   --

   --  This is a small object allocator used for allocating small
   --  objects that persist for more than one time step.
   --
   --  See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
   --

   type b2BlockAllocator is private;


   --    b2BlockAllocator();

   function to_b2BlockAllocator return b2BlockAllocator;



   --    ~b2BlockAllocator();

   procedure destruct (Self : in out b2BlockAllocator);



   --    Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.

   --    void* Allocate (int32 size);

   function allocate (Self : in out b2BlockAllocator;   Size : in int32) return void_ptr;



   --    Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.

   --    void Free (void* p, int32 size);

   procedure free (Self : in out b2BlockAllocator;   p    : in void_ptr;
                                                     Size : in int32);



   --    void Clear();

   procedure clear (Self : in out b2BlockAllocator);





   procedure memset (Dest  : in void_ptr;
                     Ch    : in uint8;
                     Count : in Integer);



private


   --  struct b2Block
   --  {
   --    b2Block* next;
   --  };

   type b2Block is
      record
         --  Next : b2Block_ptr;
         Next : access b2Block;
      end record;

   type b2Blocks is array (Natural range <>) of aliased b2Block;

   null_b2Block : constant b2Block := (others => <>);

   package b2Block_Pointers is new interfaces.C.Pointers (Index              => Natural,
                                                          Element            => b2Block,
                                                          Element_array      => b2Blocks,
                                                          default_Terminator => null_b2Block);
   --  type b2Block_ptr  is access all b2Block;
   subtype b2Block_ptr  is b2Block_Pointers.Pointer;
   type    b2Block_ptrs is array (Natural range 0 .. b2_blockSizeCount - 1) of b2Block_ptr;







   --  struct b2Chunk
   --  {
   --    int32 blockSize;
   --    b2Block* blocks;
   --  };

   type b2Chunk is
      record
         blockSize : int32;
         Blocks    : b2Block_ptr;
      end record;

   type b2Chunks is array (Natural range <>) of aliased b2Chunk;

   null_b2Chunk : constant b2Chunk := (blockSize => 0,
                                       blocks    => null);

   package b2Chunk_Pointers is new interfaces.C.Pointers (Index              => Natural,
                                                          Element            => b2Chunk,
                                                          Element_array      => b2Chunks,
                                                          default_Terminator => null_b2Chunk);
      --  type b2Chunk_ptr is access all b2Chunk;
   subtype b2Chunk_ptr  is b2Chunk_Pointers.Pointer;




   --  class b2BlockAllocator
   --  {
   --  private:
   --
   --    b2Chunk* m_chunks;
   --    int32    m_chunkCount;
   --    int32    m_chunkSpace;
   --
   --    b2Block* m_freeLists [b2_blockSizeCount];
   --  };

   type b2BlockAllocator is
      record
           m_Chunks     : b2Chunk_ptr;
           m_chunkCount : int32;
           m_chunkSpace : int32;

           m_freeLists  : b2Block_ptrs;
      end record;


end b2_block_Allocator;
