with
     b2_block_Allocator,
     interfaces.C,
     ada.unchecked_Conversion,
     system.Address_to_Access_conversions;


package body b2_block_Allocator
is

   use type int32;


   --  static const int32 b2_chunkSize = 16 * 1024;
   --  static const int32 b2_maxBlockSize = 640;
   --  static const int32 b2_chunkArrayIncrement = 128;

   b2_chunkSize           : constant int32 := 16 * 1024;
   b2_maxBlockSize        : constant int32 := 640;
   b2_chunkArrayIncrement : constant int32 := 128;



   --  These are the supported object sizes. Actual allocations are rounded up the next size.
   --

   --  static const int32 b2_blockSizes[b2_blockSizeCount] =
   --  {
   --    16,      // 0
   --    32,      // 1
   --    64,      // 2
   --    96,      // 3
   --    128,  // 4
   --    160,  // 5
   --    192,  // 6
   --    224,  // 7
   --    256,  // 8
   --    320,  // 9
   --    384,  // 10
   --    448,  // 11
   --    512,  // 12
   --    640,  // 13
   --  };

   b2_blockSizes : constant array (int32 range 0 .. 13) of int32 := ( 0 =>  16,
                                                                      1 =>  32,
                                                                      2 =>  64,
                                                                      3 =>  96,
                                                                      4 => 128,
                                                                      5 => 160,
                                                                      6 => 192,
                                                                      7 => 224,
                                                                      8 => 256,
                                                                      9 => 320,
                                                                     10 => 384,
                                                                     11 => 448,
                                                                     12 => 512,
                                                                     13 => 640);


   ------------
   -- b2SizeMap
   --

   --  This maps an arbitrary allocation size to a suitable slot in b2_blockSizes.

   --  struct b2SizeMap
   --  {
   --    uint8   values [b2_maxBlockSize + 1];
   --  };

   type Values_type is array (int32 range 0 .. b2_maxBlockSize) of uint8;

   type b2SizeMap is
      record
         Values : Values_type;
      end record;



   --    b2SizeMap()
   --    {
   --       int32 j = 0;
   --       values[0] = 0;
   --       for (int32 i = 1; i <= b2_maxBlockSize; ++i)
   --       {
   --          b2Assert(j < b2_blockSizeCount);
   --          if (i <= b2_blockSizes[j])
   --          {
   --             values[i] = (uint8)j;
   --          }
   --          else
   --          {
   --             ++j;
   --             values[i] = (uint8)j;
   --          }
   --       }
   --    }

   function to_b2SizeMap return b2SizeMap
   is
      Self : b2SizeMap;
      j    : int32    := 0;
   begin
      Self.Values (0) := 0;

      for i in 1 .. b2_maxBlockSize
      loop
         pragma assert (j < b2_blockSizeCount);

         if i <= b2_blockSizes (j)
         then
            Self.Values (i) := uint8 (j);
         else
            j               := j + 1;
            Self.Values (i) := uint8 (j);
         end if;
      end loop;

      return Self;
   end to_b2SizeMap;


   --  static const b2SizeMap b2_sizeMap;

   b2_sizeMap : constant b2SizeMap := to_b2SizeMap;




   --  struct b2Chunk
   --  {
   --    int32 blockSize;
   --    b2Block* blocks;
   --  };
   --
   -- Moved to spec.



   --  struct b2Block
   --  {
   --    b2Block* next;
   --  };
   --
   -- Moved to spec.



   package conversions is new system.Address_to_Access_conversions (uint8);



   procedure memset (Dest  : in void_ptr;
                     Ch    : in uint8;
                     Count : in Integer)
   is
      use conversions;

      Address : constant system.Address := to_Address (Object_pointer (Dest));

      type Bytes is array (1 .. Count) of uint8;

      Block : Bytes     with import;
      for Block'Address use Address;

      --  pragma import (C, Block);
   begin
      Block := (others => 0);
   end memset;



   procedure memcpy (Dest  : in void_ptr;
                     Src   : in void_ptr;
                     Count : in Integer)
   is
      use conversions;

      From_Addr : constant system.Address := to_Address (Object_pointer (Src));
      To_Addr   : constant system.Address := to_Address (Object_pointer (Dest));

      type Bytes is array (1 .. Count) of uint8;

      From : Bytes     with import;
      for From'Address use From_Addr;

      To   : Bytes     with import;
      for To'Address   use To_Addr;

      --  pragma import (C, Block);
   begin
      To := From;
   end memcpy;




   --------------
   -- Conversions
   --

   function to_b2Block_ptr is new ada.unchecked_Conversion (void_ptr, b2Block_ptr);
   function to_b2Chunk_ptr is new ada.unchecked_Conversion (void_ptr, b2Chunk_ptr);

   function to_b2void_ptr  is new ada.unchecked_Conversion (b2Chunk_ptr, void_ptr);
   function to_b2void_ptr  is new ada.unchecked_Conversion (b2Block_ptr, void_ptr);


   type int8s is array (Natural range <>) of aliased int8;

   package int8_Pointers is new interfaces.C.Pointers (Index              => Natural,
                                                       Element            => int8,
                                                       Element_array      => int8s,
                                                       default_Terminator => int8'First);
   subtype int8_ptr is int8_Pointers.Pointer;

   function to_int8_ptr    is new ada.unchecked_Conversion (b2Block_ptr, int8_ptr);
   function to_b2Block_ptr is new ada.unchecked_Conversion (int8_ptr,    b2Block_ptr);

   function to_int8_ptr    is new ada.unchecked_Conversion (void_ptr, int8_ptr);





   --  b2BlockAllocator::b2BlockAllocator()
   --  {
   --    b2Assert(b2_blockSizeCount < UCHAR_MAX);
   --
   --    m_chunkSpace = b2_chunkArrayIncrement;
   --    m_chunkCount = 0;
   --    m_chunks = (b2Chunk*)b2Alloc(m_chunkSpace * sizeof(b2Chunk));
   --
   --    memset(m_chunks, 0, m_chunkSpace * sizeof(b2Chunk));
   --    memset(m_freeLists, 0, sizeof(m_freeLists));
   --  }

   function to_b2BlockAllocator return b2BlockAllocator
   is
      pragma assert (b2_blockSizeCount < interfaces.C.UCHAR_MAX);

      Chunks : void_ptr;
      Self   : b2BlockAllocator;
   begin
      Self.m_chunkSpace := b2_chunkArrayIncrement;
      Self.m_chunkCount := 0;

      Chunks        := b2Alloc (Self.m_chunkSpace * (b2Chunk'Size / 8));
      Self.m_chunks := to_b2Chunk_ptr (Chunks);

      memset (Chunks,
              0,
              Integer (Self.m_chunkSpace * b2Chunk'Size / 8));

      --  memset (Self.m_freeLists, 0, sizeof (Self.m_freeLists));              -- The 'Self.m_freeLists' array of access type is automatically set to null by Ada.

      return Self;
   end to_b2BlockAllocator;





   --  b2BlockAllocator::~b2BlockAllocator()
   --  {
   --    for (int32 i = 0; i < m_chunkCount; ++i)
   --    {
   --       b2Free(m_chunks[i].blocks);
   --    }
   --
   --    b2Free(m_chunks);
   --  }

   procedure destruct (Self : in out b2BlockAllocator)
   is
      use b2Block_Pointers,
          b2Chunk_Pointers,
          interfaces.C;

      Chunk : b2Chunk_ptr;
      Block : b2Block_ptr;
      Void  : void_ptr;
   begin
      for i in 0 .. Self.m_chunkCount - 1
      loop
         Chunk := Self.m_chunks + ptrdiff_t (i);
         Block := Chunk.blocks;
         Void  := to_b2void_ptr (Block);

         b2free (Void);
      end loop;

      Void := to_b2void_ptr (Self.m_chunks);
      b2Free (Void);
   end destruct;





   --  void* b2BlockAllocator::Allocate (int32 size)
   --  {
   --    if (size == 0)
   --    {
   --       return nullptr;
   --    }
   --
   --    b2Assert(0 < size);
   --
   --    if (size > b2_maxBlockSize)
   --    {
   --       return b2Alloc(size);
   --    }
   --
   --    int32 index = b2_sizeMap.values[size];
   --    b2Assert(0 <= index && index < b2_blockSizeCount);
   --
   --    if (m_freeLists[index])
   --    {
   --       b2Block* block = m_freeLists[index];
   --       m_freeLists[index] = block->next;
   --       return block;
   --    }
   --    else
   --    {
   --       if (m_chunkCount == m_chunkSpace)
   --       {
   --          b2Chunk* oldChunks = m_chunks;
   --          m_chunkSpace += b2_chunkArrayIncrement;
   --          m_chunks = (b2Chunk*)b2Alloc(m_chunkSpace * sizeof(b2Chunk));
   --          memcpy(m_chunks, oldChunks, m_chunkCount * sizeof(b2Chunk));
   --          memset(m_chunks + m_chunkCount, 0, b2_chunkArrayIncrement * sizeof(b2Chunk));
   --          b2Free(oldChunks);
   --       }
   --
   --       b2Chunk* chunk = m_chunks + m_chunkCount;
   --       chunk->blocks = (b2Block*)b2Alloc(b2_chunkSize);
   --  #if defined(_DEBUG)
   --       memset(chunk->blocks, 0xcd, b2_chunkSize);
   --  #endif
   --       int32 blockSize = b2_blockSizes[index];
   --       chunk->blockSize = blockSize;
   --       int32 blockCount = b2_chunkSize / blockSize;
   --       b2Assert(blockCount * blockSize <= b2_chunkSize);
   --       for (int32 i = 0; i < blockCount - 1; ++i)
   --       {
   --          b2Block* block = (b2Block*)((int8*)chunk->blocks + blockSize * i);
   --          b2Block* next = (b2Block*)((int8*)chunk->blocks + blockSize * (i + 1));
   --          block->next = next;
   --       }
   --       b2Block* last = (b2Block*)((int8*)chunk->blocks + blockSize * (blockCount - 1));
   --       last->next = nullptr;
   --
   --       m_freeLists[index] = chunk->blocks->next;
   --       ++m_chunkCount;
   --
   --       return chunk->blocks;
   --    }
   --  }


   function allocate (Self : in out b2BlockAllocator;   Size : in int32) return void_ptr
   is
   begin
      if Size = 0
      then
         return null;
      end if;

      pragma assert (0 < Size);

      if Size > b2_maxBlockSize
      then
         return b2Alloc (Size);
      end if;


      declare
         use b2Chunk_Pointers,
             Interfaces.C;

         use type b2Block_Pointers.Pointer;

         Index : constant int32 := int32 (b2_sizeMap.values (Size));

         pragma assert (    0     <= Index
                        and Index < b2_blockSizeCount);
      begin
         if Self.m_freeLists (Integer (Index)) /= null
         then
            declare
               Block : constant b2Block_ptr := Self.m_freeLists (Integer (Index));
            begin
               Self.m_freeLists (Integer (Index)) := Block.next;
               return to_b2void_ptr (Block);
            end;

         else
            if Self.m_chunkCount = Self.m_chunkSpace
            then
               declare
                  Void      : void_ptr;
                  oldChunks : constant b2Chunk_ptr := Self.m_chunks;
               begin
                  Self.m_chunkSpace := Self.m_chunkSpace + b2_chunkArrayIncrement;

                  Void          := b2Alloc (Self.m_chunkSpace * (b2Chunk'Size / 8));
                  Self.m_Chunks := to_b2Chunk_ptr (Void);

                  memcpy (to_b2void_ptr (Self.m_chunks),
                          to_b2void_ptr (oldChunks),
                          Integer       (Self.m_chunkCount * (b2Chunk'Size / 8)));

                  declare
                     Chunk : constant b2Chunk_ptr := Self.m_Chunks + ptrdiff_t (Self.m_chunkCount);
                  begin
                     memset (to_b2void_ptr (Chunk),
                             0,
                             Integer (b2_chunkArrayIncrement * (b2Chunk'Size / 8)));
                  end;

                  Void := to_b2void_ptr (oldChunks);
                  b2free (Void);
               end;
            end if;

            declare
               use b2Block_Pointers,
                   int8_Pointers;

               Chunk      : constant b2Chunk_ptr := Self.m_chunks + ptrdiff_t (Self.m_chunkCount);
               blockSize  : constant int32       := b2_blockSizes (Index);
               blockCount : constant int32       := b2_chunkSize / blockSize;
               Last       :          b2Block_ptr;
            begin
               Chunk.blocks := to_b2Block_ptr (b2Alloc (b2_chunkSize));

               if DEBUG
               then
                  memset (to_b2void_ptr (Chunk.Blocks),
                          16#cd#,
                          Integer (b2_chunkSize));
               end if;

               Chunk.blockSize := blockSize;
               pragma assert (blockCount * blockSize <= b2_chunkSize);

               for i in 0 .. blockCount - 2
               loop
                  declare
                     Block : constant b2Block_ptr := to_b2Block_ptr (to_int8_ptr (Chunk.Blocks) + ptrdiff_t (blockSize * i));
                     Next  : constant b2Block_ptr := to_b2Block_ptr (to_int8_ptr (Chunk.Blocks) + ptrdiff_t (blockSize * (i + 1)));
                  begin
                     Block.Next := Next;
                  end;
               end loop;

               Last      := to_b2Block_ptr (to_int8_ptr (Chunk.Blocks) + ptrdiff_t (blockSize * (blockCount - 1)));
               Last.Next := null;

               Self.m_freeLists (Integer (Index)) := Chunk.Blocks.Next;
               Self.m_chunkCount                  := Self.m_chunkCount + 1;

               return to_b2void_ptr (Chunk.Blocks);
            end;
         end if;
      end;

   end allocate;





   --  void b2BlockAllocator::Free (void* p, int32 size)
   --  {
   --    if (size == 0)
   --    {
   --       return;
   --    }
   --
   --    b2Assert(0 < size);
   --
   --    if (size > b2_maxBlockSize)
   --    {
   --       b2Free(p);
   --       return;
   --    }
   --
   --    int32 index = b2_sizeMap.values[size];
   --    b2Assert(0 <= index && index < b2_blockSizeCount);
   --
   --  #if defined(_DEBUG)
   --    // Verify the memory address and size is valid.
   --    int32 blockSize = b2_blockSizes[index];
   --    bool found = false;
   --    for (int32 i = 0; i < m_chunkCount; ++i)
   --    {
   --       b2Chunk* chunk = m_chunks + i;
   --       if (chunk->blockSize != blockSize)
   --       {
   --          b2Assert((int8*) p             + blockSize    <= (int8*) chunk->blocks ||
   --                   (int8*) chunk->blocks + b2_chunkSize <= (int8*) p);
   --       }
   --       else
   --       {
   --          if ((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + b2_chunkSize)
   --          {
   --             found = true;
   --          }
   --       }
   --    }
   --
   --    b2Assert(found);
   --
   --    memset(p, 0xfd, blockSize);
   --  #endif
   --
   --    b2Block* block = (b2Block*)p;
   --    block->next = m_freeLists[index];
   --    m_freeLists[index] = block;
   --  }

   procedure free (Self : in out b2BlockAllocator;    p    : in void_ptr;
                                                      Size : in int32)
   is
      Void : void_ptr;
   begin
      if Size = 0
      then
         return;
      end if;

      pragma assert (0 < Size);

      if size > b2_maxBlockSize
      then
         Void := p;
         b2free (Void);
         return;
      end if;


      declare
         Index : constant int32 := int32 (b2_sizeMap.Values (Size));

         pragma assert (    0     <= Index
                        and Index <  b2_blockSizeCount);
      begin
         if DEBUG
         then     -- Verify the memory address and size is valid.
            declare
               use b2Chunk_pointers,
                   int8_pointers,
                   interfaces.C;

               blockSize : constant int32      := b2_blockSizes (Index);
               Found     :          Boolean    := False;
               Chunk     :          b2Chunk_ptr;
            begin
               for i in 0 .. Self.m_chunkCount - 1
               loop
                  Chunk := Self.m_Chunks + ptrdiff_t (i);

                  declare
                     function to_uint64 is new ada.unchecked_Conversion (int8_ptr, interfaces.Unsigned_64);

                     use type interfaces.Unsigned_64;

                     l1 : int8_ptr;     -- 1st left  condition.
                     r1 : int8_ptr;     -- 1st right condition.
                     l2 : int8_ptr;     -- 2nd left  condition.
                     r2 : int8_ptr;     -- 2nd right condition.
                  begin
                     if Chunk.blockSize /= blockSize
                     then
                        l1 := to_int8_ptr (p)            + ptrdiff_t (blockSize);
                        r1 := to_int8_ptr (Chunk.Blocks);
                        l2 := to_int8_ptr (Chunk.Blocks) + ptrdiff_t (b2_chunkSize);
                        r2 := to_int8_ptr (p);

                        pragma assert (   to_uint64 (l1) <= to_uint64 (r1)
                                       or to_uint64 (l2) <= to_uint64 (r2));
                     else
                        l1 := to_int8_ptr (Chunk.Blocks);
                        r1 := to_int8_ptr (p);
                        l2 := to_int8_ptr (p)            + ptrdiff_t (blockSize);
                        r2 := to_int8_ptr (Chunk.Blocks) + ptrdiff_t (b2_chunkSize);

                        if    to_uint64 (l1) <= to_uint64 (r1)
                          and to_uint64 (l2) <= to_uint64 (r2)
                        then
                           Found := True;
                        end if;
                     end if;
                  end;
               end loop;

               pragma assert (Found);

               memset (p,  16#fd#,  Integer (blockSize));
            end;
         end if;

         declare
            Block : constant b2Block_ptr := to_b2Block_ptr (p);
         begin
            Block.Next                         := Self.m_freeLists (Integer (Index));
            Self.m_freeLists (Integer (Index)) := Block;
         end;
      end;
   end free;




   --  void b2BlockAllocator::Clear()
   --  {
   --    for (int32 i = 0; i < m_chunkCount; ++i)
   --    {
   --       b2Free(m_chunks[i].blocks);
   --    }
   --
   --    m_chunkCount = 0;
   --    memset(m_chunks, 0, m_chunkSpace * sizeof(b2Chunk));
   --    memset(m_freeLists, 0, sizeof(m_freeLists));
   --  }

   procedure clear (Self : in out b2BlockAllocator)
   is
      use b2Chunk_Pointers,
          interfaces.C;

      Chunk : b2Chunk_ptr;
      Void  : void_ptr;
   begin
      for i in 0 .. Self.m_chunkCount - 1
      loop
         Chunk := Self.m_chunks + ptrdiff_t (i);
         Void  := to_b2void_ptr (Chunk.Blocks);

         b2free (Void);
      end loop;

      Self.m_chunkCount := 0;

      memset (to_b2void_ptr (Self.m_chunks),
              0,
              Integer (Self.m_chunkSpace * (b2Chunk'Size / 8)));

      Self.m_freeLists := (others => null);
   end clear;


end b2_block_Allocator;
