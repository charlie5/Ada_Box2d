--  #define _CRT_SECURE_NO_WARNINGS
--
--  #include "box2d/b2_settings.h"
--  #include <stdio.h>
--  #include <stdarg.h>
--  #include <stdlib.h>

package body b2_Settings
is
   procedure dummy is null;


   --
--  b2Version b2_version = {2, 4, 0};
--
--  // Memory allocators. Modify these to use your own allocator.
--  void* b2Alloc_Default(int32 size)
--  {
--    return malloc(size);
--  }
--
--  void b2Free_Default(void* mem)
--  {
--    free(mem);
--  }
--
--  // You can modify this to use your logging facility.
--  void b2Log_Default(const char* string, va_list args)
--  {
--    vprintf(string, args);
--  }
--
--  FILE* b2_dumpFile = nullptr;
--
--  void b2OpenDump(const char* fileName)
--  {
--    b2Assert(b2_dumpFile == nullptr);
--    b2_dumpFile = fopen(fileName, "w");
--  }
--
--  void b2Dump(const char* string, ...)
--  {
--    if (b2_dumpFile == nullptr)
--    {
--       return;
--    }
--
--    va_list args;
--    va_start(args, string);
--    vfprintf(b2_dumpFile, string, args);
--    va_end(args);
--  }
--
--  void b2CloseDump()
--  {
--    fclose(b2_dumpFile);
--    b2_dumpFile = nullptr;
--  }
end b2_Settings;
