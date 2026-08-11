#ifndef _FILESTREAM_H_
#define _FILESTREAM_H_

#include <stdio.h>
#include <stdlib.h>

#include "StreamOutput.h"
#include "libs/FirmwareFileSystem.h"

class FileStream : public StreamOutput {
 public:
  FileStream(const char* filename) { fd = fwfs::fopen(filename, "w"); }
  virtual ~FileStream() { close(); }
  int puts(const char* str, int size = 0) { return (fd == NULL) ? 0 : fwfs::fwrite(str, 1, strlen(str), fd); }
  void close() {
    if (fd != NULL) fwfs::fclose(fd);
    fd = NULL;
  }
  bool is_open() { return fd != NULL; }

 private:
  FILE* fd;
};

#endif
