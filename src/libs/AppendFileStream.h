#ifndef _APPENDFILESTREAM_H_
#define _APPENDFILESTREAM_H_

#include "StreamOutput.h"
#include "stdlib.h"
#include "string.h"

class AppendFileStream : public StreamOutput {
 public:
  AppendFileStream(const char* filename) { fn = strdup(filename); }
  virtual ~AppendFileStream() { free(fn); }
  int puts(const char*, int size = 0);

 private:
  char* fn;
};

#endif
