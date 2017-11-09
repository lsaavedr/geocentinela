#ifndef DCODEC_H
#define DCODEC_H

#if defined(ARDUINO) and ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <SdFat.h>

#define MAX_INT8 127
#define MIN_INT8 -128
#define DATA_END MIN_INT8

typedef struct Samples
{
  uint16_t x, y, z;
} Sample;

typedef struct Deltas
{
  int8_t x, y, z;
} Delta;

class dCodec {
public:
  dCodec(Sample * samples, uint32_t samples_size, Delta * deltas, uint32_t deltas_size):
  samples(samples), samples_size(samples_size), deltas(deltas), deltas_size(deltas_size) {}

  Sample * samples;
  uint32_t samples_size;

  Delta * deltas;
  uint32_t deltas_size;

  boolean encodeFile(SdFile & ifile, SdFile & ofile, uint32_t maxdata);
  boolean decodeFile(SdFile & ifile, SdFile & ofile);

public:
  boolean diff(Sample const* prev, Sample const* next, Delta * delta);
  boolean sum(Sample const* prev, Sample * next, Delta const* delta);
};

#endif // DECODEC_H
