#include "dcodec.h"

boolean dCodec::diff(Sample const* prev, Sample const* next, Delta * delta)
{
  int32_t x = ((int32_t)next->x)-((int32_t)prev->x);
  int32_t y = ((int32_t)next->y)-((int32_t)prev->y);
  int32_t z = ((int32_t)next->z)-((int32_t)prev->z);

  if (x > MAX_INT8) return false;
  if (y > MAX_INT8) return false;
  if (z > MAX_INT8) return false;

  if (x <= MIN_INT8) return false;
  if (y <= MIN_INT8) return false;
  if (z <= MIN_INT8) return false;

  delta->x = (int8_t)x;
  delta->y = (int8_t)y;
  delta->z = (int8_t)z;

  return true;
}

boolean dCodec::sum(Sample const* prev, Sample * next, Delta const* delta)
{
  if (DATA_END == delta->x) return false;

  next->x = (uint16_t)(((int8_t)delta->x)+((int32_t)prev->x));
  next->y = (uint16_t)(((int8_t)delta->y)+((int32_t)prev->y));
  next->z = (uint16_t)(((int8_t)delta->z)+((int32_t)prev->z));

  return true;
}

boolean dCodec::encodeFile(SdFile & ifile, SdFile & ofile, uint32_t maxdata)
{
  uint32_t nread = ifile.read(samples, sizeof(Sample)*samples_size);

  uint32_t nwrite = 0;

  uint32_t nrest = nread % sizeof(Sample);
  nread /= sizeof(Sample);

  uint32_t sindex = 0; // samples index
  uint32_t dindex = 0; // deltas index

  if (nread > 0) {
    *((Sample *)&deltas[dindex++]) = samples[sindex++]; // first element
    dindex++; // sizeof(Sample) == 2*sizeof(Deltas)
  }

  uint32_t ndata = 1;
  while (nread > 1 and ndata < maxdata) {
    for (; sindex < nread and ndata < maxdata; sindex++, ndata++) {
      if (!diff(samples+(sindex-1), samples+sindex, deltas+(dindex++))) {
        deltas[dindex-1].x = deltas[dindex-1].y = deltas[dindex-1].z = DATA_END;

        nwrite = ofile.write(deltas, sizeof(Delta)*dindex);
        if (nwrite != sizeof(Delta)*dindex) {
          return false;
        }
        dindex = 0;

        *((Sample *)&deltas[dindex++]) = samples[sindex];
        dindex++;
      }

      if (deltas_size == dindex) {
        nwrite = ofile.write(deltas, sizeof(Delta)*deltas_size);
        if (sizeof(Delta)*deltas_size != nwrite) {
          return false;
        }
        dindex = 0;
      }
    }

    if (0 == nrest) {
      samples[0] = samples[sindex-1];
      nread = ifile.read(samples+1, sizeof(Sample)*(samples_size-1)) + 1*sizeof(Sample);
      nrest = nread % sizeof(Sample);
      nread /= sizeof(Sample);
    } else nread = 1;
    sindex = 1;
  }

  if (dindex > 0) {
    nwrite = ofile.write(deltas, sizeof(Delta)*dindex);
    if (sizeof(Delta)*dindex != nwrite) {
      return false;
    }
    dindex = 0;
  }

  return nrest == 0;
}

boolean dCodec::decodeFile(SdFile & ifile, SdFile & ofile)
{
  uint32_t nread = ifile.read(deltas, sizeof(Delta)*deltas_size);
  uint32_t nwrite = 0;

  uint32_t nrest = nread % sizeof(Delta);
  nread /= sizeof(Delta);

  uint32_t sindex = 0; // samples index
  uint32_t dindex = 0; // deltas index

  if (nread > 1) {
    samples[sindex++] = *((Sample *)&deltas[dindex++]);
    dindex++;
  }

  while (nread > 0) {
    for (; dindex < nread; dindex++) {
      if (!sum(samples+(sindex-1), samples+sindex, deltas+dindex)) { // deltas[dindex].x == DATA_END --> dindex+=2
        dindex++;

        Delta deltas2[2];
        uint8_t dindex2 = 0;
        if (dindex < nread) deltas2[dindex2++] = deltas[dindex++];
        if (dindex < nread) deltas2[dindex2++] = deltas[dindex++];
        if (dindex >= nread and dindex2 < 2) {
          if (0 == nrest) {
            nread = ifile.read(deltas, sizeof(Delta)*deltas_size);
            nrest = nread % sizeof(Delta);
            nread /= sizeof(Delta);
          } else nread = 0;
          dindex = 0;

          if (dindex < nread and dindex2 < 2)
            deltas2[dindex2++] = deltas[dindex++];
          if (dindex < nread and dindex2 < 2)
            deltas2[dindex2++] = deltas[dindex++];
        }

        if (2 == dindex2) {
          samples[sindex] = *((Sample *)&deltas2[0]);
          dindex--;
        } else {
          return false;
        }
      }
      sindex++;

      if (samples_size == sindex) {
        Sample prev = samples[sindex-1];
        nwrite = ofile.write(samples, sizeof(Sample)*(samples_size-1));
        sindex = 0;
        samples[sindex++] = prev;

        if (sizeof(Sample)*(samples_size-1) != nwrite) {
          return false;
        }
      }
    }

    if (0 == nrest) {
      nread = ifile.read(deltas, sizeof(Delta)*deltas_size);
      nrest = nread % sizeof(Delta);
      nread /= sizeof(Delta);
    } else nread = 0;
    dindex = 0;
  }

  if (sindex > 0) {
    nwrite = ofile.write(samples, sizeof(Sample)*sindex);
    if (sizeof(Sample)*sindex != nwrite) {
      return false;
    }
    sindex = 0;
  }

  return 0 == nrest;
}
