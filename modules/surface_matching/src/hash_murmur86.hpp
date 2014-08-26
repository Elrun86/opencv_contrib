
// Taken from http://code.google.com/p/smhasher/
// License: MIT

//-----------------------------------------------------------------------------
// MurmurHash3 was written by Austin Appleby, and is placed in the public
// domain. The author hereby disclaims copyright to this source code.

// Note - The x86 and x64 versions do _not_ produce the same results, as the
// algorithms are optimized for their respective platforms. You can still
// compile and run any of them on any platform, but your performance with the
// non-native version will be less than optimal.

//-----------------------------------------------------------------------------
// Platform-specific functions and macros


#ifndef __OPENCV_HASH_MURMUR86_HPP_
#define __OPENCV_HASH_MURMUR86_HPP_

//-----------------------------------------------------------------------------
// Block read - if your platform needs to do endian-swapping or can only
// handle aligned reads, do the conversion here

FORCE_INLINE unsigned int getblock32 ( const unsigned int * p, int i )
{
  return p[i];
}

//-----------------------------------------------------------------------------
// Finalization mix - force all bits of a hash block to avalanche

FORCE_INLINE unsigned int fmix32 ( unsigned int h )
{
  h ^= h >> 16;
  h *= 0x85ebca6b;
  h ^= h >> 13;
  h *= 0xc2b2ae35;
  h ^= h >> 16;

  return h;
}

//----------

FORCE_INLINE void hashMurmur ( const void * key, int len, unsigned int seed, void * out )
{
  const unsigned char * data = (const unsigned char*)key;
  const int nblocks = len / 4;

  unsigned int h1 = seed;

  const unsigned int c1 = 0xcc9e2d51;
  const unsigned int c2 = 0x1b873593;

  //----------
  // body

  const unsigned int * blocks = (const unsigned int *)(data + nblocks*4);

  for(int i = -nblocks; i; i++)
  {
    unsigned int k1 = getblock32(blocks,i);

    k1 *= c1;
    k1 = ROTL32(k1,15);
    k1 *= c2;
    
    h1 ^= k1;
    h1 = ROTL32(h1,13); 
    h1 = h1*5+0xe6546b64;
  }

  //----------
  // tail

  const unsigned char * tail = (const unsigned char*)(data + nblocks*4);

  unsigned int k1 = 0;

  switch(len & 3)
  {
  case 3: k1 ^= tail[2] << 16;
  case 2: k1 ^= tail[1] << 8;
  case 1: k1 ^= tail[0];
          k1 *= c1; k1 = ROTL32(k1,15); k1 *= c2; h1 ^= k1;
  };

  //----------
  // finalization

  h1 ^= len;

  h1 = fmix32(h1);

  *(unsigned int*)out = h1;
}

#endif
