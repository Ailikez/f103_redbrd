#ifndef __ALG_UTILS_H__
  #define __ALG_UTILS_H__
//  #include "stdlib.h"
//  #include "math.h"
  #define _swap(a, b) {a = a ^ b; b = a ^ b; a = a ^ b;}
  #define _abs(x) ((x)>=0 ? (x):-(x))
  
//  #define _abs(x) ((1-((((x)>>(sizeof(x)*8-1))&1)<<1))*(x))
//  #define _abs abs
  extern void _itoa(int n, char *str, char _fill);
  extern void _ftoa(float f, char *_str, char _fill);

#endif
