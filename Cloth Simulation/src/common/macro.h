#ifndef __H__MACRO__
#define __H__MACRO__

#ifdef _DEBUG
  #define DBG_HALT __asm{int 3}
  #define DBG_EXEC (exp) exp
  #define DBG_ASSERT(exp) { if(!(exp)){ DBG_HALT; } }
  #define DBG_VALID_FLOAT(f) { DBG_ASSERT(f == f); }
  #define DBG_VALID_VEC(v) { DBG_VALID_FLOAT(v.x); DBG_VALID_FLOAT(v.y); DBG_VALID_FLOAT(v.z); }
  #define DBG_VALID_QUAT(q) { DBG_VALID_FLOAT(q.w); DBG_VALID_FLOAT(q.x); DBG_VALID_FLOAT(q.y); DBG_VALID_FLOAT(q.z); }
  #define DBG_VALID_MAT4(m) { for(int i = 0; i < 16; i++) DBG_VALID_FLOAT(m.data[i]); }
  #define DBG_VALID_MAT(m) { for (int n = 0; n < m.nbrOfRow * m.nbrOfCol; n++) DBG_VALID_FLOAT(m.data[n]); }

#else
  #define DBG_HALT
  #define DBG_ASSERT (exp)
  #define DBG_EXEC (exp)
  #define DBG_VALID_FLOAT(f)
  #define DBG_VALID_VEC(v)
  #define DBG_VALID_QUAT(q)
  #define DBG_VALID_MAT4(m)
  #define DBG_VALID_MAT(m)
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(exp) {if(exp != nullptr){delete exp;}}
#endif

#ifndef SAFE_DELETE_TAB
#define SAFE_DELETE_TAB(exp) {if(exp != nullptr){delete[] exp;}}
#endif

#endif //__H__MACRO__