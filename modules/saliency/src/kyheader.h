#ifndef KYHEADER_H
#define KYHEADER_H

#include <assert.h>
#include <string>
#include <vector>
#include <functional>
#include <list>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <time.h>
#include <fstream>
//#include <atlstr.h>
//#include <atltypes.h>
#include <omp.h>


// TODO: reference additional headers your program requires here
//#include "LibLinear/linear.h"
//#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"

#define CV_VERSION_ID CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)
#ifdef _DEBUG
#define cvLIB(name) "opencv_" name CV_VERSION_ID "d"
#else
#define cvLIB(name) "opencv_" name CV_VERSION_ID
#endif

#pragma comment( lib, cvLIB("core"))
#pragma comment( lib, cvLIB("imgproc"))
#pragma comment( lib, cvLIB("highgui"))
using namespace cv;
using namespace std;
#ifdef WIN32
/* windows stuff */
#else
typedef unsigned long DWORD;
typedef unsigned short WORD;
typedef unsigned int UNINT32;
typedef bool BOOL;
typedef void *HANDLE;
typedef unsigned char byte;
#endif

typedef std::vector<int> vecI;
typedef const std::string CStr;
typedef const Mat CMat;
typedef std::vector<std::string> vecS;
typedef std::vector<Mat> vecM;
typedef std::vector<float> vecF;
typedef std::vector<double> vecD;

enum{CV_FLIP_BOTH = -1, CV_FLIP_VERTICAL = 0, CV_FLIP_HORIZONTAL = 1};
#define _S(str) ((str).c_str())
#define CHK_IND(p) ((p).x >= 0 && (p).x < _w && (p).y >= 0 && (p).y < _h)
#define CV_Assert_(expr, args) \
{\
    if(!(expr)) {\
    String msg = cv::format args; \
    printf("%s in %s:%d\n", msg.c_str(), __FILE__, __LINE__); \
    cv::error(cv::Exception(CV_StsAssert, msg, __FUNCTION__, __FILE__, __LINE__) ); }\
}

using namespace std;

// Return -1 if not in the list
template<typename T>
static inline int findFromList(const T &word, const vector<T> &strList) {
    //TODO delete test code
    //cout << "\n\n" << "word" <<" "<< word << endl;
    for(int i=0; i<strList.size(); i++) {
	//cout <<"test word:"<< word << " " << endl;
	//cout << "Size w " << word.size() << " Size L "<< strList[i].size() << endl;
    }
    
    vector<String>::iterator it = std::find(strList.begin(),strList.end(), word);
    if (it == strList.end())
    {
        return -1;
    } else
    {
      vector<String>::iterator index = std::distance(strList.begin(), it);
	//cout << "index" <<" "<< index << endl;
        return index;
    }
} 
/*template<typename T>
static inline int findFromList(const string &word, const vector<T> &strList) {
    //for(int i=0; i<strList.size(); i++){
	//cout <<"element: " <<strList[i]<<" "<<word << endl;
      //if (std::strcmp(word.c_str(),strList[i].c_str())==0) return i;
  }
  
  return -1;
}
*/

template<typename T> inline T sqr(T x) { return x * x; } // out of range risk for T = byte, ...
template<class T, int D> inline T vecSqrDist(const Vec<T, D> &v1, const Vec<T, D> &v2) {T s = 0; for (int i=0; i<D; i++) s += sqr(v1[i] - v2[i]); return s;} // out of range risk for T = byte, ...
template<class T, int D> inline T    vecDist(const Vec<T, D> &v1, const Vec<T, D> &v2) { return sqrt(vecSqrDist(v1, v2)); } // out of range risk for T = byte, ...

inline Rect Vec4i2Rect(Vec4i &v){return Rect(Point(v[0] - 1, v[1] - 1), Point(v[2], v[3])); }

#ifdef __WIN32
    #define INT64 long long
#else
    #define INT64 long
    typedef unsigned long UINT64;
#endif

/////
#if (_MSC_VER >= 1500)
# include <intrin.h>
# define POPCNT(x) __popcnt(x)
# define POPCNT64(x) __popcnt64(x)
#endif

#if defined(__GNUC__)
# define POPCNT(x) __builtin_popcount(x)
# define POPCNT64(x) __builtin_popcountll(x)
#endif

inline int popcnt64(register uint64_t u)
{
    u = (u & 0x5555555555555555) + ((u >> 1) & 0x5555555555555555);
    u = (u & 0x3333333333333333) + ((u >> 2) & 0x3333333333333333);
    u = (u & 0x0f0f0f0f0f0f0f0f) + ((u >> 4) & 0x0f0f0f0f0f0f0f0f);
    u = (u & 0x00ff00ff00ff00ff) + ((u >> 8) & 0x00ff00ff00ff00ff);
    u = (u & 0x0000ffff0000ffff) + ((u >>16) & 0x0000ffff0000ffff);
    u = (u & 0x00000000ffffffff) + ((u >>32) & 0x00000000ffffffff);
    return u;
}

inline int popcnt(register uint32_t u)
{
    u = (u & 0x55555555) + ((u >> 1) & 0x55555555);
    u = (u & 0x33333333) + ((u >> 2) & 0x33333333);
    u = (u & 0x0f0f0f0f) + ((u >> 4) & 0x0f0f0f0f);
    u = (u & 0x00ff00ff) + ((u >> 8) & 0x00ff00ff);
    u = (u & 0x0000ffff) + ((u >>16) & 0x0000ffff);
    return u;
}

inline int popcnt64_nibble(register uint64_t u)
{
    static const uint8_t Table[] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4
    };

    int c = 0;
    while (u)
    {
        c += Table[u & 0xf];
        u >>= 4;
    }
    return c;
}

inline int popcnt_nibble(register uint32_t u)
{
    static const uint8_t Table[] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4
    };

    int c = 0;
    while (u)
    {
        c += Table[u & 0xf];
        u >>= 4;
    }
    return c;
}

inline int popcnt64_byte(register uint64_t u)
{
#define B2(k) k, k+1, k+1, k+2
#define B4(k) B2(k), B2(k+1), B2(k+1), B2(k+2)
#define B6(k) B4(k), B4(k+1), B4(k+1), B4(k+2)
    static const uint8_t Table[] = {
        B6(0), B6(1), B6(1), B6(2)
    };
#undef B6
#undef B4
#undef B2

    int c = 0;
    while (u)
    {
        c += Table[u & 0xff];
        u >>= 8;
    }
    return c;
}

inline int popcnt_byte(register uint32_t u)
{
#define B2(k) k, k+1, k+1, k+2
#define B4(k) B2(k), B2(k+1), B2(k+1), B2(k+2)
#define B6(k) B4(k), B4(k+1), B4(k+1), B4(k+2)
    static const uint8_t Table[] = {
        B6(0), B6(1), B6(1), B6(2)
    };
#undef B6
#undef B4
#undef B2

    int c = 0;
    while (u)
    {
        c += Table[u & 0xff];
        u >>= 8;
    }
    return c;
}


/////
#include "CmTimer.h"
#include "CmFile.h"
#endif // KYHEADER_H
