#ifndef _CUSTOM_STRUCT_H_
#define _CUSTOM_STRUCT_H_

#define _OPENCV_COMPATIBLE_

#include <iostream>
#include <vector>
#include <memory>

#ifdef _OPENCV_COMPATIBLE_
	#include "opencv2/highgui/highgui.hpp"
	#include "opencv2/imgproc/imgproc.hpp"
	#include "opencv2/opencv.hpp"
	#include "Eigen/Dense" 
#endif

#define PI 3.141592653589793238
#define D2R 0.017453292519943
#define R2D 57.295779513082320

typedef unsigned short ushort; // 2 bytes
typedef unsigned char uchar;   // 1 byte
using namespace std;

namespace chk {
	template <typename Tp_> 
	struct Point2 { // for 2d pixel points
		Tp_ x;
		Tp_ y;
		Point2() : x(0), y(0) {}; 
		Point2(Tp_ x_, Tp_ y_) : x(x_), y(y_) {};
		Point2(const Point2& pt) { x = pt.x; y = pt.y;};

		// operator overloading
		double norm() { return sqrt((double)x*x + (double)y*y); };
        double dot(const Point2& pt) { return (x*pt.x + y*pt.y); };
        double cross2d(const Point2& pt) { return (-pt.y*x + pt.x*y); };
        template <typename Tin_>// for arbitrary datatype.
        friend Point2 operator* (Point2& pt1, Tin_ scalar) { return Point2(pt1.x*(float)scalar, pt1.y*(float)scalar); };
        
        friend Point2 operator+ (Point2& pt1, Point2& pt2) { return Point2(pt1.x + pt2.x, pt1.y + pt2.y); };
        friend Point2 operator- (Point2& pt1, Point2& pt2) { return Point2(pt1.x - pt2.x, pt1.y - pt2.y); };
        Point2& operator+=(const Point2& rhs) { x += rhs.x; y += rhs.y; return *this; };
        Point2& operator-=(const Point2& rhs) { x -= rhs.x; y -= rhs.y; return *this; };

        template <typename Tin_>// for arbitrary datatype.
        Point2& operator/=(const Tin_& rhs) { x /= (double)rhs; y /= (double)rhs; return *this; };
        template <typename Tin_>
        Point2& operator*=(const Tin_& rhs) { x *= (double)rhs; y *= (double)rhs; return *this; };
		Point2& operator=(const Point2& ref) { x = ref.x; y = ref.y; return *this; };
		friend ostream& operator<<(ostream& os, const Point2& pt) {
			os << "[" << pt.x << "," << pt.y << "]";
			return os;
		};
		cv::Point2f chk2cv() { return cv::Point2f(x, y); };
	};

	template <typename Tp_>
	struct Point3 { // (3d point)
		Tp_ x;
		Tp_ y;
		Tp_ z;
		Point3() : x(0), y(0), z(0) {}; // ±âº» »ýŒºÀÚ
		Point3(Tp_ x_, Tp_ y_, Tp_ z_) : x(x_), y(y_), z(z_) {};
		Point3(const Point3& pt) { x = pt.x; y = pt.y; z = pt.z; }; // º¹»ç »ýŒºÀÚ

        double norm() { return sqrt((double)x*x + (double)y*y + (double)z*z); };
        double dot(const Point3& pt) { return (x*pt.x + y*pt.y + z*pt.z); };
        // double cross3d(const Point3& pt) { return (-pt.y*x + pt.x*y); };
        template <typename Tin_>// for arbitrary datatype.
        friend Point3 operator* (Point3& pt1, Tin_ scalar) { return Point3(pt1.x*(float)scalar, pt1.y*(float)scalar, pt1.z*(float)scalar); };

        friend Point3 operator+ (Point3& pt1, Point3& pt2) { return Point3(pt1.x + pt2.x, pt1.y + pt2.y, pt1.z + pt2.z); };
        friend Point3 operator- (Point3& pt1, Point3& pt2) { return Point3(pt1.x - pt2.x, pt1.y - pt2.y, pt1.z - pt2.z); };
        Point3& operator+=(const Point3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; };
        Point3& operator-=(const Point3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; };

        template <typename Tin_>// for arbitrary datatype.
        Point3& operator/=(const Tin_& rhs) { x /= (double)rhs; y /= (double)rhs; z /= (double)rhs; return *this; };
        template <typename Tin_>
        Point3& operator*=(const Tin_& rhs) { x *= (double)rhs; y *= (double)rhs; z *= (double)rhs; return *this; };

        Point3& operator=(const Point3& ref) { x = ref.x; y = ref.y; z = ref.z; return *this; };
        friend ostream& operator<<(ostream& os, const Point3& pt) {
            os << "[" << pt.x << "," << pt.y <<","<<pt.z<< "]";
            return os;
        };
        cv::Point3f chk2cv() { return cv::Point3f(x, y, z); };
	};


    // defines nicknames
    typedef Point2<uchar> Point2uc;
    typedef Point2<char> Point2c;
	typedef Point2<ushort> Point2us;
	typedef Point2<short> Point2s;
	typedef Point2<int> Point2i;
	typedef Point2<float> Point2f;
	typedef Point2<double> Point2d;

	typedef Point3<uchar> Point3uc;
    typedef Point3<char> Point3c;
	typedef Point3<ushort> Point3us;
	typedef Point3<short> Point3s;
	typedef Point3<int> Point3i;
	typedef Point3<float> Point3f;
	typedef Point3<double> Point3d;

#ifdef _OPENCV_COMPATIBLE_
	template <typename Tp_>
	inline void chk2cv(const chk::Point2<Tp_>& pt, cv::Point2f& pt_cv) {
		pt_cv.x = pt.x;
		pt_cv.y = pt.y;
	};
	template <typename Tp_>
	inline void chk2cv(const chk::Point3<Tp_>& pt, cv::Point3f& pt_cv) {
		pt_cv.x = pt.x;
		pt_cv.y = pt.y;
		pt_cv.z = pt.z;
	};
#endif
};

#endif