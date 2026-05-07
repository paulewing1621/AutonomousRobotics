#ifndef LEICA_POINT_TYPE_H
#define LEICA_POINT_TYPE_H

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#if 0
struct PointXYZRGBI : public pcl::PointXYZRGBI {
    int32_t & intensity;
    PointXYZRGBI() : pcl::PointXYZRGBI(), intensity((int32_t&)intensity) {}
    PointXYZRGBI(const PointXYZRGBI & p) : pcl::PointXYZRGBI(p), intensity((int32_t&)intensity) {}
    PointXYZRGBI(const PointXYZRGBI & p) : pcl::PointXYZRGBI(p), intensity((int32_t&)intensity) {}
    PointXYZRGBI(float xf, float yf, float zf, int r, int g, int b, int intensity) :
        pcl::PointXYZRGBI(r,g,b,(uint32_t)intensity), intensity((int32_t&)intensity) {x=xf;y=yf;z=zf;}
    //int32_t & intensity() {return (int32_t&)intensity;}
    //const int32_t & intensity() const {return (const int32_t&)intensity;}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    const pcl::PointXYZRGBI & topcl() const {return (const pcl::PointXYZRGBI&)*this;}
};
#endif


struct EIGEN_ALIGN16 _PointXYZRGBI
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    float intensity;
    float snr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct EIGEN_ALIGN16 PointXYZRGBI : public _PointXYZRGBI
{
    inline PointXYZRGBI (const _PointXYZRGBI &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        rgba = p.rgba;
        intensity = p.intensity;
        snr = p.snr;
    }

    inline PointXYZRGBI ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        r = g = b = 0;
        a = 255;
        intensity = 255;
        snr = 1;
    }

    inline PointXYZRGBI (uint8_t _r, uint8_t _g, uint8_t _b, float _intensity)
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        r = _r;
        g = _g;
        b = _b;
        a = 255;
        intensity = _intensity;
        snr = 1;
    }

    inline PointXYZRGBI (float _x, float _y, float _z, uint8_t _r, uint8_t _g, uint8_t _b, float _intensity, float _snr=1)
    {
        x = _x; 
        y = _y; 
        z = _z; 
        data[3] = 1.0f;
        r = _r;
        g = _g;
        b = _b;
        a = 255;
        intensity = _intensity;
        snr = _snr;
    }

    // friend std::ostream& operator << (std::ostream& os, const PointXYZRGBI& p);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBI,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (std::uint32_t, rgba, rgba)
        (float, intensity, intensity)
        (float, snr, snr)
);

POINT_CLOUD_REGISTER_POINT_WRAPPER(PointXYZRGBI, _PointXYZRGBI)

#endif // LEICA_POINT_TYPE_H
