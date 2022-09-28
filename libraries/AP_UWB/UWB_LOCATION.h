#ifndef __UWB_LOCATION_H__
#define __UWB_LOCATION_H__
#include <math.h>

class UWB_LOCATION
{
public:

private:
    /* data */
    typedef enum triangle_type{
        NONE_TRIANGLE = -1,
        ACUTE_Angle = 1,
        RIGHT_Angle = 2,
        OBTUSE_Angle = 3,    
    }TRIANGLE_TYPE;

    typedef struct loc
    {
        double x;
        double y;
        double z;
    }LOC;

public:
    UWB_LOCATION(/* args */);
    ~UWB_LOCATION();

    double temp;

    typedef struct loc_point
    {
        struct loc loc; //参考点坐标
        double dis_n; //参考点距离n点
    }LOC_POINT;

    typedef struct point_pos
    {
        struct loc loc;
        unsigned int dis_a;
        unsigned int dis_b1;
        unsigned int dis_c;
    }POINT_POS;

    //坐标系初始化
    bool loc_init(unsigned int ab, unsigned int ac, unsigned int bc);

    bool loc_pos(unsigned int na_cm, unsigned int nb_cm, unsigned int nc_cm);

    POINT_POS get_n_pos() { return _copter_uwb; }

    //判断是否为三角形
    static TRIANGLE_TYPE is_triangle(unsigned int L1, unsigned int L2, unsigned int L3);
    /**
     * @brief 海伦公式,通过三边求三角形面积
     * @param L:三边(cm)
     * @return double 三角形面积 平方厘米
     */
    double Heron_formula(double L1, double L2, double L3) {
        double p = (L1/100+L2/100+L3/100)/2;
        return sqrt(p*(p-L1/100)*(p-L2/100)*(p-L3/100.0))*10000;
        }
    //面积求高
    static double area_get_high(unsigned int S, unsigned int w) { return 2*S/w; }

    LOC_POINT a,b,c,b1;
    POINT_POS _copter_uwb;
    bool LOC_EN;
    
private:

};

#endif  //__UWB_H__