#include "UWB_LOCATION.h"

UWB_LOCATION::UWB_LOCATION(){
    LOC_EN = false;
    a=b=c=b1={ {0,0,0}, 0 };
}

UWB_LOCATION::~UWB_LOCATION(){}


/**
 *                      * b
 *                      |
 *         a            |
 *          *-----------*b1
 *         /|          /|
 *        / |         / |
 *       /  |        /  |
 *      |---|-------|   |
 *      |   |       |   |
 *      |  O*-------|--*c
 *      | /         | /
 *      |/          |/
 *      |-----------| 
 */  
// point a,b,c,b1, b1点a点在bc上的投影 以O点作为零点,坐标为(0,0,0) 
bool UWB_LOCATION::loc_init(unsigned int ab, unsigned int ac, unsigned int bc) //cm
{
    TRIANGLE_TYPE type = is_triangle(ab, ac, bc);
    if (type == TRIANGLE_TYPE::NONE_TRIANGLE)   return false;
    double S = Heron_formula(ab, ac, bc);
    
    c.loc.y = b1.loc.y = b.loc.y = area_get_high(S, bc);
    a.loc.z = b1.loc.z = sqrt(ac*ac - c.loc.y*c.loc.y);
    b.loc.z = bc;
    LOC_EN = true;
    return true;
}

bool UWB_LOCATION::loc_pos(unsigned int na_cm, unsigned int nb_cm, unsigned int nc_cm)
{
    //Snbc:三角形nbc的面积,nHbc:n在bc上的高,temp_b1c_w:n在bc上的投影到b1点的距离,
    //nb1_cm:n到b1点的距离,Snab1:三角形nab1的面积,nHab1_cm:n在ab1上的高,
    //n1Wab1_cm:n点在面oabc上的投影在ab1上的投影到b1点距离,n1c_cm:n点在面上的投影到c点的距离
    double Snbc, nHbc, temp_b1c_w_cm, nb1_cm, Snab1, nHab1_cm, n1Wab1_cm, n1c_cm;
    TRIANGLE_TYPE type_nbc = is_triangle(nc_cm, b.loc.z, nb_cm);
    if(type_nbc == TRIANGLE_TYPE::NONE_TRIANGLE)    return false;
    Snbc = Heron_formula(nc_cm, b.loc.z, nb_cm);
    nHbc =area_get_high(Snbc, b.loc.z);
    _copter_uwb.loc.z = sqrt(nc_cm*nc_cm - nHbc*nHbc);//锐角三角形下测量点z数据
    if(type_nbc == TRIANGLE_TYPE::OBTUSE_Angle)
    {
        _copter_uwb.loc.z = - _copter_uwb.loc.z;
    }
    if(b1.loc.z ==  _copter_uwb.loc.z)
    {
        nb1_cm = sqrt(nc_cm*nc_cm - b1.loc.z*b1.loc.z);
    }
    else
    {
        temp_b1c_w_cm = b1.loc.z - _copter_uwb.loc.z;
        if(temp_b1c_w_cm < 0) temp_b1c_w_cm = -temp_b1c_w_cm;
        nb1_cm = sqrt(temp_b1c_w_cm*temp_b1c_w_cm + nHbc*nHbc);
    }
    temp = nb1_cm;
    TRIANGLE_TYPE type_nab1 = is_triangle(na_cm, b1.loc.y, nb1_cm);
    if(type_nab1 == TRIANGLE_TYPE::NONE_TRIANGLE)    return false;

    Snab1 = Heron_formula(na_cm, b1.loc.y, nb1_cm);
    nHab1_cm = area_get_high(Snab1, b1.loc.y);
    _copter_uwb.loc.y = sqrt(na_cm*na_cm - nHab1_cm*nHab1_cm);////锐角三角形下测量点y数据
    if(type_nab1 == TRIANGLE_TYPE::OBTUSE_Angle)
    {
        _copter_uwb.loc.y = - _copter_uwb.loc.y;
    }

    n1Wab1_cm = (b1.loc.y - _copter_uwb.loc.y);
    double temp_z = abs(_copter_uwb.loc.z);
    n1c_cm = sqrt(n1Wab1_cm*n1Wab1_cm + temp_z*temp_z);
    _copter_uwb.loc.x = sqrt(nc_cm*nc_cm - n1c_cm*n1c_cm);//x不能为负
    return true;
}

/**
 * @brief 验证是否为三角形,并返回前两个边对应的夹角是什么角
 * @param L:三边(cm)
 * @return TRIANGLE_TYPE NONE_TRIANGLE(非三角形); ACUTE_Angle(锐角); RIGHT_Angle(直角); OBTUSE_Angle(钝角)
 */
UWB_LOCATION::TRIANGLE_TYPE UWB_LOCATION::is_triangle(unsigned int L1, unsigned int L2, unsigned int L3)
{
    if(L1 + L2 > L3 && L1 + L3 > L2 && L2 + L3 > L1)
    {
        if(L3*L3 < L1*L1+L2*L2)
            return ACUTE_Angle;
        else if (L3*L3 > L1*L1+L2*L2)
            return OBTUSE_Angle;
        else
            return RIGHT_Angle;
    }
    return NONE_TRIANGLE;
}



