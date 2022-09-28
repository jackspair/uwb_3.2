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
// point a,b,c,b1, b1��a����bc�ϵ�ͶӰ ��O����Ϊ���,����Ϊ(0,0,0) 
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
    //Snbc:������nbc�����,nHbc:n��bc�ϵĸ�,temp_b1c_w:n��bc�ϵ�ͶӰ��b1��ľ���,
    //nb1_cm:n��b1��ľ���,Snab1:������nab1�����,nHab1_cm:n��ab1�ϵĸ�,
    //n1Wab1_cm:n������oabc�ϵ�ͶӰ��ab1�ϵ�ͶӰ��b1�����,n1c_cm:n�������ϵ�ͶӰ��c��ľ���
    double Snbc, nHbc, temp_b1c_w_cm, nb1_cm, Snab1, nHab1_cm, n1Wab1_cm, n1c_cm;
    TRIANGLE_TYPE type_nbc = is_triangle(nc_cm, b.loc.z, nb_cm);
    if(type_nbc == TRIANGLE_TYPE::NONE_TRIANGLE)    return false;
    Snbc = Heron_formula(nc_cm, b.loc.z, nb_cm);
    nHbc =area_get_high(Snbc, b.loc.z);
    _copter_uwb.loc.z = sqrt(nc_cm*nc_cm - nHbc*nHbc);//����������²�����z����
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
    _copter_uwb.loc.y = sqrt(na_cm*na_cm - nHab1_cm*nHab1_cm);////����������²�����y����
    if(type_nab1 == TRIANGLE_TYPE::OBTUSE_Angle)
    {
        _copter_uwb.loc.y = - _copter_uwb.loc.y;
    }

    n1Wab1_cm = (b1.loc.y - _copter_uwb.loc.y);
    double temp_z = abs(_copter_uwb.loc.z);
    n1c_cm = sqrt(n1Wab1_cm*n1Wab1_cm + temp_z*temp_z);
    _copter_uwb.loc.x = sqrt(nc_cm*nc_cm - n1c_cm*n1c_cm);//x����Ϊ��
    return true;
}

/**
 * @brief ��֤�Ƿ�Ϊ������,������ǰ�����߶�Ӧ�ļн���ʲô��
 * @param L:����(cm)
 * @return TRIANGLE_TYPE NONE_TRIANGLE(��������); ACUTE_Angle(���); RIGHT_Angle(ֱ��); OBTUSE_Angle(�۽�)
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



