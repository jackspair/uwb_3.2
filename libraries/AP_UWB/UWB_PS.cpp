//
// Created by liu_jiacheng on 2022/9/28.
//

#include "UWB_PS.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>

extern "C"
{
  #include "uwbSqrt.h"
}


#define uwbSqrt(x)  rsqrt(x)

UWB_PS::UWB_PS(/* args */)
{
  uwb_PS = {0};
  copter_uwb = {0};
}

// point a,b,c,b1, b1��a����bc�ϵ�ͶӰ ��O����Ϊ���,����Ϊ(0,0,0)
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
int UWB_PS::loc_init(unsigned int ab, unsigned int ac, unsigned int bc) //cm
{
  TRIANGLE_TYPE type = is_triangle(ab, ac, bc);
  if (type == NONE_TRIANGLE)   return -1;
  temp = uwb_PS.c.loc_cm.y = uwb_PS.b.loc_cm.y = uwb_PS.b1.loc_cm.y = side_get_high_cm(ab/100.0, ac/100.0, bc/100.0);
  if(ac/100.0 - uwb_PS.c.loc_cm.y/100.0 <= 0.0001)  return -1;
  uwb_PS.a.loc_cm.z = uwb_PS.b1.loc_cm.z = triangle_get_side_cm(ac/100.0, uwb_PS.c.loc_cm.y/100.0);
  uwb_PS.b.loc_cm.z = (int)bc;
  uwb_PS.system_EN = 1;
  return 0;
}

int UWB_PS::loc_pos(unsigned int na_cm, unsigned int nb_cm, unsigned int nc_cm)
{
//nHbc:n��bc�ϵĸ�,temp_b1c_w:n��bc�ϵ�ͶӰ��b1��ľ���,
  //nb1_m:n��b1��ľ���,Snab1:������nab1�����,nHab1_m:n��ab1�ϵĸ�,
  //n1Wab1_cm:n������oabc�ϵ�ͶӰ��ab1�ϵ�ͶӰ��b1�����,n1c_m:n�������ϵ�ͶӰ��c��ľ���
  double nHbc, temp_b1c_w_cm, nb1_m, nHab1_m, n1Wab1_cm, n1c_m;
  nHbc = temp_b1c_w_cm = nb1_m = nHab1_m = n1Wab1_cm = n1c_m = 0;

  TRIANGLE_TYPE type_nbc = is_triangle(nc_cm, uwb_PS.b.loc_cm.z, nb_cm);
  if(type_nbc == NONE_TRIANGLE)    return -1;

  nHbc = side_get_high_m(nc_cm/100.0, nb_cm/100.0, uwb_PS.b.loc_cm.z/100.0);
  if(nc_cm/100.0 - nHbc <= 0.0001)  return -1;
  copter_uwb.loc_cm.z = triangle_get_side_cm(nc_cm/100.0, nHbc);
  if(type_nbc == OBTUSE_Angle)
  {
    copter_uwb.loc_cm.z = - copter_uwb.loc_cm.z;
  }
  if(uwb_PS.b1.loc_cm.z - copter_uwb.loc_cm.z <= 0.001)
  {
    nb1_m = triangle_get_side_m(nc_cm / 100.0, uwb_PS.b1.loc_cm.z / 100.0);
  }
  else
  {
    temp_b1c_w_cm = fabs(uwb_PS.b1.loc_cm.z - copter_uwb.loc_cm.z);
    nb1_m = triangle_get_hypotenuse_m(temp_b1c_w_cm / 100.0, nHbc);
  }

  TRIANGLE_TYPE type_nab1 = is_triangle((unsigned int)(nb1_m*100), uwb_PS.b1.loc_cm.y, na_cm);
  if(type_nab1 == NONE_TRIANGLE)    return -1;

  nHab1_m = side_get_high_m(na_cm / 100.0, nb1_m, uwb_PS.b1.loc_cm.y / 100.0);
  if(nb1_m - nHab1_m <= 0.0001)  return -1;
  copter_uwb.loc_cm.y = uwb_PS.b1.loc_cm.y - triangle_get_side_cm(nb1_m, nHab1_m); //����������²�����y����
  if(type_nab1 == OBTUSE_Angle)
  {
    copter_uwb.loc_cm.y = fabs(copter_uwb.loc_cm.y) + uwb_PS.b1.loc_cm.y;
  }

  n1Wab1_cm = (uwb_PS.b1.loc_cm.y - copter_uwb.loc_cm.y);
  double temp_z = uwb_PS.b1.loc_cm.z - copter_uwb.loc_cm.z;
//          abs(copter_uwb.loc_cm.z);
  n1c_m = triangle_get_hypotenuse_m(n1Wab1_cm/100.0, temp_z/100.0);
  if(nb1_m - n1c_m <= 0.0001)  return -1;
  copter_uwb.loc_cm.x = triangle_get_side_cm(nb1_m, n1c_m);//x����Ϊ��

  return 0;
}


/**
 * @brief ��֤�Ƿ�Ϊ������,������ǰ�����߶�Ӧ�ļн���ʲô��
 * @param L:����(cm)
 * @return TRIANGLE_TYPE NONE_TRIANGLE(��������); ACUTE_Angle(���); RIGHT_Angle(ֱ��); OBTUSE_Angle(�۽�)
 */
UWB_PS::TRIANGLE_TYPE UWB_PS::is_triangle(unsigned int const L1_cm, unsigned int const L2_cm, unsigned int const L3_cm)
{
  if(L1_cm + L2_cm > L3_cm && L1_cm + L3_cm > L2_cm && L2_cm + L3_cm > L1_cm)
  {
    if(L3_cm * L3_cm < L1_cm * L1_cm + L2_cm * L2_cm)
      return ACUTE_Angle;
    else if (L3_cm * L3_cm > L1_cm * L1_cm + L2_cm * L2_cm)
      return OBTUSE_Angle;
    else
      return RIGHT_Angle;
  }
  return NONE_TRIANGLE;
}

/**
 * @brief ���׹�ʽ,ͨ�����������������
 * @param L:����(m)
 * @return double ��������� ƽ��m
 */
double UWB_PS::Heron_formula(double const L1_m, double const L2_m, double const L3_m)
{
  double p_m = (L1_m + L2_m + L3_m) / 2;
  return uwbSqrt(p_m * (p_m - L1_m) * (p_m - L2_m) * (p_m - L3_m));
}
//������
inline double UWB_PS::area_get_high(double const S_m2, double const w_m)
{
  return 2 * S_m2 / w_m;
}

//��������L3�ϵĸ� ������
inline double UWB_PS::side_get_high_m(double const L1_m, double const L2_m, double const L3_m)
{
  return area_get_high(Heron_formula(L1_m, L2_m, L3_m), L3_m);
}

//��������L3�ϵĸ� ��������
inline double UWB_PS::side_get_high_cm(double const L1_m, double const L2_m, double const L3_m)
{
  return area_get_high(Heron_formula(L1_m, L2_m, L3_m), L3_m)*100;
}

//���ɶ�����ֱ��֮һ ������
inline double  UWB_PS::triangle_get_side_m(double hypotenuse_m, double side_m)
{
  return uwbSqrt(hypotenuse_m * hypotenuse_m - side_m * side_m);
}

//���ɶ�����ֱ��֮һ ��������
inline double UWB_PS::triangle_get_side_cm(double hypotenuse_m, double side_m)
{
  return uwbSqrt(hypotenuse_m * hypotenuse_m - side_m * side_m)*100;
}

//���ɶ�����б�� ������
inline double  UWB_PS::triangle_get_hypotenuse_m(double side1_m, double side2_m)
{
  return uwbSqrt(side1_m * side1_m + side2_m * side2_m);
}

//���ɶ�����б�� ��������
inline double UWB_PS::triangle_get_hypotenuse_cm(double side1_m, double side2_m)
{
  return uwbSqrt(side1_m * side1_m - side2_m * side2_m)*100;
}