//
// Created by liu_jiacheng on 2022/9/28.
//

#ifndef CPP_UWB_PS_H
#define CPP_UWB_PS_H

//������ö��
typedef enum triangle_type{
    NONE_TRIANGLE = -1,
    ACUTE_Angle = 1,
    RIGHT_Angle = 2,
    OBTUSE_Angle = 3,
}TRIANGLE_TYPE;

//���׾�������
typedef struct loc
{
    int x;
    int y;
    int z;
}LOC;

//�ο����������
typedef struct loc_point
{
    struct loc loc_cm; //�ο�������
    unsigned int dis_n_cm; //�ο������n��
}LOC_POINT;

typedef struct loc_system
{
    int system_EN;
    LOC_POINT a;
    LOC_POINT b;
    LOC_POINT c;
    LOC_POINT b1;
}LOC_SYSTEM;

//����������
typedef struct point_pos
{
    struct loc loc_cm;
    unsigned int dis_a_cm;
    unsigned int dis_b_cm;
    unsigned int dis_b1_cm;
    unsigned int dis_c_cm;
}POINT_POS;


int loc_init(unsigned int ab, unsigned int ac, unsigned int bc);
int loc_pos(unsigned int na_cm, unsigned int nb_cm, unsigned int nc_cm);

TRIANGLE_TYPE is_triangle(unsigned int L1_cm, unsigned int L2_cm, unsigned int L3_cm);
double Heron_formula(double L1_m, double L2_m, double L3_m);
double area_get_high(double S_m2, double w_m);
double side_get_high_m(double L1_m, double L2_m, double L3_m);
int side_get_high_cm(double L1_m, double L2_m, double L3_m);
double triangle_get_side_m(double hypotenuse_m, double side_m);
int triangle_get_side_cm(double hypotenuse_m, double side_m);
double triangle_get_hypotenuse_m(double side1_m, double side2_m);
int triangle_get_hypotenuse_cm(double side1_m, double side2_m);
LOC_SYSTEM uwb_PS_get_system();

#endif //CPP_UWB_PS_H
