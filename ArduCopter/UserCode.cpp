#include "Copter.h"
//#include "Parameters.h"

ParametersG2 _G2;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

//UWB坐标根据与NE角度差换算成NE坐标系坐标
void AP_UWB::UWB_TO_NE(float x,float y)
{
    //UWB坐标系与NE坐标系分为四个角度差（0-90，90-180，180-270，270-360）
    //每个角度差里分为UWB第一象限和第四象限（x轴垂直向上为正，y轴水平向右为正，顺时针）
    //每个UWB象限里又分为两个NE坐标系的象限，总共有16种情况（主要为符号的不同）
    Vector2f _uwb,_ne;  //_ne.x=ne.n ,_ne.y=ne.e;
    float _radian;
    float _angle;
    _uwb.x=fabsf(x);
    _uwb.y=fabsf(y);
    
    _angle=_G2.uwb_to_ne_angle; 
    if((_angle >=0) && (_angle <90))
    {
        _radian=_angle*pi/180;
        if((x >=0) && (y >=0))   
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=_uwb.x;
                _ne.y=_uwb.y;
            }
            else if(_uwb.y >= _uwb.x*tanf(_radian))     //uwb第一象限，ne第一象限
            {
                _ne.x=fabsf(_uwb.y/sinf(_radian)-(_uwb.y/tanf(_radian)-_uwb.x)*cosf(_radian));
                _ne.y=fabsf(_uwb.y*cosf(_radian)-_uwb.x*sinf(_radian));
            }
            else                                     //uwb第一象限，ne第四象限
            {
                _ne.x=fabsf(_uwb.x/cosf(_radian)-(_uwb.x*tanf(_radian)-_uwb.y)*sinf(_radian));
                _ne.y=-fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
            }
        }
        else if((x >=0) && (y <0))
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=_uwb.x;
                _ne.y=-_uwb.y;
            }
            else if(_uwb.x <= _uwb.y*tanf(_radian))     //uwb第四象限，ne第三象限
            {
                _ne.x=-fabsf(_uwb.y*sinf(_radian)-_uwb.x*cosf(_radian));
                _ne.y=-fabsf(_uwb.y/cosf(_radian)-(_uwb.y*tanf(_radian)-_uwb.x)*sinf(_radian));
            }
            else                                      //uwb第四象限，ne第四象限
            {
                _ne.x=fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
                _ne.y=-fabsf(_uwb.x/sinf(_radian)-(_uwb.x/tanf(_radian)-_uwb.y)*cosf(_radian));
            }
        }
    }
    else if((_angle >=90) && (_angle <180))
    {
        _radian=(_angle-90)*pi/180;
        if((x >=0) && (y >=0))   
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=_uwb.y;
                _ne.y=-_uwb.x;
            }
            else if(_uwb.y >= _uwb.x*tanf(_radian))     //uwb第一象限，ne第四象限
            {
                _ne.x=fabsf(_uwb.y*cosf(_radian)-_uwb.x*sinf(_radian));
                _ne.y=-fabsf(_uwb.y/sinf(_radian)-(_uwb.y/tanf(_radian)-_uwb.x)*cosf(_radian));
            }
            else                                     //uwb第一象限，ne第三象限
            {
                _ne.x=-fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
                _ne.y=-fabsf(_uwb.x/cosf(_radian)-(_uwb.x*tanf(_radian)-_uwb.y)*sinf(_radian));              
            }
        }
        else if((x >=0) && (y <0))
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=-_uwb.y;
                _ne.y=-_uwb.x;
            }
            else if(_uwb.x <= _uwb.y*tanf(_radian))     //uwb第四象限，ne第二象限
            {
                _ne.x=-fabsf(_uwb.y/cosf(_radian)-(_uwb.y*tanf(_radian)-_uwb.x)*sinf(_radian));
                _ne.y=fabsf(_uwb.y*sinf(_radian)-_uwb.x*cosf(_radian));
            }
            else                                      //uwb第四象限，ne第三象限
            {
                _ne.x=-fabsf(_uwb.x/sinf(_radian)-(_uwb.x/tanf(_radian)-_uwb.y)*cosf(_radian));
                _ne.y=-fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
            }
        }
    }
    else if((_angle >=180) && (_angle <270))
    {
        _radian=(_angle-180)*pi/180;
        if((x >=0) && (y >=0))   
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=-_uwb.x;
                _ne.y=-_uwb.y;
            }
            else if(_uwb.y >= _uwb.x*tanf(_radian))     //uwb第一象限，ne第三象限
            {
                _ne.x=-fabsf(_uwb.y/sinf(_radian)-(_uwb.y/tanf(_radian)-_uwb.x)*cosf(_radian));
                _ne.y=-fabsf(_uwb.y*cosf(_radian)-_uwb.x*sinf(_radian));
            }
            else                                //uwb第一象限，ne第二象限
            {
                _ne.x=-fabsf(_uwb.x/cosf(_radian)-(_uwb.x*tanf(_radian)-_uwb.y)*sinf(_radian));
                _ne.y=fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
            }
        }
        else if((x >=0) && (y <0))
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=-_uwb.x;
                _ne.y=_uwb.y;
            }
            else if(_uwb.x <= _uwb.y*tanf(_radian))     //uwb第四象限，ne第一象限
            {
                _ne.x=fabsf(_uwb.y*sinf(_radian)-_uwb.x*cosf(_radian));
                _ne.y=fabsf(_uwb.y/cosf(_radian)-(_uwb.y*tanf(_radian)-_uwb.x)*sinf(_radian));
            }
            else                                   //uwb第四象限，ne第二象限
            {
                _ne.x=-fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
                _ne.y=fabsf(_uwb.x/sinf(_radian)-(_uwb.x/tanf(_radian)-_uwb.y)*cosf(_radian));
            }
        }
    }
    else if((_angle >=270) && (_angle <360))
    {
        _radian=(_angle-270)*pi/180;
        if((x >=0) && (y >=0))   
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=-_uwb.y;
                _ne.y=_uwb.x;
            }
            else if(_uwb.y >= _uwb.x*tanf(_radian))     //uwb第一象限，ne第二象限
            {
                _ne.x=-fabsf(_uwb.y*cosf(_radian)-_uwb.x*sinf(_radian));
                _ne.y=fabsf(_uwb.y/sinf(_radian)-(_uwb.y/tanf(_radian)-_uwb.x)*cosf(_radian));
            }
            else                                //uwb第一象限，ne第一象限
            {
                _ne.x=fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
                _ne.y=fabsf(_uwb.x/cosf(_radian)-(_uwb.x*tanf(_radian)-_uwb.y)*sinf(_radian));
            }
        }
        else if((x >=0) && (y <0))
        {
            if(_radian <= 0.00001)    //坐标轴重合
            {
                _ne.x=_uwb.y;
                _ne.y=_uwb.x;
            }
            else if(_uwb.x <= _uwb.y*tanf(_radian))     //uwb第四象限，ne第四象限
            {
                _ne.x=fabsf(_uwb.y/cosf(_radian)-(_uwb.y*tanf(_radian)-_uwb.x)*sinf(_radian));
                _ne.y=-fabsf(_uwb.y*sinf(_radian)-_uwb.x*cosf(_radian));
            }
            else                                //uwb第四象限，ne第一象限
            {
                _ne.x=fabsf(_uwb.x/sinf(_radian)-(_uwb.x/tanf(_radian)-_uwb.y)*cosf(_radian));
                _ne.y=fabsf(_uwb.x*cosf(_radian)-_uwb.y*sinf(_radian));
            }
        }
    }
    //将转换后的ne坐标赋值给相关变量
    gcs().send_text(MAV_SEVERITY_CRITICAL, "RX success all");

}
