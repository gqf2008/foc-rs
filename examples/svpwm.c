#include <math.h>
#include <stdio.h>

#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#ifndef _round
#define _round(x) ((x) >= 0 ? (long)((x) + 0.5f) : (long)((x)-0.5f))
#endif
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ((a) != (NOT_SET))
#define _UNUSED(v) (void)(v)

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

#define NOT_SET -12345.0
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC (NOT_SET)

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI / 101.0f)

void svpwm(float Uq, float Ud, float angle_el, float voltage_limit);
void spwm(float Uq, float Ud, float angle_el, float voltage_limit);
int main()
{
  printf("=================SVPWM=================\n");
  for (int i = 0; i <=500; i++)
  {
     float angle = _3PI_2 + _2PI * i / 500.;
     // printf("i=%d,angle=%f\n",i,angle);
    svpwm(6.0f, 0.0f, angle, 12.0f);
  }

  // printf("=================SPWM=================\n");
  // for (int i = 0; i <=1; i++)
  // {
  //    float angle = _3PI_2 + _2PI * i / 500.;
  //   spwm(6.0f, 0.0f, angle, 12.0f);
  // }

  return 0;
}
// int array instead of float array
// 4x200 points per 360 deg
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
const int sine_array[200] = {0, 79, 158, 237, 316, 395, 473, 552, 631, 710, 789, 867, 946, 1024, 1103, 1181, 1260, 1338, 1416, 1494, 1572, 1650, 1728, 1806, 1883, 1961, 2038, 2115, 2192, 2269, 2346, 2423, 2499, 2575, 2652, 2728, 2804, 2879, 2955, 3030, 3105, 3180, 3255, 3329, 3404, 3478, 3552, 3625, 3699, 3772, 3845, 3918, 3990, 4063, 4135, 4206, 4278, 4349, 4420, 4491, 4561, 4631, 4701, 4770, 4840, 4909, 4977, 5046, 5113, 5181, 5249, 5316, 5382, 5449, 5515, 5580, 5646, 5711, 5775, 5839, 5903, 5967, 6030, 6093, 6155, 6217, 6279, 6340, 6401, 6461, 6521, 6581, 6640, 6699, 6758, 6815, 6873, 6930, 6987, 7043, 7099, 7154, 7209, 7264, 7318, 7371, 7424, 7477, 7529, 7581, 7632, 7683, 7733, 7783, 7832, 7881, 7930, 7977, 8025, 8072, 8118, 8164, 8209, 8254, 8298, 8342, 8385, 8428, 8470, 8512, 8553, 8594, 8634, 8673, 8712, 8751, 8789, 8826, 8863, 8899, 8935, 8970, 9005, 9039, 9072, 9105, 9138, 9169, 9201, 9231, 9261, 9291, 9320, 9348, 9376, 9403, 9429, 9455, 9481, 9506, 9530, 9554, 9577, 9599, 9621, 9642, 9663, 9683, 9702, 9721, 9739, 9757, 9774, 9790, 9806, 9821, 9836, 9850, 9863, 9876, 9888, 9899, 9910, 9920, 9930, 9939, 9947, 9955, 9962, 9969, 9975, 9980, 9985, 9989, 9992, 9995, 9997, 9999, 10000, 10000};

// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin(float a)
{
  if (a < _PI_2)
  {
    // return sine_array[(int)(199.0f*( a / (_PI/2.0)))];
    // return sine_array[(int)(126.6873f* a)];           // float array optimized
    return 0.0001f * sine_array[_round(126.6873f * a)]; // int array optimized
  }
  else if (a < _PI)
  {
    // return sine_array[(int)(199.0f*(1.0f - (a-_PI/2.0) / (_PI/2.0)))];
    // return sine_array[398 - (int)(126.6873f*a)];          // float array optimized
    return 0.0001f * sine_array[398 - _round(126.6873f * a)]; // int array optimized
  }
  else if (a < _3PI_2)
  {
    // return -sine_array[(int)(199.0f*((a - _PI) / (_PI/2.0)))];
    // return -sine_array[-398 + (int)(126.6873f*a)];           // float array optimized
    return -0.0001f * sine_array[-398 + _round(126.6873f * a)]; // int array optimized
  }
  else
  {
    // return -sine_array[(int)(199.0f*(1.0f - (a - 3*_PI/2) / (_PI/2.0)))];
    // return -sine_array[796 - (int)(126.6873f*a)];           // float array optimized
    return -0.0001f * sine_array[796 - _round(126.6873f * a)]; // int array optimized
  }
}

// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a)
{
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}

// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs)
{
  return (shaft_angle * pole_pairs);
}

// square root approximation function using
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox(float number)
{ // low in fat
  long i;
  float y;
  // float x;
  // const float f = 1.5F; // better precision

  // x = number * 0.5F;
  y = number;
  i = *(long *)&y;
  i = 0x5f375a86 - (i >> 1);
  y = *(float *)&i;
  // y = y * ( f - ( x * y * y ) ); // better precision
  return number * y;
}

void spwm(float Uq, float Ud, float angle_el, float voltage_limit)
{
  float _ca, _sa;
  float Ualpha, Ubeta;
  float center;
  angle_el = _normalizeAngle(angle_el);
  _ca = _cos(angle_el);
  _sa = _sin(angle_el);
  // Inverse park transform
  Ualpha = _ca * Ud - _sa * Uq; // -sin(angle) * Uq;
  Ubeta = _sa * Ud + _ca * Uq;  //  cos(angle) * Uq;

  // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
  center = voltage_limit / 2;
  // Clarke transform
  float Ua = Ualpha + center;
  float Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta + center;
  float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta + center;

  printf("%f,%f,%f\n", Ua, Ub, Uc);
}

void svpwm(float Uq, float Ud, float angle_el, float voltage_limit)
{
  float center;
  int sector;
  float _ca, _sa;
  float Uout;
  // a bit of optitmisation
  if (Ud)
  { // only if Ud and Uq set
    // _sqrt is an approx of sqrt (3-4% error)
    Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_limit;
    // angle normalisation in between 0 and 2pi
    // only necessary if using _sin and _cos - approximation functions
    angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
  }
  else
  { // only Uq available - no need for atan2 and sqrt
    Uout = Uq / voltage_limit;
    // angle normalisation in between 0 and 2pi
    // only necessary if using _sin and _cos - approximation functions
    angle_el = _normalizeAngle(angle_el + _PI_2);
  }
  // find the sector we are in currently
  sector = floor(angle_el / _PI_3) + 1;
  
  // calculate the duty cycles
  float T1 = _SQRT3 * _sin(sector * _PI_3 - angle_el) * Uout;
  float T2 = _SQRT3 * _sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
  // two versions possible
  float T0 = 0;     // pulled to 0 - better for low power supply voltage
  T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2
// printf("Uout=%f,sector=%d,T1=%f,T2=%f,T0=%f\n",Uout,sector,T1,T2,T0);
  // calculate the duty cycles(times)
  float Ta, Tb, Tc;
  switch (sector)
  {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    // possible error state
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  // calculate the phase voltages and center
  float Ua = Ta * voltage_limit;
  float Ub = Tb * voltage_limit;
  float Uc = Tc * voltage_limit;
  printf("%f,%f,%f\n", Ua, Ub, Uc);
}
