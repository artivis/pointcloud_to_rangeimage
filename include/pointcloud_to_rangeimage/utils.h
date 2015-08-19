#ifndef POINTCLOUD_TO_RANGEIMAGE_UTILS_H_
#define POINTCLOUD_TO_RANGEIMAGE_UTILS_H_

#include <limits>
#include <cstring>
#include <cmath>

void getFalseColorFromRange(unsigned short value, unsigned char& r, unsigned char& g, unsigned char& b)
{
  unsigned char data[sizeof(unsigned short)];

  std::memcpy(data, &value, sizeof value);

  r = data[0];
  g = data[1];
  b = 0;
}

void getFalseColorFromRange2(unsigned short value, unsigned char& r, unsigned char& g, unsigned char& b)
{
  float intpart, fractpart;
  float maxuchar = static_cast<float>(std::numeric_limits<unsigned char>::max());

  fractpart = std::modf(value/maxuchar, &intpart);

  // keep integer part of division
  r = intpart;

  // deal with fractional part
  fractpart = std::modf((fractpart*100.f)/maxuchar, &intpart);
  g = intpart;

  fractpart = std::modf((fractpart*100.f)/maxuchar, &intpart);
  b = intpart;
}

void getRangeFromFalseColor(unsigned char r, unsigned char g, unsigned char b, unsigned short& value)
{
  unsigned char data[sizeof(unsigned short)];

  data[0] = r;
  data[1] = g;

  std::memcpy(&value, data, sizeof data);
}

void getRangeFromFalseColor2(unsigned char r, unsigned char g, unsigned char b, unsigned short& value)
{
  float maxuchar = static_cast<float>(std::numeric_limits<unsigned char>::max());

  float sum = static_cast<float>(r)*maxuchar;

  sum += static_cast<float>(g) / 100.f * maxuchar;
  sum += static_cast<float>(b) / 100.f * maxuchar;

  value = static_cast<unsigned short>(sum);
}

#endif // POINTCLOUD_TO_RANGEIMAGE_UTILS_H_
