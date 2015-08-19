#ifndef POINTCLOUD_TO_RANGEIMAGE_UTILS_H_
#define POINTCLOUD_TO_RANGEIMAGE_UTILS_H_

#include <limits>
#include <cstring>
#include <cmath>

void getColorFromRange(float value, unsigned char& r, unsigned char& g, unsigned char& b)
{
  if (std::isinf(value))
  {
    if (value > 0.0f)
    {
      r = 150; g = 150; b = 200; // INFINITY
      return;
    }
    r = 150; g = 200; b = 150; // -INFINITY
    return;
  }
  if (!std::isfinite (value))
  {
    r = 200; g = 150; b = 150; // -INFINITY
    return;
  }
  r = g = b = 0;
  value *= 10;
  if (value <= 1.0)
  { // black -> purple
    b = static_cast<unsigned char> (static_cast<long int>(round((value*200))));
    r = static_cast<unsigned char> (static_cast<long int>(round((value*120))));
  }
  else if (value <= 2.0)
  { // purple -> blue
    b = static_cast<unsigned char> (200 + static_cast<long int>(round((value-1.0)*55)));
    r = static_cast<unsigned char> (120 - static_cast<long int>(round((value-1.0)*120)));
  }
  else if (value <= 3.0)
  { // blue -> turquoise
    b = static_cast<unsigned char> (255 - static_cast<long int>(round((value-2.0)*55)));
    g = static_cast<unsigned char> (static_cast<long int>(round((value-2.0)*200)));
  }
  else if (value <= 4.0)
  { // turquoise -> green
    b = static_cast<unsigned char> (200 - static_cast<long int>(round((value-3.0)*200)));
    g = static_cast<unsigned char> (200 + static_cast<long int>(round((value-3.0)*55)));
  }
  else if (value <= 5.0)
  { // green -> greyish green
    g = static_cast<unsigned char> (255 - static_cast<long int>(round((value-4.0)*100)));
    r = static_cast<unsigned char> (static_cast<long int>(round((value-4.0)*120)));
  }
  else if (value <= 6.0)
  { // greyish green -> red
    r = static_cast<unsigned char> (100 + static_cast<long int>(round((value-5.0)*155)));
    g = static_cast<unsigned char> (120 - static_cast<long int>(round((value-5.0)*120)));
    b = static_cast<unsigned char> (120 - static_cast<long int>(round((value-5.0)*120)));
  }
  else if (value <= 7.0)
  { // red -> yellow
    r = 255;
    g = static_cast<unsigned char> (static_cast<long int>(round((value-6.0)*255)));
  }
  else
  { // yellow -> white
    r = 255;
    g = 255;
    b = static_cast<unsigned char> (static_cast<long int>(round((value-7.0)*255.0/3.0)));
  }
}

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

void getRangeFromColor(unsigned char r, unsigned char g, unsigned char b, float& value)
{

  value = std::numeric_limits<float>::infinity();

//  if (r = 150 && g = 150 && b = 200)
//  {
//    value = std::numeric_limits<float>::infinity();
//    if (value > 0.0f)
//    {
//      r = 150; g = 150; b = 200; // INFINITY
//      return;
//    }
//    r = 150; g = 200; b = 150; // -INFINITY
//    return;
//  }

//  if (!std::isfinite (value))
//  {
//    r = 200; g = 150; b = 150; // -INFINITY
//    return;
//  }

//  r = g = b = 0;
//  value *= 10;
//  if (value <= 1.0)
//  { // black -> purple
//    b = static_cast<unsigned char> (static_cast<long int>(round((value*200))));
//    r = static_cast<unsigned char> (static_cast<long int>(round((value*120))));
//  }
//  else if (value <= 2.0)
//  { // purple -> blue
//    b = static_cast<unsigned char> (200 + static_cast<long int>(round((value-1.0)*55)));
//    r = static_cast<unsigned char> (120 - static_cast<long int>(round((value-1.0)*120)));
//  }
//  else if (value <= 3.0)
//  { // blue -> turquoise
//    b = static_cast<unsigned char> (255 - static_cast<long int>(round((value-2.0)*55)));
//    g = static_cast<unsigned char> (static_cast<long int>(round((value-2.0)*200)));
//  }
//  else if (value <= 4.0)
//  { // turquoise -> green
//    b = static_cast<unsigned char> (200 - static_cast<long int>(round((value-3.0)*200)));
//    g = static_cast<unsigned char> (200 + static_cast<long int>(round((value-3.0)*55)));
//  }
//  else if (value <= 5.0)
//  { // green -> greyish green
//    g = static_cast<unsigned char> (255 - static_cast<long int>(round((value-4.0)*100)));
//    r = static_cast<unsigned char> (static_cast<long int>(round((value-4.0)*120)));
//  }
//  else if (value <= 6.0)
//  { // greyish green -> red
//    r = static_cast<unsigned char> (100 + static_cast<long int>(round((value-5.0)*155)));
//    g = static_cast<unsigned char> (120 - static_cast<long int>(round((value-5.0)*120)));
//    b = static_cast<unsigned char> (120 - static_cast<long int>(round((value-5.0)*120)));
//  }
//  else if (value <= 7.0)
//  { // red -> yellow
//    r = 255;
//    g = static_cast<unsigned char> (static_cast<long int>(round((value-6.0)*255)));
//  }
//  else
//  { // yellow -> white
//    r = 255;
//    g = 255;
//    b = static_cast<unsigned char> (static_cast<long int>(round((value-7.0)*255.0/3.0)));
//  }
}

#endif // POINTCLOUD_TO_RANGEIMAGE_UTILS_H_
