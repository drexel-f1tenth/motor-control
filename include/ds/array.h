#pragma once

#include <stddef.h>

template<typename T, size_t Len>
struct Array
{
  T _data[Len];

  constexpr size_t len()
  {
    return Len;
  }

  T& operator[](size_t index)
  {
    return _data[index];
  }

  T* begin() const
  {
    return _data;
  }

  T* end() const
  {
    return &_data[Len];
  }
};
