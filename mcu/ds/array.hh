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

  T const& operator[](size_t index) const
  {
    return _data[index];
  }

  T* begin()
  {
    return _data;
  }

  T const* begin() const
  {
    return _data;
  }

  T* end()
  {
    return &_data[Len];
  }

  T const* end() const
  {
    return &_data[Len];
  }
};
