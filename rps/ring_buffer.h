#pragma once

#include "array.h"

#include <stddef.h>
#include <stdint.h>

template<typename T, size_t Len>
struct RingBuffer
{
  Array<T, Len> _buf = {0};
  size_t _index = 0;

  void shift()
  {
    _index = (_index + 1) % _buf.len();
  }

  T& current()
  {
    return _buf[_index];
  }

  void push(T value)
  {
    _buf[_index] = value;
    shift();
  }

  T sum() const
  {
    T s = 0;
    for (auto x : _buf)
      s += x;

    return s;
  }
};
