#pragma once

#include "array.hh"

#include <stddef.h>

template<typename T, size_t Len>
struct RingBuffer
{
  Array<T, Len> _buf = {};
  size_t _index = 0;

  size_t cap() const
  {
    return Len;
  }

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

  template<typename S = T>
  S sum() const
  {
    S s = 0;
    for (auto x : _buf)
      s += x;

    return s;
  }
};
