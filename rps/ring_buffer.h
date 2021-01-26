
#include <stddef.h>
#include <stdint.h>

template<typename T, size_t Len>
struct RingBuffer
{
  T data[Len] = {0};
  size_t index = 0;

  void push(T value)
  {
    data[index] = value;
    index = (index + 1) % Len;
  }
};
