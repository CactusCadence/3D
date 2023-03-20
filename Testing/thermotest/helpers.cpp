#include "helpers.h"

#include <cstring>
#include <limits>

//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]
void SlideLeft(double* arr, unsigned int size)
{
  memmove(arr, arr + 1, sizeof(double) * (size-1));
  arr[size - 1] = 0.0;

  return;
}

//Sliiiiiiiiiiiiide to the right
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(double* arr, unsigned int size)
{
  memmove(arr + 1, arr, sizeof(double) * (size-1));
  arr[0] = 0.0;

  return;
}

// (...chris-cross...?)
void Prepend(double val, double* arr, unsigned int size)
{
  SlideRight(arr, size);
  arr[0] = val;

  return;
}
