#ifndef INCLUDED_HELPERS
#define INCLUDED_HELPERS

// Helper functions
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]
void SlideLeft(double* arr, unsigned int size)
{
  memmove(arr, arr + 1, sizeof(double) * size);
  arr[size - 1] = 0.0;

  return;
}

//Sliiiiiiiiiiiiide to the right
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(double* arr, unsigned int size)
{
  memmove(arr + 1, arr, sizeof(double) * size);
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

// print the contents of a given array
void printArray(double* arr, unsigned int size)
{
  Serial.print("[");
  for(uint i = 0; i < size-1; ++i)
  {
    Serial.print(arr[i]);
    Serial.print(", ");    
  }
  Serial.print(arr[size-1]);

  Serial.print("]\n");

  return;
}

#endif //INCLUDED_HELPERS
