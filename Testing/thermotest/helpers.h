#ifndef INCLUDED_ARRAY_HELPERS
#define INCLUDED_ARRAY_HELPERS

// Helper functions
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]
void SlideLeft(double* arr, unsigned int size);

//Sliiiiiiiiiiiiide to the right
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(double* arr, unsigned int size);

// (...chris-cross...?)
void Prepend(double val, double* arr, unsigned int size);

// print the contents of a given array
void printArray(double* arr, unsigned int size);

#endif //INCLUDED_ARRAY_HELPERS
