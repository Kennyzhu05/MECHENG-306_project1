#ifndef BOUNDARY_TESTING_H
#define BOUNDARY_TESTING_H

// Call once from setup() to initialise and start the two boundary tests.
void beginBoundaryTest();

// Call continuously from loop(). This function is non-blocking.
void updateBoundaryTest();

bool isBoundaryTestComplete();
bool hasBoundaryTestFault();

#endif
