/*
 * helper_functions.h
 *
 *  Created on: Oct 8, 2024
 *      Author: JDT
 */

#ifndef SRC_SENSORS_HELPER_FUNCTIONS_H_
#define SRC_SENSORS_HELPER_FUNCTIONS_H_

#include <stdbool.h>

bool FindAdjacent(const int* points, int valueToFind, int *index1, int *index2, int* y1, int* y2, const int length);
int linear(int x, int x0, int x1, int y0, int y1);

#endif /* SRC_SENSORS_HELPER_FUNCTIONS_H_ */
