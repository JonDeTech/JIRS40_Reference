/*
 * helper_functions.c
 *
 *  Created on: Oct 8, 2024
 *      Author: JDT
 */
#include "helper_functions.h"
/*
 * Function to iterate through a list, find the to closest points to valueToFind.
 */
bool FindAdjacent(const int* points, int valueToFind, int *index1, int *index2, int* y1, int* y2, const int length)
{
	bool result = false;
	*index1 = 0;
	*index2 = 0;
	*y1 = *y2 = 0;

	for (int i = 0; i < length; ++i)
	{
		int currentValue = points[i];
		if (currentValue >= valueToFind) //We passed the value we are looking for
		{
			*index2 = i;
			*y2 = points[i];

			if (i != 0)     //Is there a point before this one?
			{
				*index1 = i - 1;
				*y1 = points[i-1];
				result = true;
				break;
			}
			else            //If not use the 2 first points in the list
			{
				*index2 = i+1;
				*y2 = points[i+1];
				*index1 = i;
				*y1 = points[i];
				result = true;
				break;
			}
		}
	}
	if(*y1 == 0 && *y2 == 0) //We couldnt find any points, grab the 2 last points
	{
		*index2 = length - 1;
		*y2 = points[*index2];
		*index1 = *index2-1;
		*y1 = points[*index1];
		result = true;
	}

	return result;
}

/*
 * Function to perform linear interpolation
 */
int linear(int x, int x0, int x1, int y0, int y1)
{
	if ((x1 - x0) == 0)
	{
		return (y0 + y1) / 2;
	}
	if(x < x0)
	{
		//We are outside the interpolation area
		return y0;
	}
	if (x> x1)
	{
		//We are outside the interpolation area
		return y1;
	}
	return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}
