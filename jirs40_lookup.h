/*
 * jirs40_loopup.h
 *
 *  Created on: Oct 8, 2024
 *      Author: JDT
 */

#ifndef SRC_SENSORS_JIRS40_LOOKUP_H_
#define SRC_SENSORS_JIRS40_LOOKUP_H_


#define AMBIENT_START 0
#define AMBIENT_END 50
#define OBJECT_START 0
#define OBJECT_END  75

#define NO_AMBIENT (AMBIENT_END - AMBIENT_START) + 1
#define NO_OBJECT (OBJECT_END - OBJECT_START) + 1

extern const int jirs40_lookup[NO_AMBIENT][NO_OBJECT];
#endif /* SRC_SENSORS_JIRS40_LOOKUP_H_ */
