// 主机(PC)编译用的 Arduino 垫片，仅提供 constrain / fabsf / logf
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
