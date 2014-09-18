#ifndef __PROCESS_H__
#define __PROCESS_H__

#include "stdlib.h"
#include "unistd.h"
#include "stdio.h"
#include "global.h"

void* data_decode(void* argv);
void* processB(void* argv);
void* send_command(void* argv);

#endif