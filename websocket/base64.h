
/********************************************************************************
 *      Copyright:  (C) 2017 Yang Zheng<yz2012ww@gmail.com>
 *                  All rights reserved.
 *
 *       Filename:  base64.h
 *    Description:  This head file 
 *
 *        Version:  1.0.0(08/17/2017~)
 *         Author:  Yang Zheng <yz2012ww@gmail.com>
 *      ChangeLog:  1, Release initial version on "08/17/2017 02:11:15 PM"
 *                 
 ********************************************************************************/
#ifndef __BASE64_H__
#define __BASE64_H__
 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
 
char * base64_encode(const unsigned char *indata, char *outdata, int inlength);
int base64_decode(const char *indata, unsigned char *outdata, int inlength);
#endif
