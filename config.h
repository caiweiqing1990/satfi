#ifndef __MY_CONFIG_H__
#define __MY_CONFIG_H__

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<assert.h>
#include<ctype.h>
#include<errno.h>

#define KEYVALLEN 1024

void GetIniKeyString(char *appName, char *keyName, char *profile, char *keyVal);
int  GetIniKeyInt(char *appName, char *keyName, char *profile);
int  SetIniString(char *appName, char *keyName, char *profile, char *keyValOld, char *keyValNew);
int SetKeyString(char *appName, char *keyName, char *profile, char *keyValOld, char *keyValNew);
int SetKeyInt(char *appName, char *keyName, char *profile, int Value);

#endif
