
/*********************************************************************************
 *      Copyright:  (C) 2017 Yang Zheng<yz2012ww@gmail.com>  
 *                  All rights reserved.
 *
 *       Filename:  base64.c
 *    Description:  This file 
 *                 
 *        Version:  1.0.0(08/17/2017~)
 *         Author:  Yang Zheng <yz2012ww@gmail.com>
 *      ChangeLog:  1, Release initial version on "08/17/2017 02:09:12 PM"
 *                 
 ********************************************************************************/
#include "base64.h"
 
static const char base64char[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";

//±àÂë unsigned char --> char
char *base64_encode(const unsigned char *indata, char *outdata, int inlength)
{
    int i, j;
    unsigned char current;

    for ( i = 0, j = 0 ; i < inlength ; i += 3 )
    {
        current = (indata[i] >> 2) ;
        current &= (unsigned char)0x3F;
        outdata[j++] = base64char[(int)current];

        current = ( (unsigned char)(indata[i] << 4 ) ) & ( (unsigned char)0x30 ) ;
        if ( i + 1 >= inlength )
        {
            outdata[j++] = base64char[(int)current];
            outdata[j++] = '=';
            outdata[j++] = '=';
            break;
        }
        current |= ( (unsigned char)(indata[i+1] >> 4) ) & ( (unsigned char) 0x0F );
        outdata[j++] = base64char[(int)current];

        current = ( (unsigned char)(indata[i+1] << 2) ) & ( (unsigned char)0x3C ) ;
        if ( i + 2 >= inlength )
        {
            outdata[j++] = base64char[(int)current];
            outdata[j++] = '=';
            break;
        }
        current |= ( (unsigned char)(indata[i+2] >> 6) ) & ( (unsigned char) 0x03 );
        outdata[j++] = base64char[(int)current];

        current = ( (unsigned char)indata[i+2] ) & ( (unsigned char)0x3F ) ;
        outdata[j++] = base64char[(int)current];
    }
    outdata[j] = '\0';
    return outdata;
}

//½âÂë char --> unsigned char
int base64_decode(const char *indata, unsigned char *outdata, int inlength)
{
    int i, j;
    unsigned char k;
    unsigned char temp[4];
    for ( i = 0, j = 0; indata[i] != '\0' && i < inlength; i += 4 )
    {
        memset( temp, 0xFF, sizeof(temp) );
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == indata[i] )
                temp[0]= k;
        }
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == indata[i+1] )
                temp[1]= k;
        }
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == indata[i+2] )
                temp[2]= k;
        }
        for ( k = 0 ; k < 64 ; k ++ )
        {
            if ( base64char[k] == indata[i+3] )
                temp[3]= k;
        }

        outdata[j++] = ((unsigned char)(((unsigned char)(temp[0] << 2))&0xFC)) |
                ((unsigned char)((unsigned char)(temp[1]>>4)&0x03));
        if ( indata[i+2] == '=' )
            break;

        outdata[j++] = ((unsigned char)(((unsigned char)(temp[1] << 4))&0xF0)) |
                ((unsigned char)((unsigned char)(temp[2]>>2)&0x0F));
        if ( indata[i+3] == '=' )
            break;

        outdata[j++] = ((unsigned char)(((unsigned char)(temp[2] << 6))&0xF0)) |
                ((unsigned char)(temp[3]&0x3F));
    }
    return j;
}

