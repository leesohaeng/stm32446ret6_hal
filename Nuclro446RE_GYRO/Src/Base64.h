/*
 * Base64.h
 *
 *  Created on: 2016. 10. 14.
 *      Author: sohae
 */

#ifndef BASE64_H
#define BASE64_h

unsigned char * base64_encode(const unsigned char *src, int len, int *out_len);
unsigned char * base64_decode(const unsigned char *src, int len, int *out_len);

#endif /* BASE64_H */
