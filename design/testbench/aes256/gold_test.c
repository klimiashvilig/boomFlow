/*  
*   Byte-oriented AES-256 implementation.
*   All lookup tables replaced with 'on the fly' calculations. 
*
*   Copyright (c) 2007 Ilya O. Levin, http://www.literatecode.com
*
*   Permission to use, copy, modify, and distribute this software for any
*   purpose with or without fee is hereby granted, provided that the above
*   copyright notice and this permission notice appear in all copies.
*
*   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
*   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
*   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
*   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
*   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
*   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#define  _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include "aes256.h"

#define NUM_TESTS 2

#define DUMP(s, i, buf, sz)  {printf(s);                   \
                              for (i = 0; i < (sz);i++)    \
                                  printf("%02x ", buf[i]); \
                              printf("\n");}

uint8_t hex2byte(char ch)
{
  if (ch >= '0' && ch <= '9')
    return ch - '0';
  else if (ch >= 'A' && ch <= 'F')
    return 10 + ch - 'A';
  else
    return 10 + ch - 'a';
}

int readbit(FILE *fp, uint8_t *arr, size_t length){
  size_t len;
  char *line = malloc(length);

  size_t bytes_read;
  if ((bytes_read = getline(&line, &len, fp)) == -1) 
    return -1;

  if (len < length - 2) {
    printf("%s\n", "There was an error in the input file.");
    return -1;
  }

  //uint8_t hex_arr[32];
  for (unsigned i = 0; i < length; i += 2)
    arr[i / 2] = hex2byte(line[i]) * 16 + hex2byte(line[i + 1]);
  return 0;
}

int main (int argc, char *argv[])
{
    aes256_context ctx; 
    uint8_t key[32];
    uint8_t buf[16], out[16];
    FILE *in_fp, *key_fp, *out_fp;

    in_fp = fopen("testbench/inputs.mem", "r");
    key_fp = fopen("testbench/key.mem", "r");
    out_fp = fopen("output.txt", "r");
    if (!in_fp) {
      perror("in file doesn't exist");
      return -1;
    }
    if (!key_fp ) {
      perror("key file doesn't exist");
      return -1;
    }
    if (!out_fp) {
      perror("out file doesn't exist");
      return -1;
    }
    int errors = 0; 

    while(1){
        if(readbit(in_fp, buf, 32) == -1)
          break; 
        readbit(key_fp, key, 64);
        readbit(out_fp, out, 32);

        aes256_init(&ctx, key);
        aes256_encrypt_ecb(&ctx, buf);
        // DUMP("out: ", i, buf, sizeof(buf));

        for(int j = 0; j< 16; j++){
            if(buf[j] != out[j]){
              //printf("%02x %02x\n", buf[j], out[j]);
              errors ++;
            }
        }
        aes256_init(&ctx, key);
        aes256_decrypt_ecb(&ctx, buf);
        // DUMP("dec: ", i, buf, sizeof(buf));
        aes256_done(&ctx);
    }
    if (errors > 0)
      printf("FAIL\n");
    else
      printf("PASS\n");

} /* main */
