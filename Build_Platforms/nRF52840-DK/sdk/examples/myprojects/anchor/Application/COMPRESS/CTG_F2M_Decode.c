/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
*       (c) 2017 - 2024     SEGGER Microcontroller GmbH              *
*                                                                    *
*       www.segger.com     Support: www.segger.com/ticket            *
*                                                                    *
**********************************************************************
*                                                                    *
*       emCompress-ToGo * Compression library                        *
*                                                                    *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product.                          *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       emCompress-ToGo version: V3.40.1                             *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : CTG_F2M_Decode.c
Purpose : Decompress function to memory.
*/

#include "CTG.h"
#include <stdio.h>

static const U8 _aInput[] = {
  0xA2, 0x24, 0x8C, 0x18, 0x31, 0x65, 0xE4, 0xDC, 0x79, 0x33, 0x66, 0x4D,
  0x9E, 0x06, 0x0A, 0x40, 0x80, 0x10, 0x92, 0x05, 0x04, 0x93, 0x22, 0x57,
  0x92, 0x4C, 0x01, 0x31, 0x24, 0x88, 0x14, 0x29, 0x4F, 0x98, 0x30, 0x45,
  0x73, 0x09, 0x27, 0xA8, 0xDC, 0x09, 0x33, 0x07, 0x84, 0x18, 0x39, 0x69,
  0xD8, 0xB0, 0x49, 0x73, 0x86, 0x05, 0x88, 0x30, 0x6E, 0xC8, 0x80, 0xA0,
  0x83, 0xA6, 0x0C, 0x88, 0xB9, 0xA4, 0x13, 0x93, 0x97, 0xA8, 0x37, 0x76,
  0xCA, 0xCC, 0xDD, 0xF8, 0x05, 0x10, 0x49, 0x8B, 0xD6, 0x99, 0x3C, 0x72,
  0x11, 0xDF, 0x88, 0x33, 0x69, 0xDA, 0x88, 0x61, 0x4B, 0x54, 0x1A, 0xB7,
  0x9F, 0x0B, 0x87, 0x30, 0xA5, 0x93, 0x1D, 0x41, 0x21, 0x01, 0x44, 0x5B,
  0x84, 0xE6, 0x32, 0xEC, 0x2E, 0xC4, 0xA3, 0x3A, 0x25, 0x31, 0x6F, 0xE4,
  0xBC, 0x39, 0x3B, 0x16, 0x58, 0x86, 0x34, 0x04, 0x6D, 0x8A, 0xDA, 0xBC,
  0x69, 0x4B, 0x88, 0x9C, 0xB0, 0x44, 0x43, 0xDF, 0x9B, 0x3A, 0x74, 0xCE,
  0x92, 0x0C, 0x05, 0x2E, 0xD7, 0x08, 0x11, 0x42, 0xCA, 0x42, 0x67, 0xAF,
  0x8F, 0x2B, 0x4C, 0x35, 0xDA, 0xC2, 0xCA, 0x9C, 0x37, 0x6E, 0x42, 0x86,
  0x34, 0x50, 0x55, 0x4E, 0x4D, 0x98, 0xBB, 0x8C, 0x67, 0x27, 0x1C, 0x42,
  0x33, 0x75, 0x29, 0xCB, 0xC4, 0x4D, 0x1A, 0xC3, 0xB6, 0x14, 0x19, 0x4B,
  0xC6, 0x18, 0xB4, 0x0E, 0x0E, 0x24, 0xA4, 0x8E, 0x18, 0xB5, 0x44, 0x3A,
  0x47, 0x0E, 0xD9, 0x2C, 0xDB, 0x1C, 0x34, 0x75, 0xDC, 0x06, 0x52, 0x62,
  0x46, 0x4E, 0x5D, 0x1A, 0x4C, 0x1F, 0x61, 0x40, 0xA8, 0xEA, 0x50, 0x37,
  0xC7, 0xED, 0x39, 0x10, 0xB9, 0x62, 0x20, 0x95, 0x82, 0x37, 0x6F, 0xD6,
  0x80, 0x40, 0x93, 0x16, 0x35, 0xBB, 0xD4, 0x0A, 0x4E, 0x58, 0xCA, 0x36,
  0x97, 0x6A, 0x42, 0xE4, 0x4C, 0x10, 0xBA, 0x59, 0xB3, 0x33, 0xE4, 0x81,
  0x29, 0x35, 0x39, 0xCB, 0x54, 0x7C, 0x30, 0x29, 0x6B, 0x8B, 0x14, 0xDE,
  0x55, 0xCD, 0xCC, 0x5B, 0x32, 0xE7, 0xAC, 0x90, 0x73, 0x06, 0x0D, 0x9D,
  0xCB, 0x6C, 0x99, 0xF2, 0x96, 0xF6, 0x63, 0x8B, 0xD4, 0x96, 0xFD, 0xB3,
  0x49, 0xBD, 0xD6, 0x18, 0xAA, 0x90, 0x47, 0x97, 0x48, 0xA2, 0x11, 0x2A,
  0x1B, 0x68, 0x97, 0xAC, 0x84, 0x77, 0x52, 0xE9, 0x35, 0xCC, 0x87, 0x96,
  0x7B, 0x12, 0x6F, 0x97, 0xEA, 0xDB, 0x64, 0x1A, 0xA9, 0x4C, 0x1D, 0x33,
  0x66, 0xA9, 0x06, 0xDA, 0x50, 0xFD, 0x64, 0x6F, 0x78, 0xE9, 0xBC, 0xEB,
  0x77, 0xA0, 0xA3, 0xDC, 0xD5, 0xE3, 0x40, 0x94, 0xC9, 0x4B, 0x09, 0x31,
  0x20, 0xB3, 0xD4, 0x13, 0xDF, 0xA4, 0xA4, 0x4E, 0x19, 0x6A, 0x2C, 0xCA,
  0x30, 0x0D, 0x17, 0x6C, 0xA1, 0xD4, 0x1A, 0xA1, 0xC5, 0xBF, 0x65, 0xF3,
  0xED, 0xA4, 0x1A, 0x6C, 0xCE, 0x32, 0x17, 0xF1, 0x40, 0x4E, 0x57, 0x0F,
  0x31, 0x75, 0xE4, 0xFA, 0x05, 0x2D, 0xA8, 0x2D, 0x9C, 0xEB, 0x42, 0xA0,
  0xA6, 0x4B, 0xC2, 0x13, 0xB7, 0xB3, 0x64, 0xD6, 0x84, 0x00, 0x19, 0x27,
  0xA7, 0xF9, 0x27, 0xA5, 0xF9, 0x35, 0x9F, 0x1D, 0xD8, 0xD0, 0xB6, 0x56,
  0x18, 0x09, 0x8B, 0xFF, 0xFA, 0x23, 0xDC, 0xC2, 0x28, 0x26, 0x94, 0x96,
  0xBA, 0x52, 0x9A, 0x96, 0x9A, 0x31, 0xB9, 0x52, 0xAB, 0x11, 0x43, 0x97,
  0x59, 0x74, 0x25, 0x9B, 0x89, 0xAC, 0x4C, 0x23, 0x7F, 0x99, 0xC9, 0xA4,
  0x82, 0x50, 0x88, 0x64, 0xC8, 0xA7, 0x4B, 0xCF, 0xC0, 0x59, 0x08, 0xC5,
  0x56, 0xE0, 0x52, 0x6A, 0x0B, 0x25, 0x56, 0x0E, 0xF3, 0x67, 0xAE, 0x8B,
  0xF8, 0x36, 0x16, 0x79, 0xC3, 0xF4, 0xF1, 0x84, 0xED, 0x33, 0x1B, 0x5B,
  0xC4, 0xCF, 0x90, 0x87, 0xA1, 0x33, 0x70, 0xBC, 0x74, 0x1F, 0x24, 0x8C,
  0x9C, 0x36, 0x73, 0x7F, 0x46, 0xD9, 0x48, 0xD1, 0x67, 0x37, 0x62, 0xDE,
  0xE4, 0xB5, 0x13, 0x4F, 0x7D, 0x8E, 0xEC, 0x52, 0xBB, 0xE7, 0x40, 0x26,
  0x2C, 0x82, 0x2C, 0x86, 0x31, 0xB6, 0xF8, 0x04, 0xDA, 0x0C, 0x56, 0xD0,
  0x07, 0x59, 0x56, 0x62, 0x2C, 0x7C, 0xC9, 0xA1, 0x3B, 0xC3, 0xDC, 0x71,
  0xF4, 0x4C, 0xB2, 0x2A, 0xEF, 0x1B, 0x19, 0x14, 0xF8, 0xE2, 0x55, 0x55
};

static U8 _aOutput[sizeof(_aInput)*2];

typedef struct {  /*emDoc #1*/
  const U8 * pInput;
  unsigned   InputLen;
  unsigned   Cursor;
} CTG_RD_CONTEXT;

static int _Rd(U8 *pData, unsigned DataLen, void *pRdCtx) {  /*emDoc #2*/
  CTG_RD_CONTEXT * pCtx;
  unsigned         N;
  //
  pCtx = (CTG_RD_CONTEXT *)pRdCtx;  /*emDoc #3*/
  for (N = 0; N < DataLen; ++N) {   /*emDoc #4*/
    if (pCtx->Cursor >= pCtx->InputLen) {
      break;
    }
    *pData++ = pCtx->pInput[pCtx->Cursor++];
  }
  return N;
}

void MainTask(void);
void MainTask(void) {
  CTG_RD_CONTEXT RdCtx;
  int            DecodeLen;
  //
  RdCtx.pInput   = &_aInput[0];  /*emDoc #5*/
  RdCtx.InputLen = sizeof(_aInput);
  RdCtx.Cursor   = 0;
  DecodeLen = CTG_DecompressF2M(_Rd, &RdCtx, /*emDoc #6*/
                                &_aOutput[0], sizeof(_aOutput));
  if (DecodeLen < 0) {
    printf("Decompression error.\n");
  } else {
    printf("Decompressed %u to %d bytes.\n", sizeof(_aInput), DecodeLen);
  }
}
