/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 2007 - 2024    SEGGER Microcontroller GmbH               *
*                                                                    *
*       www.segger.com     Support: www.segger.com/ticket            *
*                                                                    *
**********************************************************************
*                                                                    *
*       emNet * TCP/IP stack for embedded applications               *
*                                                                    *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product for in-house use.         *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       emNet version: V3.56.0                                       *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : IPInf.c
Purpose : Automatically created from html\IPInf.htm using Bin2C.exe
*/

#include "IPInf.h"

const unsigned char ipinf_file[954] = {
  0x3C, 0x21, 0x44, 0x4F, 0x43, 0x54, 0x59, 0x50, 0x45, 0x20, 0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0D,
  0x0A, 0x3C, 0x68, 0x74, 0x6D, 0x6C, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x3C, 0x68, 0x65, 0x61, 0x64,
  0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x74, 0x69, 0x74, 0x6C, 0x65, 0x3E, 0x65, 0x6D,
  0x4E, 0x65, 0x74, 0x20, 0x73, 0x74, 0x61, 0x74, 0x69, 0x73, 0x74, 0x69, 0x63, 0x73, 0x20, 0x28,
  0x6D, 0x65, 0x74, 0x61, 0x20, 0x72, 0x65, 0x66, 0x72, 0x65, 0x73, 0x68, 0x29, 0x3C, 0x2F, 0x74,
  0x69, 0x74, 0x6C, 0x65, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x6D, 0x65, 0x74, 0x61,
  0x20, 0x68, 0x74, 0x74, 0x70, 0x2D, 0x65, 0x71, 0x75, 0x69, 0x76, 0x3D, 0x22, 0x72, 0x65, 0x66,
  0x72, 0x65, 0x73, 0x68, 0x22, 0x20, 0x63, 0x6F, 0x6E, 0x74, 0x65, 0x6E, 0x74, 0x3D, 0x22, 0x35,
  0x22, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x6C, 0x69, 0x6E, 0x6B, 0x20, 0x68, 0x72,
  0x65, 0x66, 0x3D, 0x22, 0x53, 0x74, 0x79, 0x6C, 0x65, 0x73, 0x2E, 0x63, 0x73, 0x73, 0x22, 0x20,
  0x72, 0x65, 0x6C, 0x3D, 0x22, 0x73, 0x74, 0x79, 0x6C, 0x65, 0x73, 0x68, 0x65, 0x65, 0x74, 0x22,
  0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x3C, 0x2F, 0x68, 0x65, 0x61, 0x64, 0x3E, 0x0D, 0x0A, 0x20, 0x20,
  0x3C, 0x62, 0x6F, 0x64, 0x79, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x68, 0x65, 0x61,
  0x64, 0x65, 0x72, 0x3E, 0x65, 0x6D, 0x4E, 0x65, 0x74, 0x20, 0x69, 0x6E, 0x66, 0x6F, 0x72, 0x6D,
  0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x28, 0x6D, 0x65, 0x74, 0x61, 0x20, 0x72, 0x65, 0x66, 0x72,
  0x65, 0x73, 0x68, 0x29, 0x3C, 0x2F, 0x68, 0x65, 0x61, 0x64, 0x65, 0x72, 0x3E, 0x0D, 0x0A, 0x20,
  0x20, 0x20, 0x20, 0x3C, 0x64, 0x69, 0x76, 0x20, 0x69, 0x64, 0x3D, 0x22, 0x65, 0x6D, 0x4E, 0x65,
  0x74, 0x22, 0x20, 0x63, 0x6C, 0x61, 0x73, 0x73, 0x3D, 0x22, 0x63, 0x6F, 0x6E, 0x74, 0x65, 0x6E,
  0x74, 0x22, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x21, 0x2D, 0x2D, 0x23,
  0x65, 0x78, 0x65, 0x63, 0x20, 0x63, 0x67, 0x69, 0x3D, 0x22, 0x47, 0x65, 0x74, 0x49, 0x50, 0x49,
  0x6E, 0x66, 0x6F, 0x22, 0x2D, 0x2D, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C,
  0x62, 0x72, 0x3E, 0x54, 0x68, 0x65, 0x20, 0x64, 0x61, 0x74, 0x61, 0x20, 0x61, 0x62, 0x6F, 0x76,
  0x65, 0x20, 0x73, 0x68, 0x6F, 0x77, 0x73, 0x20, 0x73, 0x6F, 0x6D, 0x65, 0x20, 0x72, 0x65, 0x61,
  0x6C, 0x20, 0x74, 0x69, 0x6D, 0x65, 0x20, 0x69, 0x6E, 0x66, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x69,
  0x6F, 0x6E, 0x20, 0x6F, 0x66, 0x20, 0x74, 0x68, 0x65, 0x20, 0x6E, 0x65, 0x74, 0x77, 0x6F, 0x72,
  0x6B, 0x20, 0x73, 0x74, 0x61, 0x63, 0x6B, 0x20, 0x75, 0x73, 0x65, 0x64, 0x2E, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x54,
  0x68, 0x65, 0x20, 0x73, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x20, 0x75, 0x73, 0x65, 0x73, 0x20, 0x6D,
  0x65, 0x74, 0x61, 0x20, 0x72, 0x65, 0x66, 0x72, 0x65, 0x73, 0x68, 0x20, 0x74, 0x6F, 0x20, 0x61,
  0x75, 0x74, 0x6F, 0x6D, 0x61, 0x74, 0x69, 0x63, 0x61, 0x6C, 0x6C, 0x79, 0x20, 0x72, 0x65, 0x66,
  0x72, 0x65, 0x73, 0x68, 0x20, 0x74, 0x68, 0x65, 0x20, 0x77, 0x65, 0x62, 0x20, 0x70, 0x61, 0x67,
  0x65, 0x2E, 0x20, 0x54, 0x68, 0x65, 0x20, 0x70, 0x61, 0x67, 0x65, 0x20, 0x77, 0x69, 0x6C, 0x6C,
  0x20, 0x62, 0x65, 0x20, 0x72, 0x65, 0x6C, 0x6F, 0x61, 0x64, 0x65, 0x64, 0x20, 0x65, 0x76, 0x65,
  0x72, 0x79, 0x20, 0x35, 0x20, 0x73, 0x65, 0x63, 0x6F, 0x6E, 0x64, 0x73, 0x2E, 0x0D, 0x0A, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x62, 0x72, 0x3E, 0x49, 0x6E, 0x66, 0x6F, 0x72, 0x6D, 0x61,
  0x74, 0x69, 0x6F, 0x6E, 0x20, 0x61, 0x62, 0x6F, 0x75, 0x74, 0x20, 0x79, 0x6F, 0x75, 0x72, 0x20,
  0x61, 0x70, 0x70, 0x6C, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x63, 0x61, 0x6E, 0x20,
  0x65, 0x61, 0x73, 0x69, 0x6C, 0x79, 0x20, 0x62, 0x65, 0x20, 0x64, 0x69, 0x73, 0x70, 0x6C, 0x61,
  0x79, 0x65, 0x64, 0x20, 0x69, 0x6E, 0x20, 0x74, 0x68, 0x65, 0x20, 0x73, 0x61, 0x6D, 0x65, 0x20,
  0x77, 0x61, 0x79, 0x2E, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20,
  0x3C, 0x2F, 0x64, 0x69, 0x76, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x69, 0x6D, 0x67,
  0x20, 0x73, 0x72, 0x63, 0x3D, 0x22, 0x4C, 0x6F, 0x67, 0x6F, 0x2E, 0x67, 0x69, 0x66, 0x22, 0x20,
  0x61, 0x6C, 0x74, 0x3D, 0x22, 0x53, 0x65, 0x67, 0x67, 0x65, 0x72, 0x20, 0x63, 0x6F, 0x6D, 0x70,
  0x61, 0x6E, 0x79, 0x20, 0x6C, 0x6F, 0x67, 0x6F, 0x22, 0x20, 0x63, 0x6C, 0x61, 0x73, 0x73, 0x3D,
  0x22, 0x6C, 0x6F, 0x67, 0x6F, 0x22, 0x3E, 0x20, 0x20, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C,
  0x66, 0x6F, 0x6F, 0x74, 0x65, 0x72, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
  0x3C, 0x70, 0x3E, 0x3C, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 0x69, 0x6E, 0x64, 0x65,
  0x78, 0x2E, 0x68, 0x74, 0x6D, 0x22, 0x3E, 0x42, 0x61, 0x63, 0x6B, 0x20, 0x74, 0x6F, 0x20, 0x6D,
  0x61, 0x69, 0x6E, 0x3C, 0x2F, 0x61, 0x3E, 0x3C, 0x2F, 0x70, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20,
  0x20, 0x20, 0x20, 0x20, 0x3C, 0x70, 0x3E, 0x53, 0x45, 0x47, 0x47, 0x45, 0x52, 0x20, 0x4D, 0x69,
  0x63, 0x72, 0x6F, 0x63, 0x6F, 0x6E, 0x74, 0x72, 0x6F, 0x6C, 0x6C, 0x65, 0x72, 0x20, 0x47, 0x6D,
  0x62, 0x48, 0x20, 0x7C, 0x7C, 0x20, 0x3C, 0x61, 0x20, 0x68, 0x72, 0x65, 0x66, 0x3D, 0x22, 0x68,
  0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x77, 0x77, 0x77, 0x2E, 0x73, 0x65, 0x67, 0x67, 0x65, 0x72,
  0x2E, 0x63, 0x6F, 0x6D, 0x22, 0x3E, 0x77, 0x77, 0x77, 0x2E, 0x73, 0x65, 0x67, 0x67, 0x65, 0x72,
  0x2E, 0x63, 0x6F, 0x6D, 0x3C, 0x2F, 0x61, 0x3E, 0x20, 0x3C, 0x73, 0x70, 0x61, 0x6E, 0x20, 0x63,
  0x6C, 0x61, 0x73, 0x73, 0x3D, 0x22, 0x68, 0x69, 0x6E, 0x74, 0x22, 0x3E, 0x28, 0x65, 0x78, 0x74,
  0x65, 0x72, 0x6E, 0x61, 0x6C, 0x20, 0x6C, 0x69, 0x6E, 0x6B, 0x29, 0x3C, 0x2F, 0x73, 0x70, 0x61,
  0x6E, 0x3E, 0x20, 0x3C, 0x2F, 0x70, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x2F, 0x66,
  0x6F, 0x6F, 0x74, 0x65, 0x72, 0x3E, 0x0D, 0x0A, 0x20, 0x20, 0x3C, 0x2F, 0x62, 0x6F, 0x64, 0x79,
  0x3E, 0x0D, 0x0A, 0x3C, 0x2F, 0x68, 0x74, 0x6D, 0x6C, 0x3E
};

/*************************** End of file ****************************/
