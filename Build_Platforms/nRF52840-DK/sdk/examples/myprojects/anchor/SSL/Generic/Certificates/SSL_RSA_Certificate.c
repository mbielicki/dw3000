/*********************************************************************
*                   (c) SEGGER Microcontroller GmbH                  *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
*       (c) 2014 - 2024    SEGGER Microcontroller GmbH               *
*                                                                    *
*       www.segger.com     Support: www.segger.com/ticket            *
*                                                                    *
**********************************************************************
*                                                                    *
*       emSSL * Embedded Transport Layer Security                    *
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
*       emSSL version: V3.4.0                                        *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SSL_RSA_Certificate.c
Purpose : Automatically created from SSL_RSA_Certificate.der using Bin2C.exe
*/

#include "SSL_RSA_Certificate.h"

const unsigned char ssl_rsa_certificate_file[706] = {
  0x30, 0x82, 0x02, 0xBE, 0x30, 0x82, 0x01, 0xA6, 0xA0, 0x03, 0x02, 0x01, 0x02, 0x02, 0x09, 0x00,
  0x9B, 0xED, 0x04, 0xE4, 0xCB, 0xFA, 0x41, 0xAB, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86,
  0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x30, 0x14, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55,
  0x04, 0x03, 0x0C, 0x09, 0x6C, 0x6F, 0x63, 0x61, 0x6C, 0x68, 0x6F, 0x73, 0x74, 0x30, 0x1E, 0x17,
  0x0D, 0x32, 0x32, 0x30, 0x36, 0x30, 0x32, 0x31, 0x30, 0x31, 0x35, 0x30, 0x38, 0x5A, 0x17, 0x0D,
  0x34, 0x32, 0x30, 0x35, 0x32, 0x38, 0x31, 0x30, 0x31, 0x35, 0x30, 0x38, 0x5A, 0x30, 0x14, 0x31,
  0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x03, 0x0C, 0x09, 0x6C, 0x6F, 0x63, 0x61, 0x6C, 0x68,
  0x6F, 0x73, 0x74, 0x30, 0x82, 0x01, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7,
  0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82, 0x01, 0x0A, 0x02,
  0x82, 0x01, 0x01, 0x00, 0xEB, 0xCF, 0xB3, 0xCB, 0x26, 0x24, 0xE6, 0x36, 0x3D, 0xA0, 0xAD, 0xBE,
  0x92, 0xBD, 0x54, 0x72, 0x7C, 0xDD, 0x8C, 0xF0, 0x5C, 0xE7, 0x53, 0xB9, 0x34, 0x9A, 0xF7, 0x64,
  0x03, 0xF3, 0x8A, 0xD3, 0x52, 0x01, 0xF5, 0x5A, 0x6A, 0x18, 0x0B, 0xBB, 0xC2, 0x5D, 0xDA, 0x53,
  0x78, 0xB4, 0xFE, 0x47, 0x33, 0xE5, 0x44, 0x45, 0x1C, 0x91, 0x34, 0xCD, 0x9A, 0xBD, 0xD2, 0x60,
  0x0E, 0x80, 0x3D, 0x91, 0xCD, 0x69, 0x40, 0x05, 0x54, 0x26, 0xDC, 0x67, 0xB7, 0xF7, 0xB9, 0xA5,
  0x7D, 0x9D, 0xFD, 0x68, 0x97, 0x55, 0x12, 0x09, 0xED, 0x97, 0x0B, 0xC9, 0xBB, 0x89, 0xD2, 0xCC,
  0x44, 0xF1, 0xE8, 0xA5, 0xB8, 0x3B, 0x4E, 0x28, 0xC7, 0x45, 0xA9, 0xE1, 0x3E, 0xD8, 0x5B, 0x05,
  0xD8, 0xE5, 0xBA, 0x63, 0x7D, 0x5D, 0x73, 0x0F, 0x2B, 0x31, 0x3F, 0x6A, 0x9B, 0xDF, 0x8B, 0xB5,
  0xE4, 0xDA, 0xFA, 0xD4, 0xC1, 0xE4, 0xC8, 0xD9, 0x9A, 0x11, 0x0E, 0x19, 0xB6, 0x40, 0x9A, 0x83,
  0xAC, 0xA3, 0xDA, 0xD9, 0x4A, 0xF1, 0xAE, 0x6D, 0xFD, 0x81, 0x4E, 0x2A, 0x58, 0x43, 0x40, 0x66,
  0x82, 0xA3, 0xF1, 0x85, 0x0D, 0xE1, 0x7D, 0x2D, 0xC3, 0x3C, 0x65, 0xB2, 0x97, 0x72, 0x1F, 0x2A,
  0x6C, 0xC6, 0x03, 0x18, 0x76, 0x2B, 0xE3, 0x15, 0x76, 0xC7, 0xDF, 0x85, 0xB5, 0x93, 0xC1, 0x80,
  0xDD, 0xEA, 0x73, 0x93, 0xE2, 0xFF, 0xC9, 0x92, 0xDB, 0x2E, 0xFA, 0xD0, 0x80, 0x6C, 0x3F, 0x47,
  0x53, 0xCC, 0x20, 0xD6, 0x55, 0x37, 0xEF, 0xF8, 0x95, 0xCF, 0xAF, 0x02, 0xD9, 0x7D, 0x2F, 0x88,
  0x7B, 0x86, 0x9E, 0x2B, 0x93, 0x57, 0x30, 0x43, 0xEA, 0x3F, 0x33, 0x51, 0xDE, 0x3A, 0xF9, 0x32,
  0x2C, 0xB1, 0xF5, 0x88, 0x3F, 0x5C, 0x5B, 0x94, 0x5E, 0x41, 0x26, 0x31, 0x11, 0x39, 0xA4, 0xB2,
  0x98, 0x9F, 0xF1, 0x39, 0x02, 0x03, 0x01, 0x00, 0x01, 0xA3, 0x13, 0x30, 0x11, 0x30, 0x0F, 0x06,
  0x03, 0x55, 0x1D, 0x11, 0x04, 0x08, 0x30, 0x06, 0x87, 0x04, 0x7F, 0x00, 0x00, 0x01, 0x30, 0x0D,
  0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x03, 0x82, 0x01,
  0x01, 0x00, 0x23, 0x8E, 0xFE, 0x15, 0xBA, 0xCE, 0x65, 0x71, 0x34, 0x54, 0x90, 0x4D, 0x2D, 0xF9,
  0xBD, 0xAC, 0x17, 0x16, 0x27, 0xCC, 0xDD, 0xFA, 0xBB, 0x32, 0xF6, 0xBB, 0xE8, 0x99, 0xEC, 0x88,
  0x3A, 0x9C, 0xA6, 0x4E, 0xD3, 0xA7, 0xF6, 0x65, 0xDA, 0xEC, 0x86, 0x02, 0x95, 0x95, 0x55, 0x13,
  0xF0, 0xEE, 0x7D, 0x21, 0x2A, 0x2D, 0x64, 0xF8, 0xC7, 0xD4, 0x82, 0xAA, 0x72, 0x2B, 0xE9, 0xCD,
  0x04, 0xF4, 0x29, 0x89, 0x08, 0xFF, 0xE2, 0x16, 0x7B, 0x53, 0xE6, 0xF9, 0x71, 0x14, 0xB0, 0xFC,
  0x48, 0xDD, 0xBD, 0x07, 0x3F, 0x78, 0x82, 0xE4, 0x47, 0x77, 0x81, 0xDA, 0xF5, 0xB8, 0x04, 0x56,
  0x8A, 0x8A, 0xD7, 0x7E, 0x8C, 0xFA, 0x27, 0xD3, 0x1C, 0x1F, 0xEC, 0x2A, 0x7A, 0x62, 0x38, 0x46,
  0x47, 0x69, 0x59, 0x2C, 0x41, 0xB8, 0x13, 0x62, 0x22, 0x72, 0x25, 0x3C, 0x7B, 0x2B, 0xAB, 0xF4,
  0x7D, 0xFF, 0xF9, 0x5D, 0xC0, 0x45, 0x2D, 0xD8, 0x11, 0x0D, 0xF9, 0x4D, 0xFD, 0x30, 0x5C, 0xC0,
  0x3C, 0x50, 0x65, 0x15, 0xEF, 0x90, 0xB0, 0x8F, 0x33, 0x3C, 0x76, 0x0C, 0x9E, 0x7C, 0xCE, 0x55,
  0x0C, 0xCB, 0x11, 0xC5, 0x04, 0xFF, 0xF5, 0xF5, 0xDF, 0xE9, 0x88, 0x72, 0x38, 0xE5, 0x50, 0x8F,
  0x0C, 0xC4, 0x52, 0xD1, 0x9B, 0xB6, 0x3F, 0x5C, 0xCD, 0x5F, 0x29, 0x13, 0xDB, 0x61, 0x98, 0xF8,
  0xB4, 0x28, 0xCE, 0x38, 0xA6, 0xE9, 0x74, 0x84, 0x0A, 0xC8, 0x65, 0xBB, 0xA5, 0xCA, 0xEB, 0xAF,
  0x70, 0xDC, 0xB6, 0xA6, 0x4F, 0x2D, 0x44, 0x15, 0x14, 0x02, 0xA9, 0xBC, 0x9D, 0x2D, 0x5E, 0xA8,
  0x1E, 0xEB, 0xCC, 0xDA, 0x21, 0x89, 0x29, 0xE0, 0x84, 0x40, 0x31, 0xA1, 0xB7, 0x18, 0x42, 0x91,
  0xF7, 0x67, 0x46, 0xF3, 0xE1, 0xEF, 0xA7, 0xB5, 0xC9, 0x60, 0x84, 0xB0, 0xD8, 0xC7, 0xE7, 0x2E,
  0x34, 0x67
};

/*************************** End of file ****************************/