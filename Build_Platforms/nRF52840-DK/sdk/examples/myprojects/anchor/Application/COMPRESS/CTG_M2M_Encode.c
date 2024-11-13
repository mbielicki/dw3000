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

File    : CTG_M2M_Encode.c
Purpose : Compress memory to memory
*/

#include "CTG.h"  /*emDoc #1*/
#include <stdio.h>

static const U8 _aInput[] = {  /*emDoc #2*/
  "Jabberwocky\n  BY LEWIS CARROLL\n\n\n"
  "'Twas brillig, and the slithy toves\n   Did gyre and gimble in the wabe:\n"
  "All mimsy were the borogoves,\n   And the mome raths outgrabe.\n\n"
  "\"Beware the Jabberwock, my son!\n   The jaws that bite, the claws that catch!\n"
  "Beware the Jubjub bird, and shun\n   The frumious Bandersnatch!\"\n\n"
  "He took his vorpal sword in hand;\n   Long time the manxome foe he sought-\n"
  "So rested he by the Tumtum tree\n   And stood awhile in thought.\n\n"
  "And, as in uffish thought he stood,\n   The Jabberwock, with eyes of flame,\n"
  "Came whiffling through the tulgey wood,\n   And burbled as it came!\n\n"
  "One, two! One, two! And through and through\n   The vorpal blade went snicker-snack!\n\n"
  "He left it dead, and with its head\n   He went galumphing back.\n\n"
  "\"And hast thou slain the Jabberwock?\n   Come to my arms, my beamish boy!\n"
  "O frabjous day! Callooh! Callay!\"\n\n   He chortled in his joy.\n"
  "'Twas brillig, and the slithy toves\n   Did gyre and gimble in the wabe:\n"
  "All mimsy were the borogoves,\n   And the mome raths outgrabe.\n"
};

static U8 _aOutput[sizeof(_aInput)*2];  /*emDoc #3*/

void MainTask(void);
void MainTask(void) {
  int Status;
  //
  Status = CTG_CompressM2M(&_aInput[0],  sizeof(_aInput),  /*emDoc #4*/
                           &_aOutput[0], sizeof(_aOutput),
                           0);
  if (Status >= 0) {  /*emDoc #5*/
    printf("Compressed %u to %d bytes.\n", sizeof(_aInput), Status);
  } else {
    printf("Compression error.\n");
  }
}
