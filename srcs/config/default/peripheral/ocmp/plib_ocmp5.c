/*******************************************************************************
  Output Compare OCMP5 Peripheral Library (PLIB)

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ocmp5.c

  Summary:
    OCMP5 Source File

  Description:
    None

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
#include "plib_ocmp5.h"
#include "interrupts.h"

// *****************************************************************************

// *****************************************************************************
// Section: OCMP5 Implementation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************


static volatile OCMP_OBJECT ocmp5Obj;

void OCMP5_Initialize (void)
{
    /*Setup OC5CON        */
    /*OCM         = 5        */
    /*OCTSEL       = 0        */
    /*OC32         = 1        */
    /*SIDL         = false    */

    OC5CON = 0x25;

    OC5R = 0;
    OC5RS = 0;

    IEC0SET = _IEC0_OC5IE_MASK;
}

void OCMP5_Enable (void)
{
    OC5CONSET = _OC5CON_ON_MASK;
}

void OCMP5_Disable (void)
{
    OC5CONCLR = _OC5CON_ON_MASK;
}


void OCMP5_CompareValueSet (uint32_t value)
{
    OC5R = value;
}

uint32_t OCMP5_CompareValueGet (void)
{
    return OC5R;
}

void OCMP5_CompareSecondaryValueSet (uint32_t value)
{
    OC5RS = value;
}

uint32_t OCMP5_CompareSecondaryValueGet (void)
{
    return OC5RS;
}


void OCMP5_CallbackRegister(OCMP_CALLBACK callback, uintptr_t context)
{
    ocmp5Obj.callback = callback;

    ocmp5Obj.context = context;
}

void __attribute__((used)) OUTPUT_COMPARE_5_InterruptHandler (void)
{
    /* Additional local variable to prevent MISRA C violations (Rule 13.x) */
    uintptr_t context = ocmp5Obj.context;
    IFS0CLR = _IFS0_OC5IF_MASK;    //Clear IRQ flag

    if( (ocmp5Obj.callback != NULL))
    {
        ocmp5Obj.callback(context);
    }
}

