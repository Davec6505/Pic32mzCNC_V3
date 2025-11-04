/*******************************************************************************
  Output Compare OCMP8 Peripheral Library (PLIB)

  Company:
    Microchip Technology Inc.

  File Name:
    plib_ocmp8.c

  Summary:
    OCMP8 Source File

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
#include "plib_ocmp8.h"
#include "interrupts.h"

// *****************************************************************************

// *****************************************************************************
// Section: OCMP8 Implementation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************


void OCMP8_Initialize (void)
{
    /*Setup OC8CON        */
    /*OCM         = 6        */
    /*OCTSEL       = 0        */
    /*OC32         = 0        */
    /*SIDL         = false    */

    OC8CON = 0x6;

    /* unlock system for configuration */
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    CFGCON |= 0x00010000U;
    /* Lock system since done with configuration */
    SYSKEY = 0x33333333;
    OC8R = 2000;
    OC8RS = 2000;

}

void OCMP8_Enable (void)
{
    OC8CONSET = _OC8CON_ON_MASK;
}

void OCMP8_Disable (void)
{
    OC8CONCLR = _OC8CON_ON_MASK;
}



uint16_t OCMP8_CompareValueGet (void)
{
    return (uint16_t)OC8R;
}

void OCMP8_CompareSecondaryValueSet (uint16_t value)
{
    OC8RS = value;
}

uint16_t OCMP8_CompareSecondaryValueGet (void)
{
    return (uint16_t)OC8RS;
}

