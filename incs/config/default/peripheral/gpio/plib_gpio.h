/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h UUUUUUUUU

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

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

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for EnA pin ***/
#define EnA_Set()               (LATGSET = (1U<<15))
#define EnA_Clear()             (LATGCLR = (1U<<15))
#define EnA_Toggle()            (LATGINV= (1U<<15))
#define EnA_OutputEnable()      (TRISGCLR = (1U<<15))
#define EnA_InputEnable()       (TRISGSET = (1U<<15))
#define EnA_Get()               ((PORTG >> 15) & 0x1U)
#define EnA_GetLatch()          ((LATG >> 15) & 0x1U)
#define EnA_PIN                  GPIO_PIN_RG15

/*** Macros for EnZ pin ***/
#define EnZ_Set()               (LATASET = (1U<<5))
#define EnZ_Clear()             (LATACLR = (1U<<5))
#define EnZ_Toggle()            (LATAINV= (1U<<5))
#define EnZ_OutputEnable()      (TRISACLR = (1U<<5))
#define EnZ_InputEnable()       (TRISASET = (1U<<5))
#define EnZ_Get()               ((PORTA >> 5) & 0x1U)
#define EnZ_GetLatch()          ((LATA >> 5) & 0x1U)
#define EnZ_PIN                  GPIO_PIN_RA5

/*** Macros for EnY pin ***/
#define EnY_Set()               (LATESET = (1U<<5))
#define EnY_Clear()             (LATECLR = (1U<<5))
#define EnY_Toggle()            (LATEINV= (1U<<5))
#define EnY_OutputEnable()      (TRISECLR = (1U<<5))
#define EnY_InputEnable()       (TRISESET = (1U<<5))
#define EnY_Get()               ((PORTE >> 5) & 0x1U)
#define EnY_GetLatch()          ((LATE >> 5) & 0x1U)
#define EnY_PIN                  GPIO_PIN_RE5

/*** Macros for EnX pin ***/
#define EnX_Set()               (LATESET = (1U<<6))
#define EnX_Clear()             (LATECLR = (1U<<6))
#define EnX_Toggle()            (LATEINV= (1U<<6))
#define EnX_OutputEnable()      (TRISECLR = (1U<<6))
#define EnX_InputEnable()       (TRISESET = (1U<<6))
#define EnX_Get()               ((PORTE >> 6) & 0x1U)
#define EnX_GetLatch()          ((LATE >> 6) & 0x1U)
#define EnX_PIN                  GPIO_PIN_RE6

/*** Macros for LED1 pin ***/
#define LED1_Set()               (LATESET = (1U<<7))
#define LED1_Clear()             (LATECLR = (1U<<7))
#define LED1_Toggle()            (LATEINV= (1U<<7))
#define LED1_OutputEnable()      (TRISECLR = (1U<<7))
#define LED1_InputEnable()       (TRISESET = (1U<<7))
#define LED1_Get()               ((PORTE >> 7) & 0x1U)
#define LED1_GetLatch()          ((LATE >> 7) & 0x1U)
#define LED1_PIN                  GPIO_PIN_RE7

/*** Macros for SW1 pin ***/
#define SW1_Set()               (LATCSET = (1U<<3))
#define SW1_Clear()             (LATCCLR = (1U<<3))
#define SW1_Toggle()            (LATCINV= (1U<<3))
#define SW1_OutputEnable()      (TRISCCLR = (1U<<3))
#define SW1_InputEnable()       (TRISCSET = (1U<<3))
#define SW1_Get()               ((PORTC >> 3) & 0x1U)
#define SW1_GetLatch()          ((LATC >> 3) & 0x1U)
#define SW1_PIN                  GPIO_PIN_RC3

/*** Macros for DirZ pin ***/
#define DirZ_Set()               (LATGSET = (1U<<9))
#define DirZ_Clear()             (LATGCLR = (1U<<9))
#define DirZ_Toggle()            (LATGINV= (1U<<9))
#define DirZ_OutputEnable()      (TRISGCLR = (1U<<9))
#define DirZ_InputEnable()       (TRISGSET = (1U<<9))
#define DirZ_Get()               ((PORTG >> 9) & 0x1U)
#define DirZ_GetLatch()          ((LATG >> 9) & 0x1U)
#define DirZ_PIN                  GPIO_PIN_RG9

/*** Macros for A_Max pin ***/
#define A_Max_Set()               (LATBSET = (1U<<1))
#define A_Max_Clear()             (LATBCLR = (1U<<1))
#define A_Max_Toggle()            (LATBINV= (1U<<1))
#define A_Max_OutputEnable()      (TRISBCLR = (1U<<1))
#define A_Max_InputEnable()       (TRISBSET = (1U<<1))
#define A_Max_Get()               ((PORTB >> 1) & 0x1U)
#define A_Max_GetLatch()          ((LATB >> 1) & 0x1U)
#define A_Max_PIN                  GPIO_PIN_RB1

/*** Macros for SW2 pin ***/
#define SW2_Set()               (LATBSET = (1U<<0))
#define SW2_Clear()             (LATBCLR = (1U<<0))
#define SW2_Toggle()            (LATBINV= (1U<<0))
#define SW2_OutputEnable()      (TRISBCLR = (1U<<0))
#define SW2_InputEnable()       (TRISBSET = (1U<<0))
#define SW2_Get()               ((PORTB >> 0) & 0x1U)
#define SW2_GetLatch()          ((LATB >> 0) & 0x1U)
#define SW2_PIN                  GPIO_PIN_RB0

/*** Macros for LED2 pin ***/
#define LED2_Set()               (LATASET = (1U<<9))
#define LED2_Clear()             (LATACLR = (1U<<9))
#define LED2_Toggle()            (LATAINV= (1U<<9))
#define LED2_OutputEnable()      (TRISACLR = (1U<<9))
#define LED2_InputEnable()       (TRISASET = (1U<<9))
#define LED2_Get()               ((PORTA >> 9) & 0x1U)
#define LED2_GetLatch()          ((LATA >> 9) & 0x1U)
#define LED2_PIN                  GPIO_PIN_RA9

/*** Macros for Coolant pin ***/
#define Coolant_Set()               (LATBSET = (1U<<15))
#define Coolant_Clear()             (LATBCLR = (1U<<15))
#define Coolant_Toggle()            (LATBINV= (1U<<15))
#define Coolant_OutputEnable()      (TRISBCLR = (1U<<15))
#define Coolant_InputEnable()       (TRISBSET = (1U<<15))
#define Coolant_Get()               ((PORTB >> 15) & 0x1U)
#define Coolant_GetLatch()          ((LATB >> 15) & 0x1U)
#define Coolant_PIN                  GPIO_PIN_RB15

/*** Macros for X_Min pin ***/
#define X_Min_Set()               (LATASET = (1U<<4))
#define X_Min_Clear()             (LATACLR = (1U<<4))
#define X_Min_Toggle()            (LATAINV= (1U<<4))
#define X_Min_OutputEnable()      (TRISACLR = (1U<<4))
#define X_Min_InputEnable()       (TRISASET = (1U<<4))
#define X_Min_Get()               ((PORTA >> 4) & 0x1U)
#define X_Min_GetLatch()          ((LATA >> 4) & 0x1U)
#define X_Min_PIN                  GPIO_PIN_RA4

/*** Macros for Y_Min pin ***/
#define Y_Min_Set()               (LATDSET = (1U<<0))
#define Y_Min_Clear()             (LATDCLR = (1U<<0))
#define Y_Min_Toggle()            (LATDINV= (1U<<0))
#define Y_Min_OutputEnable()      (TRISDCLR = (1U<<0))
#define Y_Min_InputEnable()       (TRISDSET = (1U<<0))
#define Y_Min_Get()               ((PORTD >> 0) & 0x1U)
#define Y_Min_GetLatch()          ((LATD >> 0) & 0x1U)
#define Y_Min_PIN                  GPIO_PIN_RD0

/*** Macros for Z_Min pin ***/
#define Z_Min_Set()               (LATDSET = (1U<<13))
#define Z_Min_Clear()             (LATDCLR = (1U<<13))
#define Z_Min_Toggle()            (LATDINV= (1U<<13))
#define Z_Min_OutputEnable()      (TRISDCLR = (1U<<13))
#define Z_Min_InputEnable()       (TRISDSET = (1U<<13))
#define Z_Min_Get()               ((PORTD >> 13) & 0x1U)
#define Z_Min_GetLatch()          ((LATD >> 13) & 0x1U)
#define Z_Min_PIN                  GPIO_PIN_RD13

/*** Macros for StepX pin ***/
#define StepX_Get()               ((PORTD >> 4) & 0x1U)
#define StepX_GetLatch()          ((LATD >> 4) & 0x1U)
#define StepX_PIN                  GPIO_PIN_RD4

/*** Macros for StepY pin ***/
#define StepY_Get()               ((PORTD >> 5) & 0x1U)
#define StepY_GetLatch()          ((LATD >> 5) & 0x1U)
#define StepY_PIN                  GPIO_PIN_RD5

/*** Macros for StepZ pin ***/
#define StepZ_Get()               ((PORTF >> 0) & 0x1U)
#define StepZ_GetLatch()          ((LATF >> 0) & 0x1U)
#define StepZ_PIN                  GPIO_PIN_RF0

/*** Macros for StepA pin ***/
#define StepA_Get()               ((PORTF >> 1) & 0x1U)
#define StepA_GetLatch()          ((LATF >> 1) & 0x1U)
#define StepA_PIN                  GPIO_PIN_RF1

/*** Macros for A_Min pin ***/
#define A_Min_Set()               (LATASET = (1U<<6))
#define A_Min_Clear()             (LATACLR = (1U<<6))
#define A_Min_Toggle()            (LATAINV= (1U<<6))
#define A_Min_OutputEnable()      (TRISACLR = (1U<<6))
#define A_Min_InputEnable()       (TRISASET = (1U<<6))
#define A_Min_Get()               ((PORTA >> 6) & 0x1U)
#define A_Min_GetLatch()          ((LATA >> 6) & 0x1U)
#define A_Min_PIN                  GPIO_PIN_RA6

/*** Macros for X_Max pin ***/
#define X_Max_Set()               (LATASET = (1U<<7))
#define X_Max_Clear()             (LATACLR = (1U<<7))
#define X_Max_Toggle()            (LATAINV= (1U<<7))
#define X_Max_OutputEnable()      (TRISACLR = (1U<<7))
#define X_Max_InputEnable()       (TRISASET = (1U<<7))
#define X_Max_Get()               ((PORTA >> 7) & 0x1U)
#define X_Max_GetLatch()          ((LATA >> 7) & 0x1U)
#define X_Max_PIN                  GPIO_PIN_RA7

/*** Macros for Y_Max pin ***/
#define Y_Max_Set()               (LATESET = (1U<<0))
#define Y_Max_Clear()             (LATECLR = (1U<<0))
#define Y_Max_Toggle()            (LATEINV= (1U<<0))
#define Y_Max_OutputEnable()      (TRISECLR = (1U<<0))
#define Y_Max_InputEnable()       (TRISESET = (1U<<0))
#define Y_Max_Get()               ((PORTE >> 0) & 0x1U)
#define Y_Max_GetLatch()          ((LATE >> 0) & 0x1U)
#define Y_Max_PIN                  GPIO_PIN_RE0

/*** Macros for Z_Max pin ***/
#define Z_Max_Set()               (LATESET = (1U<<1))
#define Z_Max_Clear()             (LATECLR = (1U<<1))
#define Z_Max_Toggle()            (LATEINV= (1U<<1))
#define Z_Max_OutputEnable()      (TRISECLR = (1U<<1))
#define Z_Max_InputEnable()       (TRISESET = (1U<<1))
#define Z_Max_Get()               ((PORTE >> 1) & 0x1U)
#define Z_Max_GetLatch()          ((LATE >> 1) & 0x1U)
#define Z_Max_PIN                  GPIO_PIN_RE1

/*** Macros for DirA pin ***/
#define DirA_Set()               (LATGSET = (1U<<12))
#define DirA_Clear()             (LATGCLR = (1U<<12))
#define DirA_Toggle()            (LATGINV= (1U<<12))
#define DirA_OutputEnable()      (TRISGCLR = (1U<<12))
#define DirA_InputEnable()       (TRISGSET = (1U<<12))
#define DirA_Get()               ((PORTG >> 12) & 0x1U)
#define DirA_GetLatch()          ((LATG >> 12) & 0x1U)
#define DirA_PIN                  GPIO_PIN_RG12

/*** Macros for DirY pin ***/
#define DirY_Set()               (LATESET = (1U<<2))
#define DirY_Clear()             (LATECLR = (1U<<2))
#define DirY_Toggle()            (LATEINV= (1U<<2))
#define DirY_OutputEnable()      (TRISECLR = (1U<<2))
#define DirY_InputEnable()       (TRISESET = (1U<<2))
#define DirY_Get()               ((PORTE >> 2) & 0x1U)
#define DirY_GetLatch()          ((LATE >> 2) & 0x1U)
#define DirY_PIN                  GPIO_PIN_RE2

/*** Macros for Spindle pin ***/
#define Spindle_Get()               ((PORTE >> 3) & 0x1U)
#define Spindle_GetLatch()          ((LATE >> 3) & 0x1U)
#define Spindle_PIN                  GPIO_PIN_RE3

/*** Macros for DirX pin ***/
#define DirX_Set()               (LATESET = (1U<<4))
#define DirX_Clear()             (LATECLR = (1U<<4))
#define DirX_Toggle()            (LATEINV= (1U<<4))
#define DirX_OutputEnable()      (TRISECLR = (1U<<4))
#define DirX_InputEnable()       (TRISESET = (1U<<4))
#define DirX_Get()               ((PORTE >> 4) & 0x1U)
#define DirX_GetLatch()          ((LATE >> 4) & 0x1U)
#define DirX_PIN                  GPIO_PIN_RE4


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/


#define    GPIO_PORT_A  (0)
#define    GPIO_PORT_B  (1)
#define    GPIO_PORT_C  (2)
#define    GPIO_PORT_D  (3)
#define    GPIO_PORT_E  (4)
#define    GPIO_PORT_F  (5)
#define    GPIO_PORT_G  (6)
typedef uint32_t GPIO_PORT;

typedef enum
{
    GPIO_INTERRUPT_ON_MISMATCH,
    GPIO_INTERRUPT_ON_RISING_EDGE,
    GPIO_INTERRUPT_ON_FALLING_EDGE,
    GPIO_INTERRUPT_ON_BOTH_EDGES,
}GPIO_INTERRUPT_STYLE;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/


#define     GPIO_PIN_RA0  (0U)
#define     GPIO_PIN_RA1  (1U)
#define     GPIO_PIN_RA2  (2U)
#define     GPIO_PIN_RA3  (3U)
#define     GPIO_PIN_RA4  (4U)
#define     GPIO_PIN_RA5  (5U)
#define     GPIO_PIN_RA6  (6U)
#define     GPIO_PIN_RA7  (7U)
#define     GPIO_PIN_RA9  (9U)
#define     GPIO_PIN_RA10  (10U)
#define     GPIO_PIN_RA14  (14U)
#define     GPIO_PIN_RA15  (15U)
#define     GPIO_PIN_RB0  (16U)
#define     GPIO_PIN_RB1  (17U)
#define     GPIO_PIN_RB2  (18U)
#define     GPIO_PIN_RB3  (19U)
#define     GPIO_PIN_RB4  (20U)
#define     GPIO_PIN_RB5  (21U)
#define     GPIO_PIN_RB6  (22U)
#define     GPIO_PIN_RB7  (23U)
#define     GPIO_PIN_RB8  (24U)
#define     GPIO_PIN_RB9  (25U)
#define     GPIO_PIN_RB10  (26U)
#define     GPIO_PIN_RB11  (27U)
#define     GPIO_PIN_RB12  (28U)
#define     GPIO_PIN_RB13  (29U)
#define     GPIO_PIN_RB14  (30U)
#define     GPIO_PIN_RB15  (31U)
#define     GPIO_PIN_RC1  (33U)
#define     GPIO_PIN_RC2  (34U)
#define     GPIO_PIN_RC3  (35U)
#define     GPIO_PIN_RC4  (36U)
#define     GPIO_PIN_RC12  (44U)
#define     GPIO_PIN_RC13  (45U)
#define     GPIO_PIN_RC14  (46U)
#define     GPIO_PIN_RC15  (47U)
#define     GPIO_PIN_RD0  (48U)
#define     GPIO_PIN_RD1  (49U)
#define     GPIO_PIN_RD2  (50U)
#define     GPIO_PIN_RD3  (51U)
#define     GPIO_PIN_RD4  (52U)
#define     GPIO_PIN_RD5  (53U)
#define     GPIO_PIN_RD9  (57U)
#define     GPIO_PIN_RD10  (58U)
#define     GPIO_PIN_RD11  (59U)
#define     GPIO_PIN_RD12  (60U)
#define     GPIO_PIN_RD13  (61U)
#define     GPIO_PIN_RD14  (62U)
#define     GPIO_PIN_RD15  (63U)
#define     GPIO_PIN_RE0  (64U)
#define     GPIO_PIN_RE1  (65U)
#define     GPIO_PIN_RE2  (66U)
#define     GPIO_PIN_RE3  (67U)
#define     GPIO_PIN_RE4  (68U)
#define     GPIO_PIN_RE5  (69U)
#define     GPIO_PIN_RE6  (70U)
#define     GPIO_PIN_RE7  (71U)
#define     GPIO_PIN_RE8  (72U)
#define     GPIO_PIN_RE9  (73U)
#define     GPIO_PIN_RF0  (80U)
#define     GPIO_PIN_RF1  (81U)
#define     GPIO_PIN_RF2  (82U)
#define     GPIO_PIN_RF3  (83U)
#define     GPIO_PIN_RF4  (84U)
#define     GPIO_PIN_RF5  (85U)
#define     GPIO_PIN_RF8  (88U)
#define     GPIO_PIN_RF12  (92U)
#define     GPIO_PIN_RF13  (93U)
#define     GPIO_PIN_RG0  (96U)
#define     GPIO_PIN_RG1  (97U)
#define     GPIO_PIN_RG6  (102U)
#define     GPIO_PIN_RG7  (103U)
#define     GPIO_PIN_RG8  (104U)
#define     GPIO_PIN_RG9  (105U)
#define     GPIO_PIN_RG12  (108U)
#define     GPIO_PIN_RG13  (109U)
#define     GPIO_PIN_RG14  (110U)
#define     GPIO_PIN_RG15  (111U)

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
#define    GPIO_PIN_NONE   (-1)

typedef uint32_t GPIO_PIN;


void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
     uint32_t xvalue = (uint32_t)value;
    GPIO_PortWrite((pin>>4U), (uint32_t)(0x1U) << (pin & 0xFU), (xvalue) << (pin & 0xFU));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return ((((GPIO_PortRead((GPIO_PORT)(pin>>4U))) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (((GPIO_PortLatchRead((GPIO_PORT)(pin>>4U)) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
