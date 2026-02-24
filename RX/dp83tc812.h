/*
 *  Copyright (c) Texas Instruments Incorporated 2020
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * \file  dp83tc812.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        DP83TC812 Ethernet PHY.
 */

/*!
 * \ingroup  DRV_ENETPHY
 * \defgroup ENETPHY_DP83TC812 TI DP83TC812 PHY
 *
 * TI DP83TC812 Ethernet PHY.
 *
 * @{
 */

#ifndef DP83TC812_H_
#define DP83TC812_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

#define DP83TC815_PSF_DOMAIN_ANY              (0xFFU)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief DP83TC812 Master/Slave modes.
 */
typedef enum Dp83tc812_operationMode_e
{
    /*! Use Strap Setting */
    DP83TC812_STRAP_MODE   = 0x0U,

    /*! Use Master Mode */
    DP83TC812_CONTROLLER_MODE     = 0x1U,

    /*! Use Slave Mode */
    DP83TC812_PERIPHERAL_MODE     = 0x2U,
} Dp83tc812_operationMode;

/*!
 * \brief DP83TC812 PHY configuration parameters.
 */
typedef struct Dp83tc812_Cfg_s
{
    /*! Enable TX clock shift */
    bool txClkShiftEn;

    /*! Enable RX clock shift */
    bool rxClkShiftEn;

    /*! Enable PHY interrupts */
    bool interruptEn;

    /*! Enable SGMII auto negotiation */
    bool sgmiiAutoNegEn;

    /*! Master/Slave configuration */
    Dp83tc812_operationMode OpeMode;
} Dp83tc812_Cfg;

typedef struct Dp83tc815_RxTsStats_s
{
    uint32_t psfSeen;
    uint32_t psfParsedOk;
    uint32_t queueOverflow;
    uint32_t lookupHit;
    uint32_t lookupMiss;
    uint32_t staleDrop;
} Dp83tc815_RxTsStats;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize DP83TC812 PHY specific config params.
 *
 * Initializes the DP83TC812 PHY specific configuration parameters.
 *
 * \param cfg       DP83TC812 PHY config structure pointer
 */
void Dp83tc812_initCfg(Dp83tc812_Cfg *cfg);

int32_t Dp83tc815_PsfIngestFrame(const uint8_t *frame,
                                 uint32_t frameSize,
                                 uint8_t port);

int32_t Dp83tc815_RxTsLookup(uint8_t port,
                             uint8_t msgType,
                             uint16_t seqId,
                             uint8_t domain,
                             uint64_t *ts);

void Dp83tc815_RxTsReset(void);

void Dp83tc815_RxTsGetStats(Dp83tc815_RxTsStats *stats);

int32_t Dp83tc815_SetPtpTime(uint64_t ts64);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* DP83TC812_H_ */

/*! @} */
