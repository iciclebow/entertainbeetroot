/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  dp83tc812.c
 *
 * \brief This file contains the implementation of the DP83TC812 PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <include/core/enet_utils.h>
#include <priv/core/enet_trace_priv.h>
#include <include/phy/enetphy.h>
#include <include/phy/dp83tc812.h>
#include "enetphy_priv.h"
#include "generic_phy.h"
#include "dp83tc812_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83TC812_OUI                         (0x0800xxx)	//PHYIDR2 Register: 15-10 Unique Identifier 2 R 28h 
//#define DP83TC812_MODEL                       (0x27U)
#define DP83TC812_MODEL                       (0xxxx)		//PHYIDR2 Register: 9-4 Model Number R 2Eh //I only have one Phy and one revision I care about
// #define DP83TC812_REV_CS1					  (0U)
// #define DP83TC812_REV_CS2			          (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

enum dp83720_chip_type {
         DP83TC812_CS1 = 0,
         DP83TC812_CS2 = 1,
};

struct dp83tc812_privParams {
         int chip;
		 bool is_master;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Dp83tc812_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Dp83tc812_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Dp83tc812_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Dp83tc812_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable);

static void Dp83tc812_reset(EnetPhy_Handle hPhy);

static void Dp83tc812_resetHw(EnetPhy_Handle hPhy);

static bool Dp83tc812_isResetComplete(EnetPhy_Handle hPhy);

static int32_t Dp83tc812_readMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val);

static void Dp83tc812_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_setBitsMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_readStraps(EnetPhy_Handle hPhy, Dp83tc812_operationMode msMode);

static void Dp83tc812_writeSeq(EnetPhy_Handle hPhy, const struct dp83tc812_init_reg *init_data, int size);

static void Dp83tc812_chipInit(EnetPhy_Handle hPhy);

static void Dp83tc812_setMiiMode(EnetPhy_Handle hPhy, EnetPhy_Mii mii);

static void Dp83tc812_configAutoNeg(EnetPhy_Handle hPhy, bool sgmiiAutoNegEn);

static void Dp83tc812_configClkShift(EnetPhy_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn);

static void Dp83tc812_configIntr(EnetPhy_Handle hPhy, bool intrEn);

static int32_t Dp83tc812_processStatusFrame(EnetPhy_Handle hPhy,
                                            Enet_MacPort macPort,
                                            const uint8_t *frame,
                                            uint32_t frameLen);

static void Dp83tc812_printRegs(EnetPhy_Handle hPhy);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83tc812 =
{
    .name               = "dp83tc812",
    .isPhyDevSupported  = Dp83tc812_isPhyDevSupported,
    .isMacModeSupported = Dp83tc812_isMacModeSupported,
    .config             = Dp83tc812_config,
    .reset              = Dp83tc812_reset,
    .isResetComplete    = Dp83tc812_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .processStatusFrame = Dp83tc812_processStatusFrame,
    .printRegs          = Dp83tc812_printRegs,
};

/* PHY Device Attributes */
static struct dp83tc812_privParams dp83tc812_params = {
	.chip = -1,
	.is_master = false,
};

/* ======================= Phase 1 PHY PTP PSF ingest ======================= */
/*
 * Phase-1 goal:
 * - Accept all PHY status frames at the driver boundary through the new PHY IOCTL.
 * - Validate the fixed L2 header, decode known PSF shapes, and store only compact
 *   semantic data in a driver-owned ring buffer.
 * - Keep everything centralized in one block so bring-up prints and storage logic
 *   are easy to inspect while the API path is still being validated.
 *
 * Notes:
 * - Single-PHY assumption is intentional for this phase.
 * - The type values are accepted in both 0x0010/0x0020/0x0040 and 0x1000/0x2000/0x4000
 *   form until the exact 815 encoding is fully pinned down from runtime captures.
 * - TX PSF payload is accepted with the 8-byte timestamp body already observed on
 *   the wire; if extra words are present, they are decoded opportunistically.
 *
 * Debug controls:
 * - The ingest path always runs. Only the print volume is controlled here.
 * - DP83TC812_PSF_DEBUG_ENABLE is intentionally handled as a behavioral switch,
 *   not as a way to compile out the phase-1 block. This keeps the generated
 *   driver surface stable when toggling debug on/off during bring-up.
 * - DP83TC812_PSF_DEBUG_PRINT_EVERY_N decimates per-frame prints. A value of 1000
 *   means "print one detailed frame trace every 1000 ingested PSFs".
 * - DP83TC812_PSF_DEBUG_SUMMARY_EVERY_N prints one summary after that many ingested
 *   PSFs have been seen.
 *
 * Summary field explainer:
 * - ingested         : all PSFs handed into the driver API
 * - storedTx/storedRx: recognized TX/RX PSFs that were decoded and written to the ring
 * - rejectShort      : null/too-short frames rejected before header/type decode
 * - rejectHdr        : frames reaching this API but failing the fixed PSF L2 header match
 * - rejectTxShort    : TX PSF type matched, but payload was shorter than the expected body
 * - rejectRxShort    : RX PSF type matched, but payload was shorter than the expected body
 * - dropEvent        : event PSFs seen and intentionally ignored in phase 1
 * - dropUnsupported  : non-TX/RX/event PSF types seen and intentionally ignored
 * - ringCount        : number of valid entries currently retained in the fixed ring buffer
 * - ringWrite        : next ring slot that will be overwritten on the next store
 * - nextStoreIdx     : monotonic sequence number that will be assigned to the next stored entry
 */
#define DP83TC812_PSF_RING_DEPTH              (32U)
#define DP83TC812_PSF_ETH_HDR_LEN             (14U)
#define DP83TC812_PSF_PTP_HDR_LEN             (2U)
#define DP83TC812_PSF_TYPE_LEN                (2U)
#define DP83TC812_PSF_TX_TYPE0                (0x0010U)
#define DP83TC812_PSF_RX_TYPE0                (0x0020U)
#define DP83TC812_PSF_EVENT_TYPE0             (0x0040U)
#define DP83TC812_PSF_TX_TYPE1                (0x1000U)
#define DP83TC812_PSF_RX_TYPE1                (0x2000U)
#define DP83TC812_PSF_EVENT_TYPE1             (0x4000U)

#ifndef DP83TC812_PSF_DEBUG_ENABLE
#define DP83TC812_PSF_DEBUG_ENABLE            (1U)
#endif

#ifndef DP83TC812_PSF_DEBUG_PRINT_EVERY_N
#define DP83TC812_PSF_DEBUG_PRINT_EVERY_N     (1000U)
#endif

#ifndef DP83TC812_PSF_DEBUG_PRINT_STORES
#define DP83TC812_PSF_DEBUG_PRINT_STORES      (1U)
#endif

#ifndef DP83TC812_PSF_DEBUG_PRINT_DROPS
#define DP83TC812_PSF_DEBUG_PRINT_DROPS       (1U)
#endif

#ifndef DP83TC812_PSF_DEBUG_SUMMARY_ENABLE
#define DP83TC812_PSF_DEBUG_SUMMARY_ENABLE    (1U)
#endif

#ifndef DP83TC812_PSF_DEBUG_SUMMARY_EVERY_N
#define DP83TC812_PSF_DEBUG_SUMMARY_EVERY_N   (50000U)
#endif

typedef struct Dp83tc812_PsfStoreEntry_s
{
    bool valid;
    uint16_t psfType;
    Enet_MacPort macPort;
    uint64_t ts64;
    uint16_t seqId;
    uint8_t msgType;
    uint16_t hash;
    uint32_t storeIndex;
} Dp83tc812_PsfStoreEntry;

static const uint8_t gDp83tc812PsfEthHdr[DP83TC812_PSF_ETH_HDR_LEN] =
{
    0x01U, 0x1BU, 0x19U, 0x00U, 0x00U, 0x00U,
    0x08U, 0x00U, 0x17U, 0x0BU, 0x6BU, 0x0FU,
    0x88U, 0xF7U,
};

static Dp83tc812_PsfStoreEntry gDp83tc812PsfRing[DP83TC812_PSF_RING_DEPTH];
static uint32_t gDp83tc812PsfWriteIdx = 0U;
static uint32_t gDp83tc812PsfCount = 0U;
static uint32_t gDp83tc812PsfStoreSeq = 0U;
static uint32_t gDp83tc812PsfIngestCnt = 0U;
static uint32_t gDp83tc812PsfStoredTxCnt = 0U;
static uint32_t gDp83tc812PsfStoredRxCnt = 0U;
static uint32_t gDp83tc812PsfRejectShortCnt = 0U;
static uint32_t gDp83tc812PsfRejectHdrCnt = 0U;
static uint32_t gDp83tc812PsfRejectTxShortCnt = 0U;
static uint32_t gDp83tc812PsfRejectRxShortCnt = 0U;
static uint32_t gDp83tc812PsfDropEventCnt = 0U;
static uint32_t gDp83tc812PsfDropUnsupportedCnt = 0U;

static bool Dp83tc812_psfShouldPrintFrame(void)
{
    bool shouldPrint = false;
    uint32_t printEveryN = DP83TC812_PSF_DEBUG_PRINT_EVERY_N;

    if (DP83TC812_PSF_DEBUG_ENABLE == 0U)
    {
        return false;
    }

    if (printEveryN == 0U)
    {
        printEveryN = 1U;
    }

    shouldPrint = ((gDp83tc812PsfIngestCnt % printEveryN) == 0U);

    return shouldPrint;
}

static void Dp83tc812_psfMaybePrintSummary(EnetPhy_Handle hPhy)
{
    uint32_t summaryEveryN = DP83TC812_PSF_DEBUG_SUMMARY_EVERY_N;

    if ((DP83TC812_PSF_DEBUG_ENABLE == 0U) || (DP83TC812_PSF_DEBUG_SUMMARY_ENABLE == 0U))
    {
        ENET_UTILS_UNUSED(hPhy);
        return;
    }

    if (summaryEveryN == 0U)
    {
        summaryEveryN = 1U;
    }

    if ((gDp83tc812PsfIngestCnt % summaryEveryN) == 0U)
    {
        EnetUtils_printf(
            "PHY %u: phase1 PSF summary ingested=%u storedTx=%u storedRx=%u rejectShort=%u rejectHdr=%u rejectTxShort=%u rejectRxShort=%u dropEvent=%u dropUnsupported=%u ringCount=%u ringWrite=%u nextStoreIdx=%u\r\n",
            hPhy->addr,
            gDp83tc812PsfIngestCnt,
            gDp83tc812PsfStoredTxCnt,
            gDp83tc812PsfStoredRxCnt,
            gDp83tc812PsfRejectShortCnt,
            gDp83tc812PsfRejectHdrCnt,
            gDp83tc812PsfRejectTxShortCnt,
            gDp83tc812PsfRejectRxShortCnt,
            gDp83tc812PsfDropEventCnt,
            gDp83tc812PsfDropUnsupportedCnt,
            gDp83tc812PsfCount,
            gDp83tc812PsfWriteIdx,
            gDp83tc812PsfStoreSeq);
    }
}

static uint16_t Dp83tc812_readLe16(const uint8_t *ptr)
{
    return (uint16_t)ptr[0U] | ((uint16_t)ptr[1U] << 8U);
}

static uint64_t Dp83tc812_decodeTs64(const uint8_t *ptr)
{
    const uint32_t nsLow = Dp83tc812_readLe16(&ptr[0U]);
    const uint32_t nsHigh = Dp83tc812_readLe16(&ptr[2U]) & 0x3FFFU;
    const uint64_t secLow = Dp83tc812_readLe16(&ptr[4U]);
    const uint64_t secHigh = Dp83tc812_readLe16(&ptr[6U]);
    const uint64_t sec = secLow | (secHigh << 16U);
    const uint64_t nsec = nsLow | ((uint64_t)nsHigh << 16U);

    return (sec * 1000000000ULL) + nsec;
}

static bool Dp83tc812_isTxPsfType(uint16_t psfType)
{
    return ((psfType == DP83TC812_PSF_TX_TYPE0) || (psfType == DP83TC812_PSF_TX_TYPE1));
}

static bool Dp83tc812_isRxPsfType(uint16_t psfType)
{
    return ((psfType == DP83TC812_PSF_RX_TYPE0) || (psfType == DP83TC812_PSF_RX_TYPE1));
}

static bool Dp83tc812_isEventPsfType(uint16_t psfType)
{
    return ((psfType == DP83TC812_PSF_EVENT_TYPE0) || (psfType == DP83TC812_PSF_EVENT_TYPE1));
}

static int32_t Dp83tc812_processStatusFrame(EnetPhy_Handle hPhy,
                                            Enet_MacPort macPort,
                                            const uint8_t *frame,
                                            uint32_t frameLen)
{
    Dp83tc812_PsfStoreEntry *entry;
    const uint8_t *payload;
    uint32_t payloadLen;
    uint16_t ptpHdr;
    uint16_t psfType;
    uint32_t slot;
    bool wrapped;
    bool printThisFrame;
    int32_t status = ENETPHY_SOK;

    if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
    {
        gDp83tc812PsfIngestCnt++;
    }
    printThisFrame = Dp83tc812_psfShouldPrintFrame();

    if (printThisFrame)
    {
        EnetUtils_printf("PHY %u: phase1 PSF ingest macPort=%u len=%u\r\n",
                         hPhy->addr, ENET_MACPORT_ID(macPort), frameLen);
    }

    if ((frame == NULL) ||
        (frameLen < (DP83TC812_PSF_ETH_HDR_LEN + DP83TC812_PSF_PTP_HDR_LEN + DP83TC812_PSF_TYPE_LEN)))
    {
        if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
        {
            gDp83tc812PsfRejectShortCnt++;
        }
        if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
            (DP83TC812_PSF_DEBUG_PRINT_DROPS != 0U) &&
            printThisFrame)
        {
            EnetUtils_printf("PHY %u: phase1 PSF reject: short/null frame len=%u\r\n",
                             hPhy->addr, frameLen);
        }
        status = ENETPHY_EBADARGS;
    }
    else if (memcmp(frame, gDp83tc812PsfEthHdr, sizeof(gDp83tc812PsfEthHdr)) != 0)
    {
        if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
        {
            gDp83tc812PsfRejectHdrCnt++;
        }
        if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
            (DP83TC812_PSF_DEBUG_PRINT_DROPS != 0U) &&
            printThisFrame)
        {
            EnetUtils_printf("PHY %u: phase1 PSF reject: header mismatch\r\n", hPhy->addr);
        }
        status = ENETPHY_EINVALIDPARAMS;
    }
    else
    {
        ptpHdr = Dp83tc812_readLe16(&frame[DP83TC812_PSF_ETH_HDR_LEN]);
        psfType = Dp83tc812_readLe16(&frame[DP83TC812_PSF_ETH_HDR_LEN + DP83TC812_PSF_PTP_HDR_LEN]);
        payload = &frame[DP83TC812_PSF_ETH_HDR_LEN + DP83TC812_PSF_PTP_HDR_LEN + DP83TC812_PSF_TYPE_LEN];
        payloadLen = frameLen - (DP83TC812_PSF_ETH_HDR_LEN + DP83TC812_PSF_PTP_HDR_LEN + DP83TC812_PSF_TYPE_LEN);

        if (printThisFrame)
        {
            EnetUtils_printf("PHY %u: phase1 PSF accept ptpHdr=0x%04x type=0x%04x payloadLen=%u\r\n",
                             hPhy->addr, ptpHdr, psfType, payloadLen);
        }

        if (Dp83tc812_isTxPsfType(psfType))
        {
            if (payloadLen < 8U)
            {
                if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
                {
                    gDp83tc812PsfRejectTxShortCnt++;
                }
                if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
                    (DP83TC812_PSF_DEBUG_PRINT_DROPS != 0U) &&
                    printThisFrame)
                {
                    EnetUtils_printf("PHY %u: phase1 PSF reject TX: payload too short (%u)\r\n",
                                     hPhy->addr, payloadLen);
                }
            }
            else
            {
                slot = gDp83tc812PsfWriteIdx;
                entry = &gDp83tc812PsfRing[slot];
                memset(entry, 0, sizeof(*entry));
                entry->valid = true;
                entry->psfType = psfType;
                entry->macPort = macPort;
                entry->ts64 = Dp83tc812_decodeTs64(payload);
                entry->storeIndex = gDp83tc812PsfStoreSeq++;
                if (payloadLen >= 12U)
                {
                    const uint16_t msgTypeHash = Dp83tc812_readLe16(&payload[10U]);
                    entry->seqId = Dp83tc812_readLe16(&payload[8U]);
                    entry->msgType = (uint8_t)(msgTypeHash & 0x000FU);
                    entry->hash = (msgTypeHash >> 4U) & 0x0FFFU;
                }

                gDp83tc812PsfWriteIdx = (slot + 1U) % DP83TC812_PSF_RING_DEPTH;
                wrapped = (gDp83tc812PsfCount == DP83TC812_PSF_RING_DEPTH);
                if (!wrapped)
                {
                    gDp83tc812PsfCount++;
                }
                if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
                {
                    gDp83tc812PsfStoredTxCnt++;
                }

                if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
                    (DP83TC812_PSF_DEBUG_PRINT_STORES != 0U) &&
                    printThisFrame)
                {
                    EnetUtils_printf(
                        "PHY %u: phase1 PSF store TX slot=%u idx=%u count=%u wrapped=%u ts64=%llu seqId=%u msgType=%u hash=0x%03x\r\n",
                        hPhy->addr,
                        slot,
                        entry->storeIndex,
                        gDp83tc812PsfCount,
                        wrapped ? 1U : 0U,
                        (unsigned long long)entry->ts64,
                        entry->seqId,
                        entry->msgType,
                        entry->hash);
                }
            }
        }
        else if (Dp83tc812_isRxPsfType(psfType))
        {
            if (payloadLen < 12U)
            {
                if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
                {
                    gDp83tc812PsfRejectRxShortCnt++;
                }
                if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
                    (DP83TC812_PSF_DEBUG_PRINT_DROPS != 0U) &&
                    printThisFrame)
                {
                    EnetUtils_printf("PHY %u: phase1 PSF reject RX: payload too short (%u)\r\n",
                                     hPhy->addr, payloadLen);
                }
            }
            else
            {
                const uint16_t msgTypeHash = Dp83tc812_readLe16(&payload[10U]);

                slot = gDp83tc812PsfWriteIdx;
                entry = &gDp83tc812PsfRing[slot];
                memset(entry, 0, sizeof(*entry));
                entry->valid = true;
                entry->psfType = psfType;
                entry->macPort = macPort;
                entry->ts64 = Dp83tc812_decodeTs64(payload);
                entry->seqId = Dp83tc812_readLe16(&payload[8U]);
                entry->msgType = (uint8_t)(msgTypeHash & 0x000FU);
                entry->hash = (msgTypeHash >> 4U) & 0x0FFFU;
                entry->storeIndex = gDp83tc812PsfStoreSeq++;

                gDp83tc812PsfWriteIdx = (slot + 1U) % DP83TC812_PSF_RING_DEPTH;
                wrapped = (gDp83tc812PsfCount == DP83TC812_PSF_RING_DEPTH);
                if (!wrapped)
                {
                    gDp83tc812PsfCount++;
                }
                if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
                {
                    gDp83tc812PsfStoredRxCnt++;
                }

                if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
                    (DP83TC812_PSF_DEBUG_PRINT_STORES != 0U) &&
                    printThisFrame)
                {
                    EnetUtils_printf(
                        "PHY %u: phase1 PSF store RX slot=%u idx=%u count=%u wrapped=%u ts64=%llu seqId=%u msgType=%u hash=0x%03x\r\n",
                        hPhy->addr,
                        slot,
                        entry->storeIndex,
                        gDp83tc812PsfCount,
                        wrapped ? 1U : 0U,
                        (unsigned long long)entry->ts64,
                        entry->seqId,
                        entry->msgType,
                        entry->hash);
                }
            }
        }
        else if (Dp83tc812_isEventPsfType(psfType))
        {
            if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
            {
                gDp83tc812PsfDropEventCnt++;
            }
            if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
                (DP83TC812_PSF_DEBUG_PRINT_DROPS != 0U) &&
                printThisFrame)
            {
                EnetUtils_printf("PHY %u: phase1 PSF drop event type=0x%04x (not handled yet)\r\n",
                                 hPhy->addr, psfType);
            }
        }
        else
        {
            if (DP83TC812_PSF_DEBUG_ENABLE != 0U)
            {
                gDp83tc812PsfDropUnsupportedCnt++;
            }
            if ((DP83TC812_PSF_DEBUG_ENABLE != 0U) &&
                (DP83TC812_PSF_DEBUG_PRINT_DROPS != 0U) &&
                printThisFrame)
            {
                EnetUtils_printf("PHY %u: phase1 PSF drop unsupported type=0x%04x\r\n",
                                 hPhy->addr, psfType);
            }
        }
    }

    Dp83tc812_psfMaybePrintSummary(hPhy);

    return status;
}

/*We only need one master and one slave init script because we are only using one chip revision*/
/*! \brief Chip specific init scripts */

static const struct dp83tc812_init_reg dp83tc812_master_init[] = {		//renamed to remove the _cs1
		/* placeholder for Ti's init script.
         *
         *
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * 
         * DO NOT REMOVE THIS EMPTY SPACE
         */

		/* SPARE_IN_FROM_DIG_SL_1 (0x05B7): "register with configurable bits for analog section".
		 * Datasheet note for PTP/PSF: toggle bit5 from 0->1->0 before accessing 0x0Dxx/PTP registers.
		 */
		{0x1F, 0x05B7, 0x0043}, /* 0b0000_0000_0100_0011: bit6=1, bit1=1, bit0=1, bit5=0 (toggle start) */
		{0x1F, 0x05B7, 0x0063}, /* 0b0000_0000_0110_0011: bit5=1 while keeping bit6/1/0 unchanged */
		{0x1F, 0x05B7, 0x0043}, /* 0b0000_0000_0100_0011: bit5 cleared back to 0 (toggle complete) */

		/* PTP_CTL (0x0D00): "provides basic controls for the PTP 802.1AS operation". */
		{0x1F, 0x0D00, 0x0001}, /* 0b0000_0000_0000_0001: PTP Reset bit0=1 (assert reset) */
		{0x1F, 0x0D00, 0x0000}, /* 0b0000_0000_0000_0000: bit0=0 (release reset) */
		{0x1F, 0x0D00, 0x0004}, /* 0b0000_0000_0000_0100: PTP Enable bit2=1 */

		/* PSF_CFG0 (0x0D14): "configuration for the Phy Status Frame function". */
		{0x1F, 0x0D14, 0x470D}, /* 0b0100_0111_0000_1101:
		                         * bit14=1 termination field enable,
		                         * bits12:11=00 MAC source selection default,
		                         * bits10:8=111 minimum preamble=7,
		                         * bit7=0 network-byte-order fields,
		                         * bit6=0 Layer2 PSF packet type,
		                         * bit3=1 TX TS PSF enable, bit2=1 RX TS PSF enable, bit0=1 Event PSF enable.
		                         */

		/* PSF_CFG1 (0x0D21): "first 16-bits of the PTP header for PSF packets". */
		{0x1F, 0x0D21, 0x0200}, /* 0b0000_0010_0000_0000: versionPTP bits11:8=0b0010 (PTP v2), others 0 */

		/* PTP_TXCFG0 (0x0D12): "configuration for IEEE 802.1AS transmit timestamp operation". */
		{0x1F, 0x0D12, 0x4085}, /* 0b0100_0000_1000_0101:
		                         * bit14=1 TX timestamp info enable, so PSF carries sequenceId, messageType, and hash information for timestamp matching,
		                         * bit7=1 Layer2 timestamp detect,
		                         * bits4:1=0b0010 PTP version match=v2,
		                         * bit0=1 transmit timestamp enable.
		                         */

		/* PTP_RXCFG0 (0x0D15): "configuration for IEEE 802.1AS receive timestamp operation". */
		{0x1F, 0x0D15, 0x0085}, /* 0b0000_0000_1000_0101:
		                         * bit7=1 Layer2 timestamp detect,
		                         * bits4:1=0b0010 PTP version match=v2,
		                         * bit0=1 receive timestamp enable.
		                         */
};

// static const struct dp83tc812_init_reg dp83tc812_slave_init[] = { 	//renamed to remove the _cs1
// 		//unused, one one specif PHY version and configuration will be used
// };

/* static const struct dp83tc812_init_reg dp83tc812_cs2_master_init[] = {
		//unused, one one specif PHY version and configuration will be used
}; */

/*static const struct dp83tc812_init_reg dp83tc812_cs2_slave_init[] = {
		//unused, one one specif PHY version and configuration will be used
}; */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83tc812_initCfg(Dp83tc812_Cfg *cfg)
{
	cfg->txClkShiftEn = true;
	cfg->rxClkShiftEn = true;
	cfg->interruptEn = false;
	cfg->sgmiiAutoNegEn = true;
	cfg->OpeMode = DP83TC812_STRAP_MODE;
}

/*This function only needs to check for Vendor ID and Model ID since the hardware this will be running on will only ever use one type and revion*/
static bool Dp83tc812_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

    ENETTRACE_INFO("This is a WIP driver for the 815\n");

    if ((version->oui == DP83TC812_OUI) && (version->model == DP83TC812_MODEL))
    {
        supported = true;
    }

	/*commented out because we are only using one chip version*/
/* 	//Determine PHY version
    if (version->revision == DP83TC812_REV_CS1)
		dp83tc812_params.chip = DP83TC812_CS1;
    else if (version->revision == DP83TC812_REV_CS2)
		dp83tc812_params.chip = DP83TC812_CS2; */

    return supported;
}

static bool Dp83tc812_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported = false;

    switch (mii)
    {
        case ENETPHY_MAC_MII_MII:
			break;
        case ENETPHY_MAC_MII_GMII:
			break;
        case ENETPHY_MAC_MII_RGMII:
			supported = true;
            break;
		case ENETPHY_MAC_MII_SGMII:
            supported = true;
            break;
        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Dp83tc812_config(EnetPhy_Handle hPhy,
                                const EnetPhy_Cfg *cfg,
								EnetPhy_Mii mii)
{
    const Dp83tc812_Cfg *extendedCfg = (const Dp83tc812_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;

    int32_t status = ENETPHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                         hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;
    }

	/* Read strap register */
	if (status == ENETPHY_SOK)
    {
		Dp83tc812_readStraps(hPhy, extendedCfg->OpeMode);
	}

	/* Set Controller/Peripheral mode - through PHY config */
	if(extendedCfg->OpeMode == DP83TC812_CONTROLLER_MODE)
	{
		dp83tc812_params.is_master = true;
		ENETTRACE_DBG("PHY %u: Controller Mode enabled\n", hPhy->addr);
	}
	else if(extendedCfg->OpeMode == DP83TC812_PERIPHERAL_MODE)
	{
		dp83tc812_params.is_master = false;
		ENETTRACE_DBG("PHY %u: Peripheral Mode enabled\n", hPhy->addr);
	}

	/* Init specific chip */
	if (status == ENETPHY_SOK)
    {
		Dp83tc812_chipInit(hPhy);
	}

	/* Configure MII interface */
	if (status == ENETPHY_SOK)
    {
		Dp83tc812_setMiiMode(hPhy, mii);
	}

	/* Configure SGMII auto negotiation */
	if (status == ENETPHY_SOK &&
		mii == ENETPHY_MAC_MII_SGMII)
    {
		Dp83tc812_configAutoNeg(hPhy, extendedCfg->sgmiiAutoNegEn);
	}

	/* Configure RGMII clock shift */
	if (status == ENETPHY_SOK &&
		mii == ENETPHY_MAC_MII_RGMII)
	{
		Dp83tc812_configClkShift(hPhy,
								 extendedCfg->txClkShiftEn,
							     extendedCfg->rxClkShiftEn);
	}

	/* Configure interrupts */
	if (status == ENETPHY_SOK)
    {
		Dp83tc812_configIntr(hPhy, extendedCfg->interruptEn);
	}

	/* Set loopback configuration: enable or disable */
    if (status == ENETPHY_SOK)
    {
        Dp83tc812_setLoopbackCfg(hPhy, cfg->loopbackEn);
    }

    return status;
}

static void Dp83tc812_setLoopbackCfg(EnetPhy_Handle hPhy,
                                   bool enable)
{
    bool complete;
	int32_t status;
    uint16_t val;

    ENETTRACE_DBG("PHY %u: %s loopback\n", hPhy->addr, enable ? "enable" : "disable");

	status = EnetPhy_readReg(hPhy, PHY_BMCR, &val);
	if(status != ENETPHY_SOK && enable)
	{
		ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to set loopback mode: could not read reg %u\n", hPhy->addr, PHY_BMCR);
		return;
	}
    if (enable)
    {
		val |= BMCR_LOOPBACK;
    }
    else
    {
		//Normal Mode
        val &= ~BMCR_LOOPBACK;
    }

	/* Specific predefined loopback configuration values are required for
     * normal mode or loopback mode */
    EnetPhy_writeReg(hPhy, PHY_BMCR, val);

    /* Software restart is required after changing LOOPCR register */
    Dp83tc812_reset(hPhy);

    do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);
}


static void Dp83tc812_reset(EnetPhy_Handle hPhy)
{
    /* Global software reset */
    ENETTRACE_DBG("PHY %u: global soft-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83TC812_RESET_CTRL, DP83TC812_SW_RESET, DP83TC812_SW_RESET);
}

static void Dp83tc812_resetHw(EnetPhy_Handle hPhy)
{
    /* Global hardware reset */
    ENETTRACE_DBG("PHY %u: global hard-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83TC812_RESET_CTRL, DP83TC812_HW_RESET, DP83TC812_HW_RESET);
}

static bool Dp83tc812_isResetComplete(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t val;
    bool complete = false;

    /* Reset is complete when RESET bits have self-cleared */
    status = EnetPhy_readReg(hPhy, MII_DP83TC812_RESET_CTRL, &val);
    if (status == ENETPHY_SOK)
    {
        complete = ((val & (DP83TC812_SW_RESET | DP83TC812_HW_RESET)) == 0U);
    }

    ENETTRACE_DBG("PHY %u: global reset is %s complete\n", hPhy->addr, complete ? "" : "not");

    return complete;
}

static int32_t Dp83tc812_readMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t *val)
{
    int32_t status;

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_readReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_VERBOSE_IF(status == ENETPHY_SOK,
						 "PHY %u: read reg %u val 0x%04x\n", hPhy->addr, reg, *val);
    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to read reg %u\n", hPhy->addr, reg);

	return status;
}

static void Dp83tc812_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
    int32_t status;

    ENETTRACE_VERBOSE("PHY %u: write %u val 0x%04x\n", hPhy->addr, reg, val);

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }

    if (status == ENETPHY_SOK)
    {
        EnetPhy_writeReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n", hPhy->addr, reg, val);
}

static void Dp83tc812_setBitsMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
	uint16_t value;
	int32_t status;
	status = Dp83tc812_readMmd(hPhy, devad, reg, &value);
	if (status == ENETPHY_SOK)
    {
		value = value | val;
		Dp83tc812_writeMmd(hPhy, devad, reg, value);
	}
}

static void Dp83tc812_readStraps(EnetPhy_Handle hPhy, Dp83tc812_operationMode msMode)
{
	uint16_t strap;
	int32_t status;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_STRAP, &strap);
	if (status != ENETPHY_SOK)
		return;

	ENETTRACE_DBG("PHY %u: Strap register is 0x%X\n", hPhy->addr, strap);

	if(msMode == DP83TC812_STRAP_MODE)
	{
		if (strap & DP83TC812_MASTER_MODE_EN)
		{
			dp83tc812_params.is_master = true;
			ENETTRACE_DBG("PHY %u: Strap: Master Mode enabled\n", hPhy->addr);
		}
		else
		{
			dp83tc812_params.is_master = false;
			ENETTRACE_DBG("PHY %u: Strap: Slave Mode enabled\n", hPhy->addr);
		}
	}

	if (strap & DP83TC812_RGMII_IS_EN)
	{
		ENETTRACE_DBG("PHY %u: Strap: RGMII Mode enabled\n", hPhy->addr);
		if (((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_TX_RX_SHIFT_EN) ||
			((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_TX_SHIFT_EN))
			ENETTRACE_DBG("PHY %u: Strap: TX Clock Shift enabled\n", hPhy->addr);
		if (((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_TX_RX_SHIFT_EN) ||
			((strap & DP83TC812_TX_RX_SHIFT) == DP83TC812_RX_SHIFT_EN))
			ENETTRACE_DBG("PHY %u: Strap: RX Clock Shift enabled\n", hPhy->addr);
	}
	else
	{
		ENETTRACE_DBG("PHY %u: Strap: SGMII Mode enabled\n", hPhy->addr);
	}

};

static void Dp83tc812_writeSeq(EnetPhy_Handle hPhy, const struct dp83tc812_init_reg *init_data, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		Dp83tc812_writeMmd(hPhy, init_data[i].mmd, init_data[i].reg, init_data[i].val);
	}
}

static void Dp83tc812_chipInit(EnetPhy_Handle hPhy)
{
	bool complete = false;

	/* Perform a hardware reset prior to configuration */
	Dp83tc812_resetHw(hPhy);
	do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);

    /* placeholder for check for strap mod, do not remove
     *
     *
     */

	Dp83tc812_writeSeq(hPhy, dp83tc812_master_init, sizeof(dp83tc812_master_init) / sizeof(dp83tc812_master_init[0]));
	ENETTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS1.0 Master\n", hPhy->addr);

	/* Perform a software reset to restart the PHY with the updated configuration */
	Dp83tc812_reset(hPhy);
	do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);

	/* Enable transmitter */
	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, 0x0523U, 0x0000U);

    /* placeholder for check for strap mod, do not remove
     *
     *
     * 
     * 
     * 
     */
}

static void Dp83tc812_setMiiMode(EnetPhy_Handle hPhy, EnetPhy_Mii mii)
{
    uint16_t rgmii_val = 0U;
	uint16_t sgmii_val = 0U;
	int32_t status = ENETPHY_SOK;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_CTRL, &rgmii_val);
	if(status != ENETPHY_SOK)
		return;
	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, &sgmii_val);
	if(status != ENETPHY_SOK)
		return;

    if (mii == ENETPHY_MAC_MII_RGMII)
    {
		rgmii_val |= DP83TC812_RGMII_EN;
		sgmii_val &= ~DP83TC812_SGMII_EN;
		ENETTRACE_DBG("PHY %u: RGMII Mode enabled\n", hPhy->addr);
    }
	else if (mii == ENETPHY_MAC_MII_SGMII)
	{
		rgmii_val &= ~DP83TC812_RGMII_EN;
		sgmii_val |= DP83TC812_SGMII_EN;
		ENETTRACE_DBG("PHY %u: SGMII Mode enabled\n", hPhy->addr);
	}

	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_CTRL, rgmii_val);
	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, sgmii_val);
}

static void Dp83tc812_configAutoNeg(EnetPhy_Handle hPhy, bool sgmiiAutoNegEn)
{
	uint16_t val = 0U;
	int32_t status = ENETPHY_SOK;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, &val);

	if(status != ENETPHY_SOK)
		return;

	if(sgmiiAutoNegEn)
	{
		val |= DP83TC812_SGMII_AUTO_NEG_EN;
		ENETTRACE_DBG("PHY %u: SGMII Auto Negotiation enabled\n", hPhy->addr);
	}
	else
	{
		val &= ~DP83TC812_SGMII_AUTO_NEG_EN;
		ENETTRACE_DBG("PHY %u: SGMII Auto Negotiation disabled\n", hPhy->addr);
	}

	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, val);
}

/*clock shift function*/
static void Dp83tc812_configClkShift(EnetPhy_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn)
{
	uint16_t val = 0U;
	int32_t status = ENETPHY_SOK;

	status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_ID_CTRL, &val);

	if(status != ENETPHY_SOK)
		return;

	if (!txClkShiftEn)
		val &= ~DP83TC812_TX_CLK_SHIFT;
	else
		val |= DP83TC812_TX_CLK_SHIFT;
	if (!rxClkShiftEn)
		val &= ~DP83TC812_RX_CLK_SHIFT;
	else
		val |= DP83TC812_RX_CLK_SHIFT;

	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_ID_CTRL, val);

	ENETTRACE_DBG("PHY %u: RGMII TX Clock Shift %s\n", hPhy->addr, txClkShiftEn ? "enabled" : "disabled");
	ENETTRACE_DBG("PHY %u: RGMII RX Clock Shift %s\n", hPhy->addr, rxClkShiftEn ? "enabled" : "disabled");
}

static void Dp83tc812_configIntr(EnetPhy_Handle hPhy, bool intrEn)
 {
	uint16_t reg_val;
	int32_t status;

	if (intrEn) {
		ENETTRACE_DBG("PHY %u: Enable interrupts\n", hPhy->addr);
		status = EnetPhy_readReg(hPhy, MII_DP83TC812_INT_STAT1, &reg_val);
        if (status != ENETPHY_SOK)
			return;

        reg_val |= (DP83TC812_ANEG_COMPLETE_INT_EN |
					DP83TC812_ESD_EVENT_INT_EN |
					DP83TC812_LINK_STAT_INT_EN |
                    DP83TC812_ENERGY_DET_INT_EN |
                    DP83TC812_LINK_QUAL_INT_EN);

		EnetPhy_writeReg(hPhy, MII_DP83TC812_INT_STAT1, reg_val);

		status = EnetPhy_readReg(hPhy, MII_DP83TC812_INT_STAT2, &reg_val);
        if (status != ENETPHY_SOK)
			return;

        reg_val |= (DP83TC812_SLEEP_MODE_INT_EN |
					DP83TC812_OVERTEMP_INT_EN |
					DP83TC812_OVERVOLTAGE_INT_EN |
					DP83TC812_UNDERVOLTAGE_INT_EN);

        EnetPhy_writeReg(hPhy, MII_DP83TC812_INT_STAT2, reg_val);

        status = EnetPhy_readReg(hPhy, MII_DP83TC812_INT_STAT3, &reg_val);
        if (status != ENETPHY_SOK)
			return;

        reg_val |= (DP83TC812_LPS_INT_EN |
					DP83TC812_WAKE_REQ_EN |
					DP83TC812_NO_FRAME_INT_EN |
					DP83TC812_POR_DONE_INT_EN);

		EnetPhy_writeReg(hPhy, MII_DP83TC812_INT_STAT3, reg_val);

    }
	else {
		ENETTRACE_DBG("PHY %u: Disable interrupts\n", hPhy->addr);
		EnetPhy_writeReg(hPhy, MII_DP83TC812_INT_STAT1, 0U);

        EnetPhy_writeReg(hPhy, MII_DP83TC812_INT_STAT2, 0U);

        EnetPhy_writeReg(hPhy, MII_DP83TC812_INT_STAT3, 0U);
    }
}

static void Dp83tc812_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    EnetUtils_printf("PHY %u: BMCR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    EnetUtils_printf("PHY %u: BMSR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    EnetUtils_printf("PHY %u: PHYIDR1 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    EnetUtils_printf("PHY %u: PHYIDR2 = 0x%04x\n", phyAddr, val);

    EnetPhy_readReg(hPhy, DP83TC812_PHYSTS, &val);
    EnetUtils_printf("PHY %u: PHYSTS  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_PHYCR, &val);
    EnetUtils_printf("PHY %u: PHYCR   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_MISR1, &val);
    EnetUtils_printf("PHY %u: MISR1   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_MISR2, &val);
    EnetUtils_printf("PHY %u: MISR2   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_RECR, &val);
    EnetUtils_printf("PHY %u: RECR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_BISCR, &val);
    EnetUtils_printf("PHY %u: BISCR   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_MISR3, &val);
    EnetUtils_printf("PHY %u: MISR3   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_REG19, &val);
    EnetUtils_printf("PHY %u: REG19   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_CDCR, &val);
    EnetUtils_printf("PHY %u: CDCR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83TC812_PHYRCR, &val);
    EnetUtils_printf("PHY %u: PHYRCR  = 0x%04x\n", phyAddr, val);

	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_SGMII_CTRL, &val);
	EnetUtils_printf("PHY %u: SGMII_CTRL      = 0x%04x\n", phyAddr, val);
	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_CTRL, &val);
	EnetUtils_printf("PHY %u: RGMII_CTRL      = 0x%04x\n", phyAddr, val);
	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC812_RGMII_ID_CTRL, &val);
	EnetUtils_printf("PHY %u: RGMII_ID_CTRL   = 0x%04x\n", phyAddr, val);
	Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR_MMD1, DP83TC812_MMD1_PMA_CTRL_2, &val);
	EnetUtils_printf("PHY %u: MMD1_PMA_CTRL_2 = 0x%04x\n", phyAddr, val);
}
