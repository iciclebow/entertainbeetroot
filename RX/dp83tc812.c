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
#include <string.h>
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

/*Keeping the variable names to hopefully get a quick test up and running*/
#define DP83TC812_OUI                         (0x080028U)	//PHYIDR2 Register: 15-10 Unique Identifier 2 R 28h 
/*I dont care about the 812*/
//#define DP83TC812_MODEL                       (0x27U)
#define DP83TC812_MODEL                       (0x2EU)		//PHYIDR2 Register: 9-4 Model Number R 2Eh
/* I only have one Phy and one revision I care about.*/
// #define DP83TC812_REV_CS1					  (0U)
// #define DP83TC812_REV_CS2			          (1U)
#define DP83TC815_SEC_NSEC                   (1000000000ULL)

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

static int32_t Dp83tc812_writeMmdChk(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_writeMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_setBitsMmd(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val);

static void Dp83tc812_readStraps(EnetPhy_Handle hPhy, Dp83tc812_operationMode msMode);

static void Dp83tc812_writeSeq(EnetPhy_Handle hPhy, const struct dp83tc812_init_reg *init_data, int size);

static void Dp83tc812_chipInit(EnetPhy_Handle hPhy);

static void Dp83tc812_setMiiMode(EnetPhy_Handle hPhy, EnetPhy_Mii mii);

static void Dp83tc812_configAutoNeg(EnetPhy_Handle hPhy, bool sgmiiAutoNegEn);

static void Dp83tc812_configClkShift(EnetPhy_Handle hPhy, bool txClkShiftEn, bool rxClkShiftEn);

static void Dp83tc812_configIntr(EnetPhy_Handle hPhy, bool intrEn);

static void Dp83tc812_printRegs(EnetPhy_Handle hPhy);

static uint16_t Dp83tc815_readU16(const uint8_t *buf, bool littleEndian);

static int32_t Dp83tc815_getPayload(const uint8_t *frame,
                                    uint32_t frameSize,
                                    uint16_t *ethType,
                                    const uint8_t **payload,
                                    uint32_t *payloadSize);

static int32_t Dp83tc815_decodeRxPsf(const uint8_t *payload,
                                     uint32_t payloadSize,
                                     uint8_t port,
                                     Dp83tc815_RxTsInfo *outTs);

static void Dp83tc815_rxTsPush(const Dp83tc815_RxTsInfo *inTs);

static int32_t Dp83tc815_enablePtpClock(EnetPhy_Handle hPhy);

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
    .printRegs          = Dp83tc812_printRegs,
};

/* PHY Device Attributes */
static struct dp83tc812_privParams dp83tc812_params = {
	.chip = -1,
	.is_master = false,
};

static Dp83tc815_RxTsQ gDp83tc815RxTsQ;
static Dp83tc815_RxTsStats gDp83tc815RxTsStats;
static EnetPhy_Handle gDp83tc815PtpPhy;
static bool gDp83tc815PtpClockEnabled;


/*We only need one master and one slave init script because we are only using one chip revision*/
/*! \brief Chip specific init scripts */
static const struct dp83tc812_init_reg dp83tc812_master_init[] = {		//renamed to remove the _cs1
		/*blank placeholder, will be added in final implementation*/
};

static const struct dp83tc812_init_reg dp83tc812_slave_init[] = { 	//renamed to remove the _cs1
		/*blank placeholder, will be added in final implementation*/
};

/* static const struct dp83tc812_init_reg dp83tc812_cs2_master_init[] = {
		{0x1F,0x0523,0x0001},
		{0x01,0x0834,0xC001},
		{0x1F,0x081C,0x0FE2},
		{0x1F,0x0872,0x0300},
		{0x1F,0x0879,0x0F00},
		{0x1F,0x0806,0x2952},
		{0x1F,0x0807,0x3361},
		{0x1F,0x0808,0x3D7B},
		{0x1F,0x083E,0x045F},
		{0x1F,0x0834,0x8000},
		{0x1F,0x0862,0x00E8},
		{0x1F,0x0896,0x32CB},
		{0x1F,0x003E,0x0009},
};

static const struct dp83tc812_init_reg dp83tc812_cs2_slave_init[] = {
		{0x1F,0x0523,0x0001},
		{0x01,0x0834,0x8001},
		{0x1F,0x0873,0x0821},
		{0x1F,0x0896,0x22FF},
		{0x1F,0x089E,0x0000},
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

static uint16_t Dp83tc815_readU16(const uint8_t *buf, bool littleEndian)
{
    uint16_t val;

    if (littleEndian)
    {
        val = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    else
    {
        val = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }

    return val;
}

static int32_t Dp83tc815_getPayload(const uint8_t *frame,
                                    uint32_t frameSize,
                                    uint16_t *ethType,
                                    const uint8_t **payload,
                                    uint32_t *payloadSize)
{
    uint16_t type;
    uint32_t offset = 14U;

    if ((frame == NULL) || (ethType == NULL) || (payload == NULL) || (payloadSize == NULL))
    {
        return ENETPHY_EBADARGS;
    }

    if (frameSize < 14U)
    {
        return ENETPHY_EINVALIDPARAMS;
    }

    type = ((uint16_t)frame[12] << 8) | (uint16_t)frame[13];
    if (type == 0x8100U)
    {
        if (frameSize < 18U)
        {
            return ENETPHY_EINVALIDPARAMS;
        }

        type = ((uint16_t)frame[16] << 8) | (uint16_t)frame[17];
        offset = 18U;
    }

    if (frameSize < offset)
    {
        return ENETPHY_EINVALIDPARAMS;
    }

    *ethType = type;
    *payload = frame + offset;
    *payloadSize = frameSize - offset;

    return ENETPHY_SOK;
}

static int32_t Dp83tc815_decodeRxPsf(const uint8_t *payload,
                                     uint32_t payloadSize,
                                     uint8_t port,
                                     Dp83tc815_RxTsInfo *outTs)
{
    uint16_t type;
    uint16_t words[DP83TC815_PSF_WORDS];
    Dp83tc815_PhyRxTs raw;
    bool littleEndian;
    uint8_t overflow;
    uint64_t sec;
    uint64_t nsec;
    uint32_t i;

    if ((payload == NULL) || (outTs == NULL))
    {
        return ENETPHY_EBADARGS;
    }

    if (payloadSize < (DP83TC815_PSF_SKIP_BYTES + DP83TC815_PSF_TYPE_BYTES + DP83TC815_PSF_DATA_BYTES))
    {
        return ENETPHY_EINVALIDPARAMS;
    }

    type = Dp83tc815_readU16(payload + DP83TC815_PSF_SKIP_BYTES, true);
    if ((type & 0xF000U) == DP83TC815_PSF_TYPE_RX)
    {
        littleEndian = true;
    }
    else
    {
        type = Dp83tc815_readU16(payload + DP83TC815_PSF_SKIP_BYTES, false);
        if ((type & 0xF000U) != DP83TC815_PSF_TYPE_RX)
        {
            return ENETPHY_EINVALIDPARAMS;
        }

        littleEndian = false;
    }

    for (i = 0U; i < DP83TC815_PSF_WORDS; i++)
    {
        words[i] = Dp83tc815_readU16(payload + DP83TC815_PSF_SKIP_BYTES + DP83TC815_PSF_TYPE_BYTES + (2U * i), littleEndian);
    }

    raw.nsLow = words[0];
    raw.nsHigh = words[1];
    raw.secLow = words[2];
    raw.secHigh = words[3];
    raw.seqId = words[4];
    raw.msgTypeHash = words[5];

    overflow = (uint8_t)((raw.nsHigh >> 14) & 0x3U);
    nsec = (uint64_t)raw.nsLow | ((uint64_t)(raw.nsHigh & 0x3FFFU) << 16);
    sec = (uint64_t)raw.secLow | ((uint64_t)raw.secHigh << 16);
    sec += overflow;

    if (nsec >= DP83TC815_SEC_NSEC)
    {
        return ENETPHY_EINVALIDPARAMS;
    }

    memset(outTs, 0, sizeof(*outTs));
    outTs->ts = (sec * DP83TC815_SEC_NSEC) + nsec;
    outTs->seqId = raw.seqId;
    outTs->msgType = (uint8_t)((raw.msgTypeHash >> 12) & 0xFU);
    outTs->hash = raw.msgTypeHash & 0x0FFFU;
    outTs->domain = DP83TC815_PSF_DOMAIN_ANY;
    outTs->port = port;
    outTs->valid = 1U;

    return ENETPHY_SOK;
}

static void Dp83tc815_rxTsPush(const Dp83tc815_RxTsInfo *inTs)
{
    Dp83tc815_RxTsInfo *slot;

    slot = &gDp83tc815RxTsQ.q[gDp83tc815RxTsQ.wr];
    if (slot->valid != 0U)
    {
        gDp83tc815RxTsStats.queueOverflow++;
    }

    *slot = *inTs;
    gDp83tc815RxTsQ.pushSeq++;
    slot->pushSeq = gDp83tc815RxTsQ.pushSeq;

    gDp83tc815RxTsQ.wr = (uint16_t)((gDp83tc815RxTsQ.wr + 1U) % DP83TC815_RXTS_Q_SIZE);
}

void Dp83tc815_RxTsReset(void)
{
    memset(&gDp83tc815RxTsQ, 0, sizeof(gDp83tc815RxTsQ));
    memset(&gDp83tc815RxTsStats, 0, sizeof(gDp83tc815RxTsStats));
}

void Dp83tc815_RxTsGetStats(Dp83tc815_RxTsStats *stats)
{
    if (stats != NULL)
    {
        *stats = gDp83tc815RxTsStats;
    }
}

int32_t Dp83tc815_PsfIngestFrame(const uint8_t *frame,
                                 uint32_t frameSize,
                                 uint8_t port)
{
    uint16_t ethType;
    const uint8_t *payload;
    uint32_t payloadSize;
    Dp83tc815_RxTsInfo tsInfo;
    int32_t status;

    status = Dp83tc815_getPayload(frame, frameSize, &ethType, &payload, &payloadSize);
    if (status != ENETPHY_SOK)
    {
        return status;
    }

    if (ethType != DP83TC815_PSF_ETHERTYPE)
    {
        return ENETPHY_EINVALIDPARAMS;
    }

    gDp83tc815RxTsStats.psfSeen++;

    status = Dp83tc815_decodeRxPsf(payload, payloadSize, port, &tsInfo);
    if (status == ENETPHY_SOK)
    {
        Dp83tc815_rxTsPush(&tsInfo);
        gDp83tc815RxTsStats.psfParsedOk++;
    }

    return status;
}

int32_t Dp83tc815_RxTsLookup(uint8_t port,
                             uint8_t msgType,
                             uint16_t seqId,
                             uint8_t domain,
                             uint64_t *ts)
{
    int32_t status = ENETPHY_EFAIL;
    uint32_t now = gDp83tc815RxTsQ.pushSeq;
    uint32_t oldestPush = 0xFFFFFFFFU;
    Dp83tc815_RxTsInfo *best = NULL;
    uint32_t i;

    if (ts == NULL)
    {
        return ENETPHY_EBADARGS;
    }

    for (i = 0U; i < DP83TC815_RXTS_Q_SIZE; i++)
    {
        Dp83tc815_RxTsInfo *entry = &gDp83tc815RxTsQ.q[i];
        uint32_t age;

        if (entry->valid == 0U)
        {
            continue;
        }

        age = now - entry->pushSeq;
        if (age > DP83TC815_RXTS_STALE_WINDOW)
        {
            entry->valid = 0U;
            gDp83tc815RxTsStats.staleDrop++;
            continue;
        }

        if ((entry->port != port) ||
            (entry->msgType != msgType) ||
            (entry->seqId != seqId))
        {
            continue;
        }

        if ((entry->domain != DP83TC815_PSF_DOMAIN_ANY) &&
            (entry->domain != domain))
        {
            continue;
        }

        if (entry->pushSeq < oldestPush)
        {
            oldestPush = entry->pushSeq;
            best = entry;
        }
    }

    if (best != NULL)
    {
        *ts = best->ts;
        best->valid = 0U;
        gDp83tc815RxTsStats.lookupHit++;
        status = ENETPHY_SOK;
    }
    else
    {
        gDp83tc815RxTsStats.lookupMiss++;
    }

    return status;
}

static int32_t Dp83tc815_enablePtpClock(EnetPhy_Handle hPhy)
{
    int32_t status;
    uint16_t ptpCtl = 0U;

    status = Dp83tc812_writeMmdChk(hPhy,
                                   DP83TC812_DEVADDR,
                                   DP83TC815_PTP_CTL,
                                   DP83TC815_PTP_RESET);
    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_writeMmdChk(hPhy,
                                       DP83TC812_DEVADDR,
                                       DP83TC815_PTP_CTL,
                                       0U);
    }
    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_readMmd(hPhy, DP83TC812_DEVADDR, DP83TC815_PTP_CTL, &ptpCtl);
    }
    if (status == ENETPHY_SOK)
    {
        ptpCtl |= DP83TC815_PTP_ENABLE;
        ptpCtl &= (uint16_t)~DP83TC815_PTP_DISABLE;
        status = Dp83tc812_writeMmdChk(hPhy, DP83TC812_DEVADDR, DP83TC815_PTP_CTL, ptpCtl);
    }

    if (status == ENETPHY_SOK)
    {
        gDp83tc815PtpClockEnabled = true;
    }

    return status;
}

int32_t Dp83tc815_SetPtpTime(uint64_t ts64)
{
    uint64_t sec = ts64 / DP83TC815_SEC_NSEC;
    uint32_t nsec = (uint32_t)(ts64 % DP83TC815_SEC_NSEC);
    int32_t status = ENETPHY_SOK;
    uint16_t ptpCtl = 0U;

    if (gDp83tc815PtpPhy == NULL)
    {
        return ENETPHY_EFAIL;
    }

    if ((sec >> 32) != 0U)
    {
        return ENETPHY_EINVALIDPARAMS;
    }

    if (!gDp83tc815PtpClockEnabled)
    {
        status = Dp83tc815_enablePtpClock(gDp83tc815PtpPhy);
    }

    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_writeMmdChk(gDp83tc815PtpPhy,
                                       DP83TC812_DEVADDR,
                                       DP83TC815_PTP_TDR,
                                       (uint16_t)(nsec & 0xFFFFU));
    }
    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_writeMmdChk(gDp83tc815PtpPhy,
                                       DP83TC812_DEVADDR,
                                       DP83TC815_PTP_TDR,
                                       (uint16_t)((nsec >> 16) & 0x3FFFU));
    }
    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_writeMmdChk(gDp83tc815PtpPhy,
                                       DP83TC812_DEVADDR,
                                       DP83TC815_PTP_TDR,
                                       (uint16_t)(sec & 0xFFFFU));
    }
    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_writeMmdChk(gDp83tc815PtpPhy,
                                       DP83TC812_DEVADDR,
                                       DP83TC815_PTP_TDR,
                                       (uint16_t)((sec >> 16) & 0xFFFFU));
    }
    if (status == ENETPHY_SOK)
    {
        status = Dp83tc812_readMmd(gDp83tc815PtpPhy, DP83TC812_DEVADDR, DP83TC815_PTP_CTL, &ptpCtl);
    }
    if (status == ENETPHY_SOK)
    {
        ptpCtl |= DP83TC815_PTP_LOAD_CLK;
        status = Dp83tc812_writeMmdChk(gDp83tc815PtpPhy, DP83TC812_DEVADDR, DP83TC815_PTP_CTL, ptpCtl);
    }

    return status;
}

/*This function only needs to check for Vendor ID and Model ID since the hardware this will be running on will only ever use one type and revion*/
static bool Dp83tc812_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

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

    if (status == ENETPHY_SOK)
    {
        gDp83tc815PtpPhy = hPhy;
        gDp83tc815PtpClockEnabled = false;
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
    if (gDp83tc815PtpPhy == hPhy)
    {
        gDp83tc815PtpClockEnabled = false;
    }
}

static void Dp83tc812_resetHw(EnetPhy_Handle hPhy)
{
    /* Global hardware reset */
    ENETTRACE_DBG("PHY %u: global hard-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, MII_DP83TC812_RESET_CTRL, DP83TC812_HW_RESET, DP83TC812_HW_RESET);
    if (gDp83tc815PtpPhy == hPhy)
    {
        gDp83tc815PtpClockEnabled = false;
    }
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

static int32_t Dp83tc812_writeMmdChk(EnetPhy_Handle hPhy, uint16_t devad, uint32_t reg, uint16_t val)
{
    int32_t status;

    status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_ADDR);
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, reg);
    }
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_CR, devad | MMD_CR_DATA_NOPOSTINC);
    }
    if (status == ENETPHY_SOK)
    {
        status = EnetPhy_writeReg(hPhy, PHY_MMD_DR, val);
    }

    ENETTRACE_ERR_IF(status != ENETPHY_SOK,
                     "PHY %u: failed to write reg %u val 0x%04x\n", hPhy->addr, reg, val);

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

	// /* Apply chip-specific configuration */
	// switch (dp83tc812_params.chip) {
	// 	case DP83TC812_CS1:
	// 		if (dp83tc812_params.is_master){
	// 			Dp83tc812_writeSeq(hPhy, dp83tc812_cs1_master_init,
	// 							   sizeof(dp83tc812_cs1_master_init)/sizeof(dp83tc812_cs1_master_init[0]));
	// 			ENETTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS1.0 Master\n", hPhy->addr);}
	// 		else{
	// 			Dp83tc812_writeSeq(hPhy, dp83tc812_cs1_slave_init,
	// 							   sizeof(dp83tc812_cs1_slave_init)/sizeof(dp83tc812_cs1_slave_init[0]));
	// 			ENETTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS1.0 Slave\n", hPhy->addr);}
	// 		break;
	// 	case DP83TC812_CS2:
	// 		if (dp83tc812_params.is_master){
	// 			Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_master_init,
	// 							   sizeof(dp83tc812_cs2_master_init)/sizeof(dp83tc812_cs2_master_init[0]));
	// 			ENETTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS2.0 Master\n", hPhy->addr);}
	// 		else{
	// 			Dp83tc812_writeSeq(hPhy, dp83tc812_cs2_slave_init,
	// 							   sizeof(dp83tc812_cs2_slave_init)/sizeof(dp83tc812_cs2_slave_init[0]));
	// 			ENETTRACE_DBG("PHY %u: Applying configuration for DP83TC812 CS2.0 Slave\n", hPhy->addr);}
	// 		break;
	// 	default:
	// 		ENETTRACE_DBG("PHY %u: No supported DP83TC812 Chip. Skipping chip-specific configuration!\n", hPhy->addr);
	// 		break;
	// };

	/* Perform a software reset to restart the PHY with the updated configuration */
	Dp83tc812_reset(hPhy);
	do
    {
        complete = Dp83tc812_isResetComplete(hPhy);
    }
    while (!complete);

	/* Enable transmitter */
	Dp83tc812_writeMmd(hPhy, DP83TC812_DEVADDR, 0x0523U, 0x0000U);

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
