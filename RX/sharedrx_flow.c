/* ======================= PSF Debug: compile-time controls ======================= */
/*
 * These controls apply to the default RX-flow debug path that observes frames
 * after CPSW DMA reception.
 *
 * Terminology note:
 * - Here, PHY Status Frames (PSFs) are always observed on the host RX side, even when
 *   they report timestamps caused by outbound traffic.
 *
 * Debug groups:
 * - RX_FRAME_DEBUG:
 *   periodic overview/statistics logging for received traffic classification
 * - RX_FRAME_DUMP:
 *   optional hexdump of selected received frames, intended only for short
 *   format-inspection runs
 * - RX_DROP_STATS:
 *   periodic CPSW drop/error counter logging
 */

/* Set to 1 to include periodic RX frame overview logs in default flow */
#ifndef ENETAPP_ENABLE_RX_FRAME_DEBUG
#define ENETAPP_ENABLE_RX_FRAME_DEBUG            (0U)
#endif

/* Period for summary prints from the RX traffic debug path. */
#ifndef ENETAPP_RX_FRAME_DEBUG_PRINT_INTERVAL_MS
#define ENETAPP_RX_FRAME_DEBUG_PRINT_INTERVAL_MS (5000U)
#endif

/* Optional frame-sampling decimation before classification/debug accounting. */
#ifndef ENETAPP_RX_FRAME_DEBUG_SAMPLE_EVERY_N_FRAMES
#define ENETAPP_RX_FRAME_DEBUG_SAMPLE_EVERY_N_FRAMES (1U)
#endif

/* Set to 1 to allow hexdumps of selected received frames. */
#ifndef ENETAPP_ENABLE_RX_FRAME_DUMP
#define ENETAPP_ENABLE_RX_FRAME_DUMP               (0U)
#endif

/* Maximum number of bytes printed for one received-frame dump. */
#ifndef ENETAPP_RX_FRAME_DUMP_MAX_BYTES
#define ENETAPP_RX_FRAME_DUMP_MAX_BYTES            (64U)
#endif

/* Dump only one out of every N eligible received frames. */
#ifndef ENETAPP_RX_FRAME_DUMP_DECIMATION
#define ENETAPP_RX_FRAME_DUMP_DECIMATION           (1U)
#endif

/* Minimum time between frame dump prints. */
#ifndef ENETAPP_RX_FRAME_DUMP_INTERVAL_MS
#define ENETAPP_RX_FRAME_DUMP_INTERVAL_MS          (1000U)
#endif