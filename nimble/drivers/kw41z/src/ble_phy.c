/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "syscfg/syscfg.h"
#include "os/os.h"
#include "ble/xcvr.h"
#include "nimble/ble.h"
#include "nimble/nimble_opt.h"
#include "nimble/nimble_npl.h"
#include "controller/ble_phy.h"
#include "controller/ble_phy_trace.h"
#include "controller/ble_ll.h"
#include "nrfx.h"
#if MYNEWT
#include "mcu/nrf52_clock.h"
#include "mcu/cmsis_nvic.h"
#include "hal/hal_gpio.h"
#else
// #include "core_cm4.h"
#endif

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
#if !MYNEWT_VAL_CHOICE(MCU_TARGET, nRF52840) && !MYNEWT_VAL_CHOICE(MCU_TARGET, nRF52811)
#error LE Coded PHY can only be enabled on nRF52811 or nRF52840
#endif
#endif

/*
 * NOTE: This code uses a couple of PPI channels so care should be taken when
 *       using PPI somewhere else.
 *
 * Pre-programmed channels: CH20, CH21, CH23, CH25, CH31
 * Regular channels: CH4, CH5 and optionally CH17, CH18, CH19
 *  - CH4 = cancel wfr timer on address match
 *  - CH5 = disable radio on wfr timer expiry
 *  - CH17 = (optional) gpio debug for radio ramp-up
 *  - CH18 = (optional) gpio debug for wfr timer RX enabled
 *  - CH19 = (optional) gpio debug for wfr timer radio disabled
 *
 */

/* XXX: 4) Make sure RF is higher priority interrupt than schedule */

/*
 * XXX: Maximum possible transmit time is 1 msec for a 60ppm crystal
 * and 16ms for a 30ppm crystal! We need to limit PDU size based on
 * crystal accuracy. Look at this in the spec.
 */

/* XXX: private header file? */
extern uint8_t g_nrf_num_irks;
extern uint32_t g_nrf_irk_list[];

/* To disable all radio interrupts */
#define NRF_RADIO_IRQ_MASK_ALL  (0x34FF)

/*
 * We configure the nrf with a 1 byte S0 field, 8 bit length field, and
 * zero bit S1 field. The preamble is 8 bits long.
 */
#define NRF_LFLEN_BITS          (8)
#define NRF_S0LEN               (1)
#define NRF_S1LEN_BITS          (0)
#define NRF_CILEN_BITS          (2)
#define NRF_TERMLEN_BITS        (3)

/* Maximum length of frames */
#define NRF_MAXLEN              (255)
#define NRF_BALEN               (3)     /* For base address of 3 bytes */

/* NRF_RADIO->PCNF0 configuration values */
#define NRF_PCNF0               (NRF_LFLEN_BITS << RADIO_PCNF0_LFLEN_Pos) | \
                                (RADIO_PCNF0_S1INCL_Msk) | \
                                (NRF_S0LEN << RADIO_PCNF0_S0LEN_Pos) | \
                                (NRF_S1LEN_BITS << RADIO_PCNF0_S1LEN_Pos)
#define NRF_PCNF0_1M            (NRF_PCNF0) | \
                                (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos)
#define NRF_PCNF0_2M            (NRF_PCNF0) | \
                                (RADIO_PCNF0_PLEN_16bit << RADIO_PCNF0_PLEN_Pos)
#define NRF_PCNF0_CODED         (NRF_PCNF0) | \
                                (RADIO_PCNF0_PLEN_LongRange << RADIO_PCNF0_PLEN_Pos) | \
                                (NRF_CILEN_BITS << RADIO_PCNF0_CILEN_Pos) | \
                                (NRF_TERMLEN_BITS << RADIO_PCNF0_TERMLEN_Pos)

/* BLE PHY data structure */
struct ble_phy_obj
{
    uint8_t phy_stats_initialized;
    int8_t  phy_txpwr_dbm;
    uint8_t phy_chan;
    uint8_t phy_state;
    uint8_t phy_transition;
    uint8_t phy_transition_late;
    uint8_t phy_rx_started;
    uint8_t phy_encrypted;
    uint8_t phy_privacy;
    uint8_t phy_tx_pyld_len;
    uint8_t phy_cur_phy_mode;
    uint8_t phy_tx_phy_mode;
    uint8_t phy_rx_phy_mode;
    uint8_t phy_bcc_offset;
    int8_t  rx_pwr_compensation;
    uint32_t phy_aar_scratch;
    uint32_t phy_access_address;
    struct ble_mbuf_hdr rxhdr;
    void *txend_arg;
    ble_phy_tx_end_func txend_cb;
    uint32_t phy_start_cputime;
};
struct ble_phy_obj g_ble_phy_data;

/* XXX: if 27 byte packets desired we can make this smaller */
/* Global transmit/receive buffer */
// static uint32_t g_ble_phy_tx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
// static uint32_t g_ble_phy_rx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
/* Make sure word-aligned for faster copies */
static uint32_t g_ble_phy_enc_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
#endif

/* RF center frequency for each channel index (offset from 2400 MHz) */
// static const uint8_t g_ble_phy_chan_freq[BLE_PHY_NUM_CHANS] = {
//      4,  6,  8, 10, 12, 14, 16, 18, 20, 22, /* 0-9 */
//     24, 28, 30, 32, 34, 36, 38, 40, 42, 44, /* 10-19 */
//     46, 48, 50, 52, 54, 56, 58, 60, 62, 64, /* 20-29 */
//     66, 68, 70, 72, 74, 76, 78,  2, 26, 80, /* 30-39 */
// };

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
/* packet start offsets (in usecs) */
static const uint16_t g_ble_phy_mode_pkt_start_off[BLE_PHY_NUM_MODE] = { 376, 40, 24, 376 };
#endif

/* Various radio timings */
/* Radio ramp-up times in usecs (fast mode) */
#define BLE_PHY_T_TXENFAST      (XCVR_TX_RADIO_RAMPUP_USECS)
#define BLE_PHY_T_RXENFAST      (XCVR_RX_RADIO_RAMPUP_USECS)
/* delay between EVENTS_READY and start of tx */
// static const uint8_t g_ble_phy_t_txdelay[BLE_PHY_NUM_MODE] = { 5, 4, 3, 5 };
/* delay between EVENTS_END and end of txd packet */
// static const uint8_t g_ble_phy_t_txenddelay[BLE_PHY_NUM_MODE] = { 9, 4, 3, 3 };
/* delay between rxd access address (w/ TERM1 for coded) and EVENTS_ADDRESS */
// static const uint8_t g_ble_phy_t_rxaddrdelay[BLE_PHY_NUM_MODE] = { 17, 6, 2, 17 };
/* delay between end of rxd packet and EVENTS_END */
// static const uint8_t g_ble_phy_t_rxenddelay[BLE_PHY_NUM_MODE] = { 27, 6, 2, 22 };

/* Statistics */
STATS_SECT_START(ble_phy_stats)
    STATS_SECT_ENTRY(phy_isrs)
    STATS_SECT_ENTRY(tx_good)
    STATS_SECT_ENTRY(tx_fail)
    STATS_SECT_ENTRY(tx_late)
    STATS_SECT_ENTRY(tx_bytes)
    STATS_SECT_ENTRY(rx_starts)
    STATS_SECT_ENTRY(rx_aborts)
    STATS_SECT_ENTRY(rx_valid)
    STATS_SECT_ENTRY(rx_crc_err)
    STATS_SECT_ENTRY(rx_late)
    STATS_SECT_ENTRY(radio_state_errs)
    STATS_SECT_ENTRY(rx_hw_err)
    STATS_SECT_ENTRY(tx_hw_err)
STATS_SECT_END
STATS_SECT_DECL(ble_phy_stats) ble_phy_stats;

STATS_NAME_START(ble_phy_stats)
    STATS_NAME(ble_phy_stats, phy_isrs)
    STATS_NAME(ble_phy_stats, tx_good)
    STATS_NAME(ble_phy_stats, tx_fail)
    STATS_NAME(ble_phy_stats, tx_late)
    STATS_NAME(ble_phy_stats, tx_bytes)
    STATS_NAME(ble_phy_stats, rx_starts)
    STATS_NAME(ble_phy_stats, rx_aborts)
    STATS_NAME(ble_phy_stats, rx_valid)
    STATS_NAME(ble_phy_stats, rx_crc_err)
    STATS_NAME(ble_phy_stats, rx_late)
    STATS_NAME(ble_phy_stats, radio_state_errs)
    STATS_NAME(ble_phy_stats, rx_hw_err)
    STATS_NAME(ble_phy_stats, tx_hw_err)
STATS_NAME_END(ble_phy_stats)

/*
 * NOTE:
 * Tested the following to see what would happen:
 *  -> NVIC has radio irq enabled (interrupt # 1, mask 0x2).
 *  -> Set up nrf to receive. Clear ADDRESS event register.
 *  -> Enable ADDRESS interrupt on nrf5 by writing to INTENSET.
 *  -> Enable RX.
 *  -> Disable interrupts globally using OS_ENTER_CRITICAL().
 *  -> Wait until a packet is received and the ADDRESS event occurs.
 *  -> Call ble_phy_disable().
 *
 *  At this point I wanted to see the state of the cortex NVIC. The IRQ
 *  pending bit was TRUE for the radio interrupt (as expected) as we never
 *  serviced the radio interrupt (interrupts were disabled).
 *
 *  What was unexpected was this: without clearing the pending IRQ in the NVIC,
 *  when radio interrupts were re-enabled (address event bit in INTENSET set to
 *  1) and the radio ADDRESS event register read 1 (it was never cleared after
 *  the first address event), the radio did not enter the ISR! I would have
 *  expected that if the following were true, an interrupt would occur:
 *      -> NVIC ISER bit set to TRUE
 *      -> NVIC ISPR bit reads TRUE, meaning interrupt is pending.
 *      -> Radio peripheral interrupts are enabled for some event (or events).
 *      -> Corresponding event register(s) in radio peripheral read 1.
 *
 *  Not sure what the end result of all this is. We will clear the pending
 *  bit in the NVIC just to be sure when we disable the PHY.
 */

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)

/*
 * Per nordic, the number of bytes needed for scratch is 16 + MAX_PKT_SIZE.
 * However, when I used a smaller size it still overwrote the scratchpad. Until
 * I figure this out I am just going to allocate 67 words so we have enough
 * space for 267 bytes of scratch. I used 268 bytes since not sure if this
 * needs to be aligned and burning a byte is no big deal.
 */
//#define NRF_ENC_SCRATCH_WORDS (((MYNEWT_VAL(BLE_LL_MAX_PKT_SIZE) + 16) + 3) / 4)
#define NRF_ENC_SCRATCH_WORDS   (67)

uint32_t g_nrf_encrypt_scratchpad[NRF_ENC_SCRATCH_WORDS];

struct nrf_ccm_data
{
    uint8_t key[16];
    uint64_t pkt_counter;
    uint8_t dir_bit;
    uint8_t iv[8];
} __attribute__((packed));

struct nrf_ccm_data g_nrf_ccm_data;
#endif

static void
ble_phy_apply_errata_102_106_107(void)
{
    /* [102] RADIO: PAYLOAD/END events delayed or not triggered after ADDRESS
     * [106] RADIO: Higher CRC error rates for some access addresses
     * [107] RADIO: Immediate address match for access addresses containing MSBs 0x00
     */
//     *(volatile uint32_t *)0x40001774 = ((*(volatile uint32_t *)0x40001774) &
//                          0xfffffffe) | 0x01000000;
}

#if (BLE_LL_BT5_PHY_SUPPORTED == 1)

/* Packet start offset (in usecs). This is the preamble plus access address.
 * For LE Coded PHY this also includes CI and TERM1. */
uint32_t
ble_phy_mode_pdu_start_off(int phy_mode)
{
    return g_ble_phy_mode_pkt_start_off[phy_mode];
}

#if NRF52840_XXAA
static inline bool
ble_phy_mode_is_coded(uint8_t phy_mode)
{
    return (phy_mode == BLE_PHY_MODE_CODED_125KBPS) ||
           (phy_mode == BLE_PHY_MODE_CODED_500KBPS);
}

static void
ble_phy_apply_nrf52840_errata(uint8_t new_phy_mode)
{
    bool new_coded = ble_phy_mode_is_coded(new_phy_mode);
    bool cur_coded = ble_phy_mode_is_coded(g_ble_phy_data.phy_cur_phy_mode);

    /*
     * Workarounds should be applied only when switching to/from LE Coded PHY
     * so no need to apply them every time.
     *
     * nRF52840 Engineering A Errata v1.2
     * [164] RADIO: Low sensitivity in long range mode
     *
     * nRF52840 Rev 1 Errata
     * [191] RADIO: High packet error rate in BLE Long Range mode
     */
    if (new_coded == cur_coded) {
        return;
    }

    if (new_coded) {
#if MYNEWT_VAL(BLE_PHY_NRF52840_ERRATA_164)
        /* [164] */
        *(volatile uint32_t *)0x4000173C |= 0x80000000;
        *(volatile uint32_t *)0x4000173C =
                        ((*(volatile uint32_t *)0x4000173C & 0xFFFFFF00) | 0x5C);
#endif
#if MYNEWT_VAL(BLE_PHY_NRF52840_ERRATA_191)
        /* [191] */
        *(volatile uint32_t *) 0x40001740 =
                        ((*((volatile uint32_t *) 0x40001740)) & 0x7FFF00FF) |
                        0x80000000 | (((uint32_t)(196)) << 8);
#endif
    } else {
#if MYNEWT_VAL(BLE_PHY_NRF52840_ERRATA_164)
        /* [164] */
        *(volatile uint32_t *)0x4000173C &= ~0x80000000;
#endif
#if MYNEWT_VAL(BLE_PHY_NRF52840_ERRATA_191)
        /* [191] */
        *(volatile uint32_t *) 0x40001740 =
                        ((*((volatile uint32_t *) 0x40001740)) & 0x7FFFFFFF);
#endif
    }
}
#endif

static void
ble_phy_mode_apply(uint8_t phy_mode)
{
    if (phy_mode == g_ble_phy_data.phy_cur_phy_mode) {
        return;
    }

#if NRF52840_XXAA
    ble_phy_apply_nrf52840_errata(phy_mode);
#endif

    switch (phy_mode) {
    case BLE_PHY_MODE_1M:
        NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_1Mbit;
        NRF_RADIO->PCNF0 = NRF_PCNF0_1M;
        break;
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_2M_PHY)
    case BLE_PHY_MODE_2M:
        NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_2Mbit;
        NRF_RADIO->PCNF0 = NRF_PCNF0_2M;
        break;
#endif
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
    case BLE_PHY_MODE_CODED_125KBPS:
        NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_LR125Kbit;
        NRF_RADIO->PCNF0 = NRF_PCNF0_CODED;
        break;
    case BLE_PHY_MODE_CODED_500KBPS:
        NRF_RADIO->MODE = RADIO_MODE_MODE_Ble_LR500Kbit;
        NRF_RADIO->PCNF0 = NRF_PCNF0_CODED;
        break;
#endif
    default:
        assert(0);
    }

    g_ble_phy_data.phy_cur_phy_mode = phy_mode;
}

void
ble_phy_mode_set(uint8_t tx_phy_mode, uint8_t rx_phy_mode)
{
    g_ble_phy_data.phy_tx_phy_mode = tx_phy_mode;
    g_ble_phy_data.phy_rx_phy_mode = rx_phy_mode;
}
#endif

int
ble_phy_get_cur_phy(void)
{
#if (BLE_LL_BT5_PHY_SUPPORTED == 1)
    switch (g_ble_phy_data.phy_cur_phy_mode) {
        case BLE_PHY_MODE_1M:
            return BLE_PHY_1M;
        case BLE_PHY_MODE_2M:
            return BLE_PHY_2M;
        case BLE_PHY_MODE_CODED_125KBPS:
        case BLE_PHY_MODE_CODED_500KBPS:
            return BLE_PHY_CODED;
        default:
            assert(0);
            return -1;
    }
#else
    return BLE_PHY_1M;
#endif
}

/**
 * Copies the data from the phy receive buffer into a mbuf chain.
 *
 * @param dptr Pointer to receive buffer
 * @param rxpdu Pointer to already allocated mbuf chain
 *
 * NOTE: the packet header already has the total mbuf length in it. The
 * lengths of the individual mbufs are not set prior to calling.
 *
 */
void
ble_phy_rxpdu_copy(uint8_t *dptr, struct os_mbuf *rxpdu)
{
    uint32_t rem_len;
    uint32_t copy_len;
    uint32_t block_len;
    uint32_t block_rem_len;
    void *dst;
    void *src;
    struct os_mbuf * om;

    /* Better be aligned */
    assert(((uint32_t)dptr & 3) == 0);

    block_len = rxpdu->om_omp->omp_databuf_len;
    rem_len = OS_MBUF_PKTHDR(rxpdu)->omp_len;
    src = dptr;

    /*
     * Setup for copying from first mbuf which is shorter due to packet header
     * and extra leading space
     */
    copy_len = block_len - rxpdu->om_pkthdr_len - 4;
    om = rxpdu;
    dst = om->om_data;

    while (true) {
        /*
         * Always copy blocks of length aligned to word size, only last mbuf
         * will have remaining non-word size bytes appended.
         */
        block_rem_len = copy_len;
        copy_len = min(copy_len, rem_len);
        copy_len &= ~3;

        dst = om->om_data;
        om->om_len = copy_len;
        rem_len -= copy_len;
        block_rem_len -= copy_len;

//         __asm__ volatile (".syntax unified              \n"
//                           "   mov  r4, %[len]           \n"
//                           "   b    2f                   \n"
//                           "1: ldr  r3, [%[src], %[len]] \n"
//                           "   str  r3, [%[dst], %[len]] \n"
//                           "2: subs %[len], #4           \n"
//                           "   bpl  1b                   \n"
//                           "   adds %[src], %[src], r4   \n"
//                           "   adds %[dst], %[dst], r4   \n"
//                           : [dst] "+r" (dst), [src] "+r" (src),
//                             [len] "+r" (copy_len)
//                           :
//                           : "r3", "r4", "memory"
//                          );

        if ((rem_len < 4) && (block_rem_len >= rem_len)) {
            break;
        }

        /* Move to next mbuf */
        om = SLIST_NEXT(om, om_next);
        copy_len = block_len;
    }

    /* Copy remaining bytes, if any, to last mbuf */
    om->om_len += rem_len;
//     __asm__ volatile (".syntax unified              \n"
//                       "   b    2f                   \n"
//                       "1: ldrb r3, [%[src], %[len]] \n"
//                       "   strb r3, [%[dst], %[len]] \n"
//                       "2: subs %[len], #1           \n"
//                       "   bpl  1b                   \n"
//                       : [len] "+r" (rem_len)
//                       : [dst] "r" (dst), [src] "r" (src)
//                       : "r3", "memory"
//                      );

    /* Copy header */
    memcpy(BLE_MBUF_HDR_PTR(rxpdu), &g_ble_phy_data.rxhdr,
           sizeof(struct ble_mbuf_hdr));
}

/**
 * Function is used to set PPI so that we can time out waiting for a reception
 * to occur. This happens for two reasons: we have sent a packet and we are
 * waiting for a respons (txrx should be set to ENABLE_TXRX) or we are
 * starting a connection event and we are a slave and we are waiting for the
 * master to send us a packet (txrx should be set to ENABLE_RX).
 *
 * NOTE: when waiting for a txrx turn-around, wfr_usecs is not used as there
 * is no additional time to wait; we know when we should receive the address of
 * the received frame.
 *
 * @param txrx Flag denoting if this wfr is a txrx turn-around or not.
 * @param tx_phy_mode phy mode for last TX (only valid for TX->RX)
 * @param wfr_usecs Amount of usecs to wait.
 */
void
ble_phy_wfr_enable(int txrx, uint8_t tx_phy_mode, uint32_t wfr_usecs)
{

}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
static uint32_t
ble_phy_get_ccm_datarate(void)
{
#if BLE_LL_BT5_PHY_SUPPORTED
    switch (g_ble_phy_data.phy_cur_phy_mode) {
    case BLE_PHY_MODE_1M:
        return CCM_MODE_DATARATE_1Mbit << CCM_MODE_DATARATE_Pos;
    case BLE_PHY_MODE_2M:
        return CCM_MODE_DATARATE_2Mbit << CCM_MODE_DATARATE_Pos;
#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_CODED_PHY)
    case BLE_PHY_MODE_CODED_125KBPS:
        return CCM_MODE_DATARATE_125Kbps << CCM_MODE_DATARATE_Pos;
    case BLE_PHY_MODE_CODED_500KBPS:
        return CCM_MODE_DATARATE_500Kbps << CCM_MODE_DATARATE_Pos;
#endif
    }

    assert(0);
    return 0;
#else
    return CCM_MODE_DATARATE_1Mbit << CCM_MODE_DATARATE_Pos;
#endif
}
#endif

static inline uint8_t
ble_phy_get_cur_rx_phy_mode(void)
{
    uint8_t phy;

    phy = g_ble_phy_data.phy_cur_phy_mode;

    return phy;
}

/**
 * ble phy init
 *
 * Initialize the PHY.
 *
 * @return int 0: success; PHY error code otherwise
 */
int
ble_phy_init(void)
{
    return 0;
}

/**
 * Puts the phy into receive mode.
 *
 * @return int 0: success; BLE Phy error code otherwise
 */
int
ble_phy_rx(void)
{
    return 0;
}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LE_ENCRYPTION)
/**
 * Called to enable encryption at the PHY. Note that this state will persist
 * in the PHY; in other words, if you call this function you have to call
 * disable so that future PHY transmits/receives will not be encrypted.
 *
 * @param pkt_counter
 * @param iv
 * @param key
 * @param is_master
 */
void
ble_phy_encrypt_enable(uint64_t pkt_counter, uint8_t *iv, uint8_t *key,
                       uint8_t is_master)
{
    memcpy(g_nrf_ccm_data.key, key, 16);
    g_nrf_ccm_data.pkt_counter = pkt_counter;
    memcpy(g_nrf_ccm_data.iv, iv, 8);
    g_nrf_ccm_data.dir_bit = is_master;
    g_ble_phy_data.phy_encrypted = 1;
    /* Enable the module (AAR cannot be on while CCM on) */
//     NRF_AAR->ENABLE = AAR_ENABLE_ENABLE_Disabled;
//     NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Enabled;
}

void
ble_phy_encrypt_set_pkt_cntr(uint64_t pkt_counter, int dir)
{
    g_nrf_ccm_data.pkt_counter = pkt_counter;
    g_nrf_ccm_data.dir_bit = dir;
}

void
ble_phy_encrypt_disable(void)
{
//     NRF_PPI->CHENCLR = PPI_CHEN_CH25_Msk;
//     NRF_CCM->TASKS_STOP = 1;
//     NRF_CCM->EVENTS_ERROR = 0;
//     NRF_CCM->ENABLE = CCM_ENABLE_ENABLE_Disabled;

    g_ble_phy_data.phy_encrypted = 0;
}
#endif

void
ble_phy_set_txend_cb(ble_phy_tx_end_func txend_cb, void *arg)
{
    /* Set transmit end callback and arg */
    g_ble_phy_data.txend_cb = txend_cb;
    g_ble_phy_data.txend_arg = arg;
}

/**
 * Called to set the start time of a transmission.
 *
 * This function is called to set the start time when we are not going from
 * rx to tx automatically.
 *
 * NOTE: care must be taken when calling this function. The channel should
 * already be set.
 *
 * @param cputime   This is the tick at which the 1st bit of the preamble
 *                  should be transmitted
 * @param rem_usecs This is used only when the underlying timing uses a 32.768
 *                  kHz crystal. It is the # of usecs from the cputime tick
 *                  at which the first bit of the preamble should be
 *                  transmitted.
 * @return int
 */
int
ble_phy_tx_set_start_time(uint32_t cputime, uint8_t rem_usecs)
{
    return 0;
}

/**
 * Called to set the start time of a reception
 *
 * This function acts a bit differently than transmit. If we are late getting
 * here we will still attempt to receive.
 *
 * NOTE: care must be taken when calling this function. The channel should
 * already be set.
 *
 * @param cputime
 *
 * @return int
 */
int
ble_phy_rx_set_start_time(uint32_t cputime, uint8_t rem_usecs)
{
    return 0;
}

int
ble_phy_tx(ble_phy_tx_pducb_t pducb, void *pducb_arg, uint8_t end_trans)
{
    return 0;
}

/**
 * ble phy txpwr set
 *
 * Set the transmit output power (in dBm).
 *
 * NOTE: If the output power specified is within the BLE limits but outside
 * the chip limits, we "rail" the power level so we dont exceed the min/max
 * chip values.
 *
 * @param dbm Power output in dBm.
 *
 * @return int 0: success; anything else is an error
 */
int
ble_phy_txpwr_set(int dbm)
{
    /* "Rail" power level if outside supported range */
    dbm = ble_phy_txpower_round(dbm);

//     NRF_RADIO->TXPOWER = dbm;
    g_ble_phy_data.phy_txpwr_dbm = dbm;

    return 0;
}

/**
 * ble phy txpwr round
 *
 * Get the rounded transmit output power (in dBm).
 *
 * @param dbm Power output in dBm.
 *
 * @return int Rounded power in dBm
 */
int ble_phy_txpower_round(int dbm)
{
    return 0;
}

/**
 * ble phy set access addr
 *
 * Set access address.
 *
 * @param access_addr Access address
 *
 * @return int 0: success; PHY error code otherwise
 */
static int
ble_phy_set_access_addr(uint32_t access_addr)
{
//     NRF_RADIO->BASE0 = (access_addr << 8);
//     NRF_RADIO->PREFIX0 = (NRF_RADIO->PREFIX0 & 0xFFFFFF00) | (access_addr >> 24);

    g_ble_phy_data.phy_access_address = access_addr;

    ble_phy_apply_errata_102_106_107();

    return 0;
}

/**
 * ble phy txpwr get
 *
 * Get the transmit power.
 *
 * @return int  The current PHY transmit power, in dBm
 */
int
ble_phy_txpwr_get(void)
{
    return g_ble_phy_data.phy_txpwr_dbm;
}

void
ble_phy_set_rx_pwr_compensation(int8_t compensation)
{
    g_ble_phy_data.rx_pwr_compensation = compensation;
}

/**
 * ble phy setchan
 *
 * Sets the logical frequency of the transceiver. The input parameter is the
 * BLE channel index (0 to 39, inclusive). The NRF frequency register works like
 * this: logical frequency = 2400 + FREQ (MHz).
 *
 * Thus, to get a logical frequency of 2402 MHz, you would program the
 * FREQUENCY register to 2.
 *
 * @param chan This is the Data Channel Index or Advertising Channel index
 *
 * @return int 0: success; PHY error code otherwise
 */
int
ble_phy_setchan(uint8_t chan, uint32_t access_addr, uint32_t crcinit)
{
    assert(chan < BLE_PHY_NUM_CHANS);

    /* Check for valid channel range */
    if (chan >= BLE_PHY_NUM_CHANS) {
        return BLE_PHY_ERR_INV_PARAM;
    }

    /* Set current access address */
    ble_phy_set_access_addr(access_addr);

//     /* Configure crcinit */
//     NRF_RADIO->CRCINIT = crcinit;
//
//     /* Set the frequency and the data whitening initial value */
//     g_ble_phy_data.phy_chan = chan;
//     NRF_RADIO->FREQUENCY = g_ble_phy_chan_freq[chan];
//     NRF_RADIO->DATAWHITEIV = chan;

    return 0;
}

/**
 * Stop the timer used to count microseconds when using RTC for cputime
 */
static void
ble_phy_stop_usec_timer(void)
{
//     NRF_TIMER0->TASKS_STOP = 1;
//     NRF_TIMER0->TASKS_SHUTDOWN = 1;
//     NRF_RTC0->EVTENCLR = RTC_EVTENSET_COMPARE0_Msk;
}

/**
 * ble phy disable irq and ppi
 *
 * This routine is to be called when reception was stopped due to either a
 * wait for response timeout or a packet being received and the phy is to be
 * restarted in receive mode. Generally, the disable routine is called to stop
 * the phy.
 */
static void
ble_phy_disable_irq_and_ppi(void)
{
//     NRF_RADIO->INTENCLR = NRF_RADIO_IRQ_MASK_ALL;
//     NRF_RADIO->SHORTS = 0;
//     NRF_RADIO->TASKS_DISABLE = 1;
//     NRF_PPI->CHENCLR = PPI_CHEN_CH4_Msk | PPI_CHEN_CH5_Msk | PPI_CHEN_CH20_Msk |
//           PPI_CHEN_CH21_Msk | PPI_CHEN_CH23_Msk |
//           PPI_CHEN_CH25_Msk | PPI_CHEN_CH31_Msk;
//     NVIC_ClearPendingIRQ(RADIO_IRQn);
//     g_ble_phy_data.phy_state = BLE_PHY_STATE_IDLE;
}

void
ble_phy_restart_rx(void)
{
    ble_phy_stop_usec_timer();
    ble_phy_disable_irq_and_ppi();

//     ble_phy_set_start_now();
    /* Enable PPI to automatically start RXEN */
//     NRF_PPI->CHENSET = PPI_CHEN_CH21_Msk;

    ble_phy_rx();
}

/**
 * ble phy disable
 *
 * Disables the PHY. This should be called when an event is over. It stops
 * the usec timer (if used), disables interrupts, disables the RADIO, disables
 * PPI and sets state to idle.
 */
void
ble_phy_disable(void)
{
    ble_phy_trace_void(BLE_PHY_TRACE_ID_DISABLE);

    ble_phy_stop_usec_timer();
    ble_phy_disable_irq_and_ppi();
}

/* Gets the current access address */
uint32_t ble_phy_access_addr_get(void)
{
    return g_ble_phy_data.phy_access_address;
}

/**
 * Return the phy state
 *
 * @return int The current PHY state.
 */
int
ble_phy_state_get(void)
{
    return g_ble_phy_data.phy_state;
}

/**
 * Called to see if a reception has started
 *
 * @return int
 */
int
ble_phy_rx_started(void)
{
    return g_ble_phy_data.phy_rx_started;
}

/**
 * Return the transceiver state
 *
 * @return int transceiver state.
 */
uint8_t
ble_phy_xcvr_state_get(void)
{
    uint32_t state;
//     state = NRF_RADIO->STATE;
    return (uint8_t)state;
}

/**
 * Called to return the maximum data pdu payload length supported by the
 * phy. For this chip, if encryption is enabled, the maximum payload is 27
 * bytes.
 *
 * @return uint8_t Maximum data channel PDU payload size supported
 */
uint8_t
ble_phy_max_data_pdu_pyld(void)
{
    return BLE_LL_DATA_PDU_MAX_PYLD;
}

#if MYNEWT_VAL(BLE_LL_CFG_FEAT_LL_PRIVACY)
void
ble_phy_resolv_list_enable(void)
{
//     NRF_AAR->NIRK = (uint32_t)g_nrf_num_irks;
    g_ble_phy_data.phy_privacy = 1;
}

void
ble_phy_resolv_list_disable(void)
{
    g_ble_phy_data.phy_privacy = 0;
}
#endif

#if MYNEWT_VAL(BLE_LL_DTM)
void ble_phy_enable_dtm(void)
{
    /* When DTM is enabled we need to disable whitening as per
     * Bluetooth v5.0 Vol 6. Part F. 4.1.1
     */
//     NRF_RADIO->PCNF1 &= ~RADIO_PCNF1_WHITEEN_Msk;
}

void ble_phy_disable_dtm(void)
{
    /* Enable whitening */
//     NRF_RADIO->PCNF1 |= RADIO_PCNF1_WHITEEN_Msk;
}
#endif

void
ble_phy_rfclk_enable(void)
{
#if MYNEWT
    nrf52_clock_hfxo_request();
#else
//     NRF_CLOCK->TASKS_HFCLKSTART = 1;
#endif
}

void
ble_phy_rfclk_disable(void)
{
#if MYNEWT
    nrf52_clock_hfxo_release();
#else
//     NRF_CLOCK->TASKS_HFCLKSTOP = 1;
#endif
}
