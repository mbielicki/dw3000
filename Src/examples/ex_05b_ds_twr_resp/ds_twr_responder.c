/*! ----------------------------------------------------------------------------
 *  @file    ds_twr_responder.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a DS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "DS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message recording its TX time-stamp, after which it waits for a "final" message from the initiator to complete
 *           the exchange. The final message contains the remote initiator's time-stamps of poll TX, response RX and final TX. With this data and the
 *           local time-stamps, (of poll RX, response TX and final RX), this example application works out a value for the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#if defined(TEST_DS_TWR_RESPONDER)

extern void test_run_info(unsigned char *data);

/* TDMA master configuration */
#define MASTER_ADDR          0x0072
#define NUM_SLAVES           3
static const uint16_t slave_addresses[NUM_SLAVES] = { 0x0099, 0x0098, 0x0008 };
#define SUPERFRAME_PERIOD_MS 400
#define SLOT_DURATION_MS     1000
#define SYNC_SLOT_MS         5

#define FUNC_CODE_SYNC       0xAA
#define FUNC_CODE_TOKEN      0xBB

/* Frames used in the TDMA scheduling. */
// SYNC message: broadcast to all slaves
static uint8_t sync_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0, 0, FUNC_CODE_SYNC };
// TOKEN message: addressed to a specific slave
static uint8_t token_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_CODE_TOKEN };

#define SRC_ADDR_IDX  7
#define DST_ADDR_IDX  5
#define SEQ_NUM_IDX   2

static volatile bool tx_done = false;

/* Example application name */
#define APP_NAME "TDMA MASTER"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = { 5,                /* Channel number. */
                               DWT_PLEN_128,     /* Preamble length. Used in TX only. */
                               DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
                               9,                /* TX preamble code. Used in TX only. */
                               9,                /* RX preamble code. Used in RX only. */
                               1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
                               DWT_BR_6M8,       /* Data rate. */
                               DWT_PHRMODE_STD,  /* PHY header mode. */
                               DWT_PHRRATE_STD,  /* PHY header rate. */
                               (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
                               DWT_STS_MODE_OFF, /* STS disabled */
                               DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
                               DWT_PDOA_M0       /* PDOA mode off */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Index to access some of the fields in the frames involved in the process. */
#define INFO_MSG_TO_ADDR_IDX  10
#define INFO_MSG_DIST_CM_IDX  12


/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;


/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

uint16_t info_msg_get_to_addr(uint8_t *ta_field)
{
    uint8_t i;
    uint16_t ta = 0;
    for (i = 0; i < 2; i++)
    {
        ta += ((uint16_t)ta_field[i] << (i * 8));
    }

    return ta;
}
uint16_t info_msg_get_dist_cm(uint8_t *dc_field)
{
    uint8_t i;
    uint16_t dc = 0;
    for (i = 0; i < 2; i++)
    {
        dc += ((uint16_t)dc_field[i] << (i * 8));
    }

    return dc;
}

void handle_info()
{
    uint16_t from_addr;
    uint16_t to_addr;
    uint16_t dist_cm;

    from_addr = get_src_addr(rx_buffer);
    to_addr = info_msg_get_to_addr(&rx_buffer[INFO_MSG_TO_ADDR_IDX]);
    dist_cm = info_msg_get_dist_cm(&rx_buffer[INFO_MSG_DIST_CM_IDX]);

    printf("{\"from\": \"%x\", \"to\": \"%x\", \"dist\": \"%i\"}\n", from_addr, to_addr, dist_cm);
}

/*!
 * @brief Sends a given message over UWB. This is a blocking function.
 *
 * @param msg pointer to the message to be sent
 * @param msg_len length of the message
 */
static void send_msg(uint8_t *msg, size_t msg_len)
{
    tx_done = false;
    // Set sequence number
    msg[SEQ_NUM_IDX] = frame_seq_nb++;

    dwt_writetxdata(msg_len, msg, 0);
    dwt_writetxfctrl(msg_len + 2, 0, 0); // No ranging, no response expected

    // Start transmission
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    // Wait for TX confirmation
    while (!tx_done) { };
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_responder()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ds_twr_responder(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 15 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Set PAN ID and master address */
    dwt_setpanid(0xDECA);
    dwt_setaddress16(MASTER_ADDR);
    /* Disable frame filtering. */
    dwt_configureframefilter(DWT_FF_DISABLE, 0);

    /* Register the call-backs (SPI CRC error callback is not used). */
    dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb, NULL, NULL, NULL);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK
                         | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
                     0, DWT_ENABLE_INT);

    /*Clearing the SPI ready interrupt*/
    dwt_writesysstatuslo(DWT_INT_RCINIT_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK);

    /* Install DW IC IRQ handler. */
    port_set_dwic_isr(dwt_isr);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);

    /* Set master address in message templates */
    sync_msg[SRC_ADDR_IDX] = MASTER_ADDR & 0xFF;
    sync_msg[SRC_ADDR_IDX + 1] = (MASTER_ADDR >> 8) & 0xFF;
    token_msg[SRC_ADDR_IDX] = MASTER_ADDR & 0xFF;
    token_msg[SRC_ADDR_IDX + 1] = (MASTER_ADDR >> 8) & 0xFF;

    /* Main TDMA master loop */
    while (1)
    {
        /* Turn off receiver before transmitting */
        dwt_forcetrxoff();

        /* 1. Broadcast a sync message to all slaves */
#ifdef DEBUG_MODE
        printf("SYNC: Broadcasting sync message\n");
#endif
        send_msg(sync_msg, sizeof(sync_msg));
#ifdef DEBUG_MODE
        printf("SYNC: Sync message transmitted.\n");
#endif
        /* Brief sleep to allow slaves to process sync */
        Sleep(SYNC_SLOT_MS);

        /* 2. Grant token to each slave and listen for info messages */
        for (int i = 0; i < NUM_SLAVES; i++)
        {
            dwt_forcetrxoff();

            /* Prepare and send token message */
            uint16_t slave_addr = slave_addresses[i];
            token_msg[DST_ADDR_IDX] = slave_addr & 0xFF;
            token_msg[DST_ADDR_IDX + 1] = (slave_addr >> 8) & 0xFF;
#ifdef DEBUG_MODE
            printf("TOKEN: Sending token to slave 0x%04X\n", slave_addr);
#endif
            send_msg(token_msg, sizeof(token_msg));
#ifdef DEBUG_MODE
            printf("TOKEN: Token to slave 0x%04X transmitted.\n", slave_addr);
#endif
            /* Listen for info messages during the slave's slot */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            Sleep(SLOT_DURATION_MS);
        }

        /* Calculate remaining time in superframe and sleep */
        int time_spent_ms = (NUM_SLAVES * SLOT_DURATION_MS) + SYNC_SLOT_MS;
        if (SUPERFRAME_PERIOD_MS > time_spent_ms)
        {
#ifdef DEBUG_MODE
            int sleep_time = SUPERFRAME_PERIOD_MS - time_spent_ms;
            printf("MASTER: All tokens sent. Sleeping for %d ms.\n", sleep_time);
#endif
            Sleep(SUPERFRAME_PERIOD_MS - time_spent_ms);
        }
    }
}

bool frame_is_info(uint8_t* rx_buffer) {
  if (rx_buffer[0] != 0x41) return false;
  if (rx_buffer[1] != 0x88) return false;
  if (rx_buffer[3] != 0xCA) return false;
  if (rx_buffer[4] != 0xDE) return false;

  if (rx_buffer[9] != 0x25) return false;

  return true;
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }
#ifdef DEBUG_MODE
    printf("MASTER RX: Received frame, len %d\n", cb_data->datalength);
#endif

    if (frame_is_info(rx_buffer)) {
#ifdef DEBUG_MODE
    printf("MASTER RX: Frame is info message.\n");
#endif
      handle_info();
    }
    else {
#ifdef DEBUG_MODE
        uint16_t src_addr = rx_buffer[7] | (rx_buffer[8] << 8);
        printf("MASTER RX: Frame is not info. From 0x%04X, FC: 0x%02X\n", src_addr, rx_buffer[9]);
#endif
    }
    
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    /* Set corresponding inter-frame delay. */

    /* TESTING BREAKPOINT LOCATION #2 */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    /* Set corresponding inter-frame delay. */

    /* TESTING BREAKPOINT LOCATION #3 */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    tx_done = true;
}


#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 *    Initiator: |Poll TX| ..... |Resp RX| ........ |Final TX|
 *    Responder: |Poll RX| ..... |Resp TX| ........ |Final RX|
 *                   ^|P RMARKER|                                    - time of Poll TX/RX
 *                                   ^|R RMARKER|                    - time of Resp TX/RX
 *                                                      ^|R RMARKER| - time of Final TX/RX
 *
 *                       <--TDLY->                                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->                            - RESP_RX_TIMEOUT_UUS   (length of poll frame)
 *                    <----RDLY------>                               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder
 *                                                                                                                      can turn around and reply)
 *
 *
 *                                        <--T2DLY->                 - RESP_TX_TO_FINAL_RX_DLY_UUS (R2DLY-FLEN)
 *                                                  <-FLEN--->       - FINAL_RX_TIMEOUT_UUS   (length of response frame)
 *                                    <----RDLY--------->            - RESP_RX_TO_FINAL_TX_DLY_UUS (depends on how quickly initiator
 *                                                                                                                      can turn around and reply)
 *
 * EXAMPLE 1: with SPI rate set to 18 MHz (default on this platform), and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 400uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 400uus (TXtoRX delays are set to 210uus)
 *            reducing the delays further can be achieved by using interrupt to handle the TX/RX events, or other code optimisations/faster SPI
 *
 * EXAMPLE 2: with SPI rate set to 4.5 MHz, and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 550uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 600uus (TXtoRX delays are set to 360 and 410 uus respectively)
 *
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    6.81 Mbps data rate used (around 220 us).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the DWK3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 14. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 36 MHz can be used
 *     thereafter.
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/
