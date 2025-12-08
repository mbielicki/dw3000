/*! ----------------------------------------------------------------------------
 *  @file    ds_twr_initiator.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
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

#if defined(TEST_DS_TWR_INITIATOR)

extern void test_run_info(unsigned char *data);

/* Example application name and version to display on LCD screen. */
#define APP_NAME "DS TWR INIT v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
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

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000 //1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define MY_COMMA_ADDRESS 0x00, MY_ADDRESS
#define MY_FULL_ADDRESS (0x0000 | MY_ADDRESS)
#define N_ANCHORS       6

static uint16_t anchor_addrs[N_ANCHORS] = { 0x0008, 0x0069, 0x0098, 0x0094, 0x0035, 0x0099 };
static int c_anchor;

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, MY_COMMA_ADDRESS, 0x21 };
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Software lock to prevent re-entrancy in the rx_ok_cb callback. */
static volatile int g_irq_lock = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 0//(0 + CPU_PROCESSING_TIME)//(300 + CPU_PROCESSING_TIME)

#define NEW_COORDS_DLY 5 * 1000 * 1000
#define NEW_ANCHOR_DLY 5 * 1000 * 1000

static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

static float dist_to_anchor[N_ANCHORS];

void info_msg_set_to_addr(uint8_t *ta_field, uint16_t ta)
{
    uint8_t i;
    for (i = 0; i < 2; i++)
    {
        ta_field[i] = (uint8_t)ta;
        ta >>= 8;
    }
}
void info_msg_set_dist_cm(uint8_t *dc_field, uint16_t dc)
{
    uint8_t i;
    for (i = 0; i < 2; i++)
    {
        dc_field[i] = (uint8_t)dc;
        dc >>= 8;
    }
}

static uint8_t tx_info_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, MY_COMMA_ADDRESS, 0x25, 0, 0, 0, 0 };
#define INFO_MSG_TO_ADDR_IDX  10
#define INFO_MSG_DIST_CM_IDX  12
#define INFO_TX_TO_RX_DLY_UUS 200
void send_info(uint16_t to_addr, uint16_t dist_cm) {
    
    dwt_forcetrxoff();
    /* Write frame data to DW IC and prepare transmission. See NOTE 9 below. */
    tx_info_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    info_msg_set_to_addr(&tx_info_msg[INFO_MSG_TO_ADDR_IDX], to_addr);
    info_msg_set_dist_cm(&tx_info_msg[INFO_MSG_DIST_CM_IDX], dist_cm);


    dwt_writetxdata(sizeof(tx_info_msg), tx_info_msg, 0);  /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_info_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

    dwt_setrxaftertxdelay(INFO_TX_TO_RX_DLY_UUS);
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    frame_seq_nb++;
    
    #ifdef DEBUG_MODE
      printf("info sent");
    #endif
}

uint32_t poll_rx_ts_32;
char json[80];
void handle_final() {
    uint64_t resp_tx_ts, final_rx_ts;
    uint32_t resp_tx_ts_32, final_rx_ts_32;

    uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
    double tag_comm_time, anchor_comm_time;
    int64_t tof_dtu;
    double tof;
    double distance;
    uint16_t src_addr;
    double Ra, Rb, Da, Db;
    
    src_addr = get_src_addr(rx_buffer);

    /* Get timestamps embedded in the final message. */
    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

    /* Retrieve response transmission and final reception timestamps. */
    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();

    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
    resp_tx_ts_32 = (uint32_t)resp_tx_ts;
    final_rx_ts_32 = (uint32_t)final_rx_ts;

    #ifdef DEBUG_MODE
    printf("poll_rx_ts: %x ", poll_rx_ts_32);
    printf("resp_rx_ts: %x ", resp_tx_ts_32);
    printf("final_rx_ts: %x\n", final_rx_ts_32);

    #endif

    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;

    /* Display computed distance */
    uint16_t distance_cm = distance * 100;
    sprintf(json, "{\"from\": \"%x\", \"to\": \"%x\", \"dist\": \"%i\"}", MY_FULL_ADDRESS, src_addr, distance_cm);
    test_run_info((unsigned char *)json);


    send_info(src_addr, distance_cm);

}

void send_poll() {

    dwt_forcetrxoff();
    /* Write frame data to DW IC and prepare transmission. See NOTE 9 below. */
    frame_seq_nb = 0;
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);  /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    frame_seq_nb++;
    
    #ifdef DEBUG_MODE
      printf("polling %x", tx_poll_msg[6]);
    #endif
}

#define POLL_RX_TO_RESP_TX_DLY_UUS 700
#define RX_AFTER_RESP_DLY 700
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, MY_COMMA_ADDRESS, 0x10, 0x02, 0, 0, 0, 0 };

static uint16_t tag_address;

void handle_poll() {
      uint32_t resp_tx_time;
      int ret;

      /* Retrieve poll transmission timestamp. */
      poll_rx_ts = get_rx_timestamp_u64();
      poll_rx_ts_32 = (uint32_t)poll_rx_ts; // save global poll_rx
      #ifdef DEBUG_MODE
      printf("saving poll_rx_ts: %x\n", poll_rx_ts_32);
      #endif
      /* Compute resp message transmission time. See NOTE 11 below. */
      resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      //dwt_setrxaftertxdelay(RX_AFTER_RESP_DLY);

      /* Resp TX timestamp is the transmission time we programmed plus the TX antenna delay. */
      resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

      /* Write and send resp message. See NOTE 9 below. */
      tag_address = get_src_addr(rx_buffer);
      set_dst_addr(tx_resp_msg, tag_address);
      tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(tx_resp_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */


      ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 13 below. */
      if (ret == DWT_SUCCESS)
      {
          frame_seq_nb++;
          #ifdef DEBUG_MODE
           printf("sent resp to %x", tag_address);
          #endif
      }
      else {
        test_run_info((unsigned char *)"resp sending ERROR");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
}

#define RESP_RX_TO_FINAL_TX_DLY_UUS 700
#define RX_AFTER_FINAL_DLY 700

static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;
static uint64_t poll_tx_ts;

static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, MY_COMMA_ADDRESS, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void handle_resp() {
      uint32_t final_tx_time;
      int ret;

      /* Retrieve poll transmission and response reception timestamp. */
      resp_rx_ts = get_rx_timestamp_u64();
      poll_tx_ts = get_tx_timestamp_u64();

      /* Compute final message transmission time. See NOTE 11 below. */
      final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(final_tx_time);

      dwt_setrxaftertxdelay(RX_AFTER_FINAL_DLY);

      /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
      final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

      /* Write all timestamps in the final message. See NOTE 12 below. */
      final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
      final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
      final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

      /* Write and send final message. See NOTE 9 below. */
      tag_address = get_src_addr(rx_buffer);
      set_dst_addr(tx_final_msg, tag_address);
      tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(tx_final_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */


      ret = dwt_starttx(DWT_START_TX_DELAYED);
      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 13 below. */
      if (ret == DWT_SUCCESS)
      {
          frame_seq_nb++;
          #ifdef DEBUG_MODE
           printf("sent final to %x", tag_address);
          #endif
      }
      else {
        test_run_info((unsigned char *)"final sending ERROR");
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }

}

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 8 below. */
extern dwt_txconfig_t txconfig_options;

void wait(uint32_t time) {
  for(uint32_t tick = 0; tick < time; tick++) { }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_initiator()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ds_twr_initiator(void)
{
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);
    printf("%x\n", MY_ADDRESS);

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

    /* Configure DW IC. See NOTE 2 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);



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


    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    // dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    #ifndef DOES_POLL

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    while (1) {}

    #else

    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
        for(c_anchor = 0; c_anchor < N_ANCHORS; c_anchor++) {
          if (anchor_addrs[c_anchor] == MY_FULL_ADDRESS) continue;

          set_dst_addr(tx_poll_msg, anchor_addrs[c_anchor]);
          send_poll();

          wait(NEW_ANCHOR_DLY);
        }

        wait(NEW_COORDS_DLY);
    }

    #endif
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
    int i;

    /* If the lock is already held, it means we are in the middle of processing an interrupt.
     * Return immediately to prevent re-entrancy and state corruption. */
    if (g_irq_lock)
    {
        return;
    }

    g_irq_lock = 1; // Acquire the lock

    /* A frame has been received, copy it to our local buffer. */
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    if (frame_is_poll_for_me(rx_buffer))
    {
#ifdef DEBUG_MODE
        printf("got poll from %x\n", rx_buffer[8]);
#endif
        handle_poll();
    }
    else if (frame_is_resp_for_me(rx_buffer))
    {
#ifdef DEBUG_MODE
        printf("got resp from %x\n", rx_buffer[8]);
#endif
        handle_resp();
    }
    else if (frame_is_final_for_me(rx_buffer))
    {
#ifdef DEBUG_MODE
        printf("got final from %x\n", rx_buffer[8]);
#endif
        handle_final();
    }
    else
    {
#ifdef DEBUG_MODE
        test_run_info((unsigned char *)"got -");
#endif
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    g_irq_lock = 0; // Release the lock
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
    #ifdef DEBUG_MODE
        printf(".\n");
    #endif
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW IC using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */
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
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
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
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.81 Mbps data rate used (around 300 us).
 * 6. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 36 MHz can be used
 *    thereafter.
 * 7. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 8. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 10. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 11. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW IC
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 * 12. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 13. When running this example on the DWK3000 platform with the RESP_RX_TO_FINAL_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange to try another one after 1 second. If this error handling code was not here, a late dwt_starttx() would result in the code
 *     flow getting stuck waiting for a TX frame sent event that will never come. The companion "responder" example (ex_05b) should timeout from
 *     awaiting the "final" and proceed to have its receiver on ready to poll of the following exchange.
 * 14. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 ****************************************************************************************************************************************************/
