/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <libmaple/rcc.h>

long int alarmDelay = 30;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x8E, 0x44, 0xB9, 0x2C, 0x9A, 0x9E, 0x4E, 0x2E, 0x5E, 0x57, 0x8C, 0xC4, 0xB9, 0x9C, 0x58, 0xFB };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x25, 0x2A, 0x8A, 0x50, 0x36, 0x72, 0xD7, 0x9B, 0x43, 0xD9, 0x47, 0x9B, 0x0F, 0xC8, 0xEE, 0x42 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260116E4 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 3600;

int msg_sent = 0;

// Pin mapping
// https://github.com/rogerclarkmelboSerial3Serial3urne/Arduino_STM32/blob/4021c32fdb809f16a61c073d04e1d8c2c487a767/STM32F1/variants/generic_stm32f103c/board.cpp
const lmic_pinmap lmic_pins = {
    .nss = 0, //PA0
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 1, //PA1
    .dio = {2, 3, LMIC_UNUSED_PIN},
};


void do_send() {
    // Check if there is not a current TX/RX job running

    if (LMIC.opmode & OP_TXRXPEND) {
        Serial3.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 1);
        Serial3.println(F("Packet queued"));

    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial3.print(os_getTime());
    Serial3.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial3.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial3.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial3.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial3.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial3.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial3.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial3.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial3.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial3.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial3.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK){
              Serial3.println(F("Received ack"));
              Serial3.println(F("Radio shutdown"));

              msg_sent=1;

              //shutdown();
            } else {
              Serial3.println(F("No ACK, sending again"));
              //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
              //do_send(&sendjob);
            }
            if (LMIC.dataLen) {
              Serial3.println(F("Received "));
              Serial3.println(LMIC.dataLen);
              Serial3.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            break;
        case EV_LOST_TSYNC:
            Serial3.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial3.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial3.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial3.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial3.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial3.println(F("Unknown event"));
            break;
    }
}

void setup() {
    delay(3000);

    Serial3.begin(115200);
    Serial3.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    Serial3.println(F("os_init"));
    // LMIC init
    os_init();

    Serial3.println(F("LMIC_reset"));
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    //return;
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send();

//    return;
/*


    while (!msg_sent){
      os_runloop_once();
    }
      LMIC_reset();
      delay(500);
      SPI.end();

      noInterrupts();
      //hal_disableIRQs();
      sleepMode(0);
      goToSleep(STANDBY);
*/
}

void loop() {
  //sleepMode(1);
  /*
  return;
*/
    //Serial3.println(F("Loop running"));
    if (!msg_sent){
      os_runloop_once();
    } else {
      LMIC_shutdown();
      //SPI.end();
      delay(500);


    }
}
