#include "Espnow.h"
#include "msp.h"
#include "msptypes.h"
#include "Output.h"
#include "HardwareConfig.h"
#include "ADC.h"
#include "RX5808.h"
#include "Comms.h"

#include <Arduino.h>
#include <esp_now.h>
#include <freertos/ringbuf.h>
#include <esp_attr.h>

#define RING_BUFFER_SIZE 256

MSP DRAM_ATTR msp_parser;

#if RING_BUFFER_SIZE
RingbufHandle_t DRAM_ATTR buffer_handle;
//StaticRingbuffer_t DRAM_ATTR buffer_struct;
//uint8_t DRAM_ATTR buffer_storage[RING_BUFFER_SIZE];
//buffer_handle = xRingbufferCreateStatic(sizeof(buffer_storage), RINGBUF_TYPE_BYTEBUF, buffer_storage, &buffer_struct);
#endif // RING_BUFFER_SIZE

static void IRAM_ATTR esp_now_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    int iter;
    bool msp_rcvd = 0;

    /* No data or peer is unknown => ignore */
    if (!data_len || !esp_now_is_peer_exist(mac_addr))
        return;

    msp_rcvd = msp_parser.processReceivedByte(data[0]);

    // Check if message is MSP
    for (iter = 1; (iter < data_len) && (msp_rcvd == 0) && msp_parser.mspOngoing(); iter++) {
        msp_rcvd = msp_parser.processReceivedByte(data[iter]);
    }

    // Only MSP packet or laptime is expected
    if (msp_rcvd) {
        /* Process the received MSP packet */
        mspPacket_t &msp_in = msp_parser.getPacket();
        if (msp_in.type == MSP_PACKET_V2_COMMAND && msp_in.function == MSP_SET_VTX_CONFIG) {
            uint8_t * payload = (uint8_t*)msp_in.payload;

            uint16_t freq = payload[1];
            freq <<= 8;
            freq += payload[0];

            uint8_t index = getFreqIndexByFrequency(freq);
            uint8_t band = index / 8, NodeAddrByte = 0;
            uint8_t channel = index % 8;

            // Set channel to RF module
            setPilotBandChannel(NodeAddrByte, band, channel);
            // Send info to all clients
            SendVRxBand(NodeAddrByte);
            SendVRxChannel(NodeAddrByte);
            SendVRxFreq(NodeAddrByte);

        //} else {
        //    Serial.println(" UNKNOWN MSP!");
        }
#if !RING_BUFFER_SIZE
    } else if (strnlen((char*)data, 32) < 32) {
        /* Race control command */
        output_input_callback((uint8_t*)data, data_len);
#else
    } else {
        xRingbufferSend(buffer_handle, data, data_len, (TickType_t)0);
#endif
    }

    msp_parser.markPacketFree();
}


void espnow_init(uint8_t if_type)
{
#if defined(ESP_NOW_PEERS)
#if RING_BUFFER_SIZE
    buffer_handle = xRingbufferCreate(RING_BUFFER_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (!buffer_handle) {
        Serial.println("[ERROR] Unable to allocate rign buffer!");
    }
#endif // RING_BUFFER_SIZE
    /*
    STA MAC Address: F0:08:D1:D4:ED:7C
    AP MAC Address: F0:08:D1:D4:ED:7D
    */

    Serial.print("Initialize ESP-NOW... ");
    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(esp_now_recv_cb);

        esp_now_peer_info_t peer_info = {
            .peer_addr = {0},
            .lmk = {0},
            .channel = 1,
            .ifidx = (wifi_interface_t)if_type,
            .encrypt = 0,
            .priv = NULL
        };
        uint8_t peers[][ESP_NOW_ETH_ALEN] = ESP_NOW_PEERS;
        uint8_t num_peers = sizeof(peers) / ESP_NOW_ETH_ALEN;
        for (uint8_t iter = 0; iter < num_peers; iter++) {
            memcpy(peer_info.peer_addr, peers[iter], ESP_NOW_ETH_ALEN);
            if (ESP_OK != esp_now_add_peer(&peer_info))
                Serial.println("ESPNOW add peer failed!");
        }

        Serial.println("DONE");

    } else {
        Serial.println("ESPNOW init failed!");
    }
#endif // ESP_NOW_PEERS
}


void IRAM_ATTR espnow_sendPacket(void* output, uint8_t* data, size_t len)
{
#if defined(ESP_NOW_PEERS)
    // TODO: Send just required packets!!
    if (data[2] == 'L') return;

    esp_now_send(NULL, data, len);
#endif // ESP_NOW_PEERS
}


void IRAM_ATTR espnow_processPacket(void* output)
{
#if defined(ESP_NOW_PEERS)
#if RING_BUFFER_SIZE
    if (buffer_handle && output) {
        output_t* out = (output_t*)output;
        size_t num_bytes = 0;
        /*
         * Normal cmd max: R*RBABECAFE\n ==> 12B
         * Extended cmd max: E*RBABE\n ==> 8B
         *    ==> Use 32B to make sure there is enough space for even multiple commands
        */
        uint8_t rx_buff[32];
        uint8_t * ptr = (uint8_t*)xRingbufferReceiveUpTo(buffer_handle, &num_bytes, pdMS_TO_TICKS(1), sizeof(rx_buff));
        if (ptr) {
            out->handle_input_callback(ptr, num_bytes);
            vRingbufferReturnItem(buffer_handle, (void *)ptr);
        }
    }
#endif // RING_BUFFER_SIZE
#endif // ESP_NOW_PEERS
}
