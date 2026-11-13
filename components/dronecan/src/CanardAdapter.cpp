#include "CanardAdapter.h"

#include "platform/runtime.h"
#include "driver/twai.h"

#include <algorithm>
#include <string.h>

// -----------------------------------------------------------------------------

#define POLLING_RATE_MS     1
#define STALE_CHECK_INTERVAL_USEC CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC

// -----------------------------------------------------------------------------

static uint64_t getTimestamp(void)
{
    static uint64_t baseTime = 0;
    static unsigned long lastMicros = 0;

    unsigned long now = platform::micros32();
    if (now < lastMicros) {
        // platform::micros32() overflowed at 32-bit max value
        baseTime += 0x100000000ULL;
    }

    lastMicros = now;

    return (now + baseTime);
}

// -----------------------------------------------------------------------------

CanardAdapterNode::CanardAdapterNode(CanardAdapter &adapter)
: _canard(NULL)
, _adapter(&adapter)
{
}

void CanardAdapterNode::_beginNode(void)
{
    _canard = &_adapter->_canard;
    _adapter->_addNode(*this);
}

// -----------------------------------------------------------------------------

CanardAdapter::CanardAdapter(void)
{
}

void CanardAdapter::begin(uint8_t *memoryPool, size_t memoryPoolSize)
{
    if (memoryPool == NULL || memoryPoolSize == 0) {
        // Handle error: invalid memory pool
        platform::console.line("CanardAdapter::begin - Bad memory pool");
        return;
    }
    canardInit(&_canard,
               memoryPool,
               memoryPoolSize,
               _onTransferReceivedTrampoline,
               _shouldAcceptTransferTrampoline,
               this);

    _lastStaleCheckTimestamp = getTimestamp();
}

void CanardAdapter::setLocalNodeId(uint8_t selfNodeId)
{
    canardSetLocalNodeID(&_canard, selfNodeId);
}

void CanardAdapter::processTxRxOnce(void)
{
    uint32_t alerts_triggered;
    // TODO: Can we set the delay to 0?
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
        platform::console.line("Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
        platform::console.line("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        platform::console.format("Bus error count: %lu\n", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
        platform::console.line("Alert: The RX queue is full causing a received frame to be lost.");
        platform::console.format("RX buffered: %lu\t", twaistatus.msgs_to_rx);
        platform::console.format("RX missed: %lu\t", twaistatus.rx_missed_count);
        platform::console.format("RX overrun %lu\n", twaistatus.rx_overrun_count);
    }

    // Receiving
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        // One or more messages received. Handle all.
        twai_message_t message;
        while (twai_receive(&message, 0) == ESP_OK) {
            const uint64_t timestamp = getTimestamp();

            CanardCANFrame rxFrame;
            memset(&rxFrame, 0, sizeof(rxFrame));

            rxFrame.id = message.identifier;
            if (message.extd) {
                rxFrame.id |= CANARD_CAN_FRAME_EFF;
            }

            rxFrame.data_len = std::min<int>(message.data_length_code, sizeof(rxFrame.data));

            if (message.rtr) {
                // Data not relevant for RTR frames
                rxFrame.id |= CANARD_CAN_FRAME_RTR;
            } else {
                for (int i = 0; i < rxFrame.data_len; ++i) {
                    rxFrame.data[i] = message.data[i];
                }
            }

            // platform::console.text("CanardAdapter::processTxRxOnce - RX - 0x");
            // platform::console.text(message.identifier, HEX);
            // platform::console.text(" - ");
            // platform::console.line(rxFrame.data_len);

            int rxRet = canardHandleRxFrame(&_canard, &rxFrame, timestamp);
            if (rxRet < 0) {
                // platform::console.text("CanardAdapter::processTxRxOnce - RX - Rejected - ");
                // platform::console.line(rxRet);
            }
        }
    }

    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&_canard)) != NULL;) {
        const uint32_t id = CANARD_CAN_EXT_ID_MASK & txf->id;

        // platform::console.text("CanardAdapter::processTxRxOnce - TX - 0x");
        // platform::console.text(id, HEX);
        // platform::console.text(" - ");
        // platform::console.line(txf->data_len);

        twai_message_t message;
        memset(&message, 0, sizeof(message));

        message.identifier = id;
        message.extd = 1;
        message.data_length_code = std::min<uint8_t>(txf->data_len, TWAI_FRAME_MAX_DLC);
        for (int i = 0; i < message.data_length_code; ++i) {
            message.data[i] = txf->data[i];
        }

        // TODO: Can we set the delay to 0?
        if (twai_transmit(&message, pdMS_TO_TICKS(POLLING_RATE_MS)) == ESP_OK) {
            // Success, remove the packet from the queue
            canardPopTxQueue(&_canard);
        } else {
            // Timeout, exit and try again later
            break;
        }
    }

    // Remove stale transfers from the queue. The timing only needs to be
    // approximate for triggering the stale check
    const uint64_t staleCheckTimestamp = getTimestamp();
    if ((staleCheckTimestamp - _lastStaleCheckTimestamp) >= STALE_CHECK_INTERVAL_USEC) {
        _lastStaleCheckTimestamp = staleCheckTimestamp;
        canardCleanupStaleTransfers(&_canard, _lastStaleCheckTimestamp);
    }
}


void CanardAdapter::_onTransferReceivedTrampoline(CanardInstance* instance,
                                                  CanardRxTransfer* transfer)
{
    CanardAdapter* adapter = static_cast<CanardAdapter*>(instance->user_reference);
    adapter->_onTransferReceived(transfer);
}

bool CanardAdapter::_shouldAcceptTransferTrampoline(const CanardInstance* instance,
                                                    uint64_t* out_data_type_signature,
                                                    uint16_t data_type_id,
                                                    CanardTransferType transfer_type,
                                                    uint8_t source_node_id)
{
    CanardAdapter* adapter = static_cast<CanardAdapter*>(instance->user_reference);
    return adapter->_shouldAcceptTransfer(out_data_type_signature,
                                          data_type_id,
                                          transfer_type,
                                          source_node_id);
}


void CanardAdapter::_addNode(CanardAdapterNode &node)
{
    // platform::console.line("CanardAdapter::_addNode");
    _nodes.push_back(&node);
}

bool CanardAdapter::_shouldAcceptTransfer(uint64_t* out_data_type_signature,
                                          uint16_t data_type_id,
                                          CanardTransferType transfer_type,
                                          uint8_t source_node_id)
{
    bool accept = false;

    for (CanardAdapterNode *node : _nodes) {
        if (node->shouldAcceptTransfer(out_data_type_signature,
                                       data_type_id,
                                       transfer_type,
                                       source_node_id)) {
            accept = true;
            break;
        }
    }

    // platform::console.text("CanardAdapter::_shouldAcceptTransfer - 0x");
    // platform::console.text(data_type_id, HEX);
    // platform::console.text(" - ");
    // platform::console.text(transfer_type);
    // platform::console.text(" - ");
    // platform::console.text(source_node_id, HEX);
    // if (accept) {
    //     platform::console.line(" - Accepted");
    // } else {
    //     platform::console.line(" - Rejected");
    // }

    return accept;
}

void CanardAdapter::_onTransferReceived(CanardRxTransfer* transfer)
{
    for (CanardAdapterNode *node : _nodes) {
        node->onTransferReceived(transfer);
    }

    // platform::console.text("CanardAdapter::_onTransferReceived - ");
    // platform::console.text(transfer->data_type_id, HEX);
    // platform::console.text(" - ");
    // platform::console.text(transfer->transfer_type);
    // platform::console.text(" - ");
    // platform::console.format("%lX\r\n", static_cast<unsigned long>(transfer->source_node_id));
}
