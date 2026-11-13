#ifndef CANARD_ADAPTER_H
#define CANARD_ADAPTER_H

#include <canard.h>

#include <vector>
#include <stdint.h>

class CanardAdapter;


class CanardAdapterNode
{
public:
    CanardAdapterNode(CanardAdapter &adapter);
    virtual ~CanardAdapterNode(void) {}

    virtual bool shouldAcceptTransfer(uint64_t* out_data_type_signature,
                                      uint16_t data_type_id,
                                      CanardTransferType transfer_type,
                                      uint8_t source_node_id) = 0;

    virtual void onTransferReceived(CanardRxTransfer* transfer) = 0;

protected:
    CanardInstance *_canard;

    void _beginNode(void);

private:
    CanardAdapter *_adapter;
};


class CanardAdapter
{
public:
    CanardAdapter(void);

    void begin(uint8_t *memoryPool, size_t memoryPoolSize);

    void setLocalNodeId(uint8_t selfNodeId);

    void processTxRxOnce(void);

private:
    friend CanardAdapterNode;

    static void _onTransferReceivedTrampoline(CanardInstance* instance,
                                              CanardRxTransfer* transfer);
    static bool _shouldAcceptTransferTrampoline(const CanardInstance* instance,
                                                uint64_t* out_data_type_signature,
                                                uint16_t data_type_id,
                                                CanardTransferType transfer_type,
                                                uint8_t source_node_id);

    void _addNode(CanardAdapterNode &node);

    bool _shouldAcceptTransfer(uint64_t* out_data_type_signature,
                               uint16_t data_type_id,
                               CanardTransferType transfer_type,
                               uint8_t source_node_id);

    void _onTransferReceived(CanardRxTransfer* transfer);

    CanardInstance _canard;
    std::vector<CanardAdapterNode*> _nodes;
    uint64_t _lastStaleCheckTimestamp;
};


#endif // CANARD_ADAPTER_H
