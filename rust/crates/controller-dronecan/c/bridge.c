/* Generic libcanard transport adapter. Application decisions remain in Rust. */
#include <canard.h>
#include <stdlib.h>
#include <string.h>
#define DC_BUFFER 1024
#define DC_QUEUE 16
struct spec{uint16_t id;uint8_t source;uint8_t type;uint64_t signature;};
struct transfer{uint16_t id,len;uint8_t source,type,tid;uint8_t* payload;};
struct bridge {CanardInstance instance;_Alignas(8) uint8_t pool[12288];struct spec specs[32];unsigned spec_count;struct transfer queue[DC_QUEUE];unsigned head,len;};
static bool accept(const CanardInstance* i,uint64_t* sig,uint16_t id,CanardTransferType type,uint8_t source){struct bridge* b=i->user_reference;for(unsigned n=0;n<b->spec_count;n++){struct spec* s=&b->specs[n];if(s->id==id&&s->type==(uint8_t)type&&(s->source==0||s->source==source)){*sig=s->signature;return true;}}return false;}
/* Keep the same 16-transfer/1024-byte limits without reserving 16 KiB empty.
 * libcanard owns RX storage only during this callback, so copy completed payloads.
 * Allocation failure drops this transfer just like a full application queue. */
static void received(CanardInstance* i, CanardRxTransfer* t) {
    struct bridge* b = i->user_reference;
    if (b->len == DC_QUEUE || t->payload_len > DC_BUFFER) return;
    uint8_t* payload = t->payload_len ? malloc(t->payload_len) : NULL;
    if (t->payload_len && !payload) return;
    struct transfer* out = &b->queue[(b->head + b->len) % DC_QUEUE];
    out->id = t->data_type_id;
    out->source = t->source_node_id;
    out->type = t->transfer_type;
    out->tid = t->transfer_id;
    out->len = t->payload_len;
    out->payload = payload;
    for (unsigned n = 0; n < t->payload_len; n++)
        canardDecodeScalar(t, n * 8, 8, false, &payload[n]);
    b->len++;
}
void* dc_new(void){struct bridge* b=calloc(1,sizeof(struct bridge));if(b)canardInit(&b->instance,b->pool,sizeof(b->pool),received,accept,b);return b;}
void dc_free(void* ptr) {
    struct bridge* b = ptr;
    if (!b) return;
    for (unsigned n = 0; n < b->len; n++)
        free(b->queue[(b->head + n) % DC_QUEUE].payload);
    free(b);
}
/* Single owner: the pending head cannot change between size and take. */
int dc_next_size(void* ptr) {
    struct bridge* b = ptr;
    return b->len ? b->queue[b->head].len : -1;
}
void dc_node(void* ptr,uint8_t node){struct bridge* b=ptr;if(node>0&&node<=127)b->instance.node_id=node;}
int dc_register(void* ptr,uint16_t id,uint8_t type,uint8_t source,uint64_t signature){struct bridge* b=ptr;if(b->spec_count>=32)return -1;b->specs[b->spec_count++]=(struct spec){id,source,type,signature};return 0;}
int dc_rx(void* ptr,uint32_t id,const uint8_t* data,uint8_t len,uint64_t now){if(len>8)return -1;struct bridge* b=ptr;CanardCANFrame f={0};f.id=id|CANARD_CAN_FRAME_EFF;f.data_len=len;memcpy(f.data,data,len);return canardHandleRxFrame(&b->instance,&f,now);}
int dc_tx(void* ptr,uint32_t* id,uint8_t* data,uint8_t* len){struct bridge* b=ptr;const CanardCANFrame* f=canardPeekTxQueue(&b->instance);if(!f)return 0;*id=f->id&CANARD_CAN_EXT_ID_MASK;*len=f->data_len;memcpy(data,f->data,*len);return 1;}
void dc_tx_pop(void* ptr){struct bridge* b=ptr;canardPopTxQueue(&b->instance);}
int dc_take(void* ptr,uint16_t* id,uint8_t* type,uint8_t* source,uint8_t* tid,uint8_t* data,uint16_t cap){struct bridge* b=ptr;if(!b->len)return -1;struct transfer* t=&b->queue[b->head];if(t->len>cap)return -2;*id=t->id;*type=t->type;*source=t->source;*tid=t->tid;if(t->len)memcpy(data,t->payload,t->len);free(t->payload);t->payload=NULL;int len=t->len;b->head=(b->head+1)%DC_QUEUE;b->len--;return len;}
int dc_request(void* ptr,uint8_t destination,uint16_t id,uint64_t signature,uint8_t* tid,const uint8_t* payload,uint16_t len){struct bridge* b=ptr;CanardTxTransfer tx={0};tx.transfer_type=CanardTransferTypeRequest;tx.data_type_signature=signature;tx.data_type_id=id;tx.inout_transfer_id=tid;tx.priority=CANARD_TRANSFER_PRIORITY_HIGHEST;tx.payload=payload;tx.payload_len=len;return canardRequestOrRespondObj(&b->instance,destination,&tx);}
void dc_cleanup(void* ptr,uint64_t now){struct bridge* b=ptr;canardCleanupStaleTransfers(&b->instance,now);}

int dc_respond(void* ptr,uint8_t destination,uint16_t id,uint64_t signature,uint8_t tid,const uint8_t* payload,uint16_t len){struct bridge* b=ptr;CanardTxTransfer tx={0};tx.transfer_type=CanardTransferTypeResponse;tx.data_type_signature=signature;tx.data_type_id=id;tx.inout_transfer_id=&tid;tx.priority=CANARD_TRANSFER_PRIORITY_HIGHEST;tx.payload=payload;tx.payload_len=len;return canardRequestOrRespondObj(&b->instance,destination,&tx);}
