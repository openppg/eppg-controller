// Host-only adapter: Rust's production Flasher reads the same C flash staging
// implementation used on S3, backed by NOR erase/program/power-cut semantics.
#include "image_staging.h"
#include "devices/peripherals.h"
#include <new>
struct HostStage {
  eppg::sim::devices::NorFlash flash{EPPG_STAGE_DATA_OFFSET+EPPG_STAGE_MAX_IMAGE};
  eppg_image_store store{};
  HostStage(){eppg_stage_io io{this,static_cast<uint32_t>(flash.size()),
    [](void*c,uint32_t o,void*p,size_t n){return static_cast<HostStage*>(c)->flash.read(o,static_cast<uint8_t*>(p),n)?0:-1;},
    [](void*c,uint32_t o,const void*p,size_t n){return static_cast<HostStage*>(c)->flash.program(o,static_cast<const uint8_t*>(p),n)?0:-1;},
    [](void*c,uint32_t o,size_t n){return static_cast<HostStage*>(c)->flash.erase(o,n)?0:-1;}};eppg_image_store_init(&store,&io);}
};
extern "C" {
void* eppg_host_stage_new(){return new(std::nothrow)HostStage;}
void eppg_host_stage_delete(void*p){delete static_cast<HostStage*>(p);}
bool eppg_host_stage_begin(void*p,uint32_t n){return eppg_image_store_begin(&static_cast<HostStage*>(p)->store,n);}
bool eppg_host_stage_write(void*p,uint32_t o,const uint8_t*b,size_t n){return eppg_image_store_write(&static_cast<HostStage*>(p)->store,o,b,n)>=0;}
bool eppg_host_stage_read(void*p,uint32_t o,uint8_t*b,size_t n){return eppg_image_store_read(&static_cast<HostStage*>(p)->store,o,b,n);}
bool eppg_host_stage_seal(void*p){return eppg_image_store_finish(&static_cast<HostStage*>(p)->store);}
void eppg_host_stage_abort(void*p){eppg_image_store_abort(&static_cast<HostStage*>(p)->store);}
size_t eppg_host_stage_ram(){return sizeof(eppg_image_store);}
}
