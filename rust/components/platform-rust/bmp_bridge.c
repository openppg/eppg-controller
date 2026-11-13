/* Bosch compensation API with only bus callbacks and opaque sensor storage. */
#include <bmp3.h>
#include <platform/peripheral.h>
#include <esp_rom_sys.h>
#include <stdlib.h>
static BMP3_INTF_RET_TYPE read_reg(uint8_t reg,uint8_t* data,uint32_t len,void* ctx){(void)ctx;return eppg_i2c_read(0x77,reg,data,len)==0?BMP3_INTF_RET_SUCCESS:-1;}
static BMP3_INTF_RET_TYPE write_reg(uint8_t reg,const uint8_t* data,uint32_t len,void* ctx){(void)ctx;return eppg_i2c_write(0x77,reg,data,len)==0?BMP3_INTF_RET_SUCCESS:-1;}
static void wait_us(uint32_t us,void* ctx){(void)ctx;esp_rom_delay_us(us);}
static int valid_calibration(struct bmp3_dev* d){
 uint8_t trim[21], stored=0,crc=0xff;
 if(bmp3_get_regs(BMP3_REG_CALIB_DATA,trim,sizeof(trim),d)!=BMP3_OK||bmp3_get_regs(0x30,&stored,1,d)!=BMP3_OK)return 0;
 for(unsigned i=0;i<sizeof(trim);++i){uint8_t byte=trim[i];for(int bit=0;bit<8;++bit){int carry=(crc^byte)&0x80;crc<<=1;byte<<=1;if(carry)crc^=0x1d;}}
 return stored==(uint8_t)(crc^0xff);
}
void* eppg_bmp_new(void){struct bmp3_dev* d=calloc(1,sizeof(*d));if(!d)return NULL;d->intf=BMP3_I2C_INTF;d->read=read_reg;d->write=write_reg;d->delay_us=wait_us;if(bmp3_soft_reset(d)!=BMP3_OK||bmp3_init(d)!=BMP3_OK||!valid_calibration(d)){free(d);return NULL;}return d;}
void eppg_bmp_free(void* p){free(p);}
int eppg_bmp_config(void* ptr,uint8_t temp_os,uint8_t pressure_os,uint8_t iir,uint8_t odr){struct bmp3_dev* d=ptr;d->settings.temp_en=BMP3_ENABLE;d->settings.press_en=BMP3_ENABLE;d->settings.odr_filter.temp_os=temp_os;d->settings.odr_filter.press_os=pressure_os;d->settings.odr_filter.iir_filter=iir;d->settings.odr_filter.odr=odr;return bmp3_set_sensor_settings(BMP3_SEL_TEMP_EN|BMP3_SEL_PRESS_EN|BMP3_SEL_TEMP_OS|BMP3_SEL_PRESS_OS|BMP3_SEL_IIR_FILTER|BMP3_SEL_ODR,d);}
int eppg_bmp_sample(void* ptr,double* temp,double* pressure){struct bmp3_dev* d=ptr;d->settings.op_mode=BMP3_MODE_FORCED;if(bmp3_set_op_mode(d)!=BMP3_OK)return -1;struct bmp3_data data={0};if(bmp3_get_sensor_data(BMP3_TEMP|BMP3_PRESS,&data,d)!=BMP3_OK)return -1;*temp=data.temperature;*pressure=data.pressure;return 0;}
