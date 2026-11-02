/* Development USB CDC descriptors; TinyUSB's example VID, not a production VID. */
#include <string.h>
#include "tusb.h"
static tusb_desc_device_t const device = {
 .bLength=sizeof(tusb_desc_device_t),.bDescriptorType=TUSB_DESC_DEVICE,.bcdUSB=0x0200,
 .bDeviceClass=TUSB_CLASS_MISC,.bDeviceSubClass=MISC_SUBCLASS_COMMON,
 .bDeviceProtocol=MISC_PROTOCOL_IAD,.bMaxPacketSize0=64,
 .idVendor=0xcafe,.idProduct=0x4010,.bcdDevice=0x0100,
 .iManufacturer=1,.iProduct=2,.iSerialNumber=3,.bNumConfigurations=1
};
uint8_t const *tud_descriptor_device_cb(void){return (uint8_t const *)&device;}
static uint8_t const config[]={
 TUD_CONFIG_DESCRIPTOR(1,2,0,TUD_CONFIG_DESC_LEN+TUD_CDC_DESC_LEN,0,500),
 TUD_CDC_DESCRIPTOR(0,4,0x81,8,0x02,0x82,64)
};
uint8_t const *tud_descriptor_configuration_cb(uint8_t index){(void)index;return config;}
uint16_t const *tud_descriptor_string_cb(uint8_t index,uint16_t langid){
 (void)langid;static uint16_t out[32];
 const char *strings[]={"","USB LiDAR","VL53L9 Full Raw","VL53L9-C552-001","Full frame CDC"};
 unsigned n;
 if(index==0){out[1]=0x0409;n=1;}
 else {if(index>=5)return NULL;n=strlen(strings[index]);if(n>31)n=31;
  for(unsigned i=0;i<n;i++)out[i+1]=(uint8_t)strings[index][i];}
 out[0]=(TUSB_DESC_STRING<<8)|(2*n+2);return out;
}
