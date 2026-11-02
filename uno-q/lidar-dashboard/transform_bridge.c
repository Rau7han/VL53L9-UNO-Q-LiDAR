/* Host adapter for STSW-IMG053. One pipeline produces XYZ and filter status;
 * coordinates are the library's millimetres, not Android's nominal metres.
 * Factory calibration must come from the connected sensor.
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "media.h"
#include "vl53l9_transform.h"
#include "vl53l9_calib_utils.h"
#define PIXELS (54u*42u)
#define RAW_SIZE 14842u
_Static_assert(sizeof(vl53l9_metadata_t)==100, "ST metadata ABI mismatch");
_Static_assert(offsetof(vl53l9_metadata_t,frame_width)==52, "ST metadata layout mismatch");
typedef struct { transform_t *transform; unsigned char calibration[2332]; } bridge_t;

/* The vendor pipeline trusts metadata dimensions and DSS table indices.
 * Check the fixed firmware ABI before it allocates or indexes any arrays. */
int l9_validate_frame(const unsigned char *raw,unsigned length){
  if(!raw || length!=RAW_SIZE)return -1;
  vl53l9_metadata_t meta;
  memcpy(&meta,raw+RAW_SIZE-sizeof(meta),sizeof(meta));
  if(meta.frame_width!=54 || meta.frame_height!=42 || meta.crop_enable ||
     meta.binning!=2 || meta.format!=1 || (meta.nb_step!=6 && meta.nb_step!=7))return -2;
  for(unsigned i=PIXELS*6;i<RAW_SIZE-sizeof(meta);i++)
    if((raw[i]&7)>=DSS_COEFFS_NB || ((raw[i]>>4)&7)>=DSS_COEFFS_NB)return -3;
  return 0;
}

static int caps(transform_t *t,const char *name,const char *format,unsigned w,unsigned h){
  property_t props[3]={
    {"format",{.val.v_string=format,.tid=VTID_STRING}},
    {"width",{.val.v_uint32=w,.tid=VTID_UINT32}},
    {"height",{.val.v_uint32=h,.tid=VTID_UINT32}}};
  properties_t *p=properties_new(3);
  if(!p)return -1;
  for(unsigned i=0;i<3;i++)properties_add(p,&props[i]);
  capabilities_t *c=capabilities_new_simple(&p);
  int result=c?transform_set_stream_capabilities(t,name,c):-1;
  properties_free(p,NULL);if(c)capabilities_free(c,NULL);
  return result;
}
void l9_destroy(void *handle){
  bridge_t *b=handle;if(!b)return;
  if(b->transform){transform_finalize(b->transform);transform_release(b->transform);vl53l9_transform_destroy(b->transform);}
  free(b);
}
void *l9_create(const unsigned char *calibration,unsigned length,int temporal){
  if(!calibration || length!=2332)return NULL;
  bridge_t *b=calloc(1,sizeof(*b));if(!b)return NULL;
  memcpy(b->calibration,calibration,length);
  b->transform=vl53l9_transform_create();if(!b->transform)goto fail;
  if(transform_initialize(b->transform) || caps(b->transform,"raw","3DMD",RAW_SIZE,1) ||
     caps(b->transform,"depth","ZAPC",54,42) || caps(b->transform,"status","CU32",54,42) || caps(b->transform,"reflectance","RF32",54,42))goto fail;
  if(transform_set_control(b->transform,"calib-buffer",(value_t){.val.v_ptr=b->calibration,.tid=VTID_POINTER}) ||
     transform_set_control(b->transform,"cover-glass",(value_t){.val.v_bool=false,.tid=VTID_BOOL}) ||
     transform_set_control(b->transform,"bypass-tnr-algo",(value_t){.val.v_bool=!temporal,.tid=VTID_BOOL}) ||
     transform_prepare(b->transform))goto fail;
  return b;
fail:l9_destroy(b);return NULL;
}
int l9_process_full(void *handle,const unsigned char *raw,unsigned length,float *xyzc,unsigned *status,float *reflectance){
  bridge_t *b=handle;if(!b || !raw || length!=RAW_SIZE || !xyzc || !status || !reflectance)return -1;
  int validation=l9_validate_frame(raw,length);
  if(validation)return validation;
  memory_t m[4]={
    {.data=(unsigned char *)raw,.size=RAW_SIZE,.maxsize=RAW_SIZE},
    {.data=(unsigned char *)xyzc,.size=PIXELS*16,.maxsize=PIXELS*16},
    {.data=(unsigned char *)status,.size=PIXELS*4,.maxsize=PIXELS*4},
    {.data=(unsigned char *)reflectance,.size=PIXELS*4,.maxsize=PIXELS*4}};
  memories_t mm[4];stream_buffer_t sb[4];const char *names[]={"raw","depth","status","reflectance"};
  for(unsigned i=0;i<4;i++){
    mm[i]=(memories_t){.items=&m[i],.size=1,.capacity=1,.item_size=sizeof(memory_t)};
    sb[i]=(stream_buffer_t){.name=names[i],.buffer={.memories=&mm[i]}};
  }
  stream_buffers_t streams={.items=sb,.size=4,.capacity=4,.item_size=sizeof(stream_buffer_t)};
  return transform_process_stream(b->transform,&streams);
}

/* Exact protocol checksum; unsigned arithmetic wraps modulo 2^32. */
uint32_t l9_fnv1a(const unsigned char *data, unsigned length){
  uint32_t hash=2166136261u;
  for(unsigned i=0;i<length;i++)hash=(hash^data[i])*16777619u;
  return hash;
}

/* Preserve the original ABI for web tools and older clients. */
int l9_process(void *handle,const unsigned char *raw,unsigned length,float *xyzc,unsigned *status){
 float reflectance[PIXELS];
 return l9_process_full(handle,raw,length,xyzc,status,reflectance);
}
