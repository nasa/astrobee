/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * Based off libsurvive: https://github.com/cnlohr/libsurvive
 */

#include "vive_data_ootx.h"

// For the crc32 function
#include <zlib.h>

// Avoids compiler reording with -O2
union custom_float {
  uint32_t i;
  float f;
};

// Converts a 16bit float to a 32 bit float
static float convert_float(uint8_t* data) {
  uint16_t x = *(uint16_t*)data;
  union custom_float fnum;
  fnum.f = 0;
  // handle sign
  fnum.i = (x & 0x8000)<<16;
  if ((x & 0x7FFF) == 0) return fnum.f; //signed zero
  if ((x & 0x7c00) == 0) {
    // denormalized
    x = (x&0x3ff)<<1; // only mantissa, advance intrinsic bit forward
    uint8_t e = 0;
    // shift until intrinsic bit of mantissa overflows into exponent
    // increment exponent each time
    while ((x&0x0400) == 0) {
      x<<=1;
      e++;
    }
    fnum.i |= ((uint32_t)(112-e))<<23;    // bias exponent to 127
    fnum.i |= ((uint32_t)(x&0x3ff))<<13;  // insert mantissa
    return fnum.f;
  }
  if((x&0x7c00) == 0x7c00) {
    // for infinity, fraction is 0; for NaN, fraction is anything non zero
    // we shift because the mantissa of a NaN can have meaning
    fnum.i |= 0x7f800000 | ((uint32_t)(x & 0x3ff))<<13;
    return fnum.f;
  }
  fnum.i |= ((((uint32_t)(x & 0x7fff)) + 0x1c000u) << 13);
  return fnum.f;
}

// Convert the packet
static void decode_packet(tracker_t * tracker, uint8_t id,
  uint8_t *data, uint32_t tc) {
  if (id < MAX_NUM_LIGHTHOUSES) {
    // Populate this data
    lighthouse_t * lh = &tracker->driver->lighthouses[id];
    lh->id = id;
    lh->fw_version = *(uint16_t*)(data + 0x00);
    sprintf(lh->serial, "%u", *(uint32_t*)(data + 0x02));
    lh->motors[0].phase = convert_float(data + 0x06);
    lh->motors[1].phase = convert_float(data + 0x08);
    lh->motors[0].tilt = convert_float(data + 0x0a);
    lh->motors[1].tilt = convert_float(data + 0x0c);
    lh->sys_unlock_count = *(uint8_t*)(data + 0x0e);
    lh->hw_version = *(uint8_t*)(data + 0x0f);
    lh->motors[0].curve = convert_float(data + 0x10);
    lh->motors[1].curve = convert_float(data + 0x12);
    lh->accel[0] = *(int8_t*)(data + 0x14);
    lh->accel[1] = *(int8_t*)(data + 0x15);
    lh->accel[2] = *(int8_t*)(data + 0x16);
    lh->motors[0].gibphase = convert_float(data + 0x17);
    lh->motors[1].gibphase = convert_float(data + 0x19);
    lh->motors[0].gibmag = convert_float(data + 0x1b);
    lh->motors[1].gibmag = convert_float(data + 0x1d);
    lh->mode_current = *(int8_t*)(data + 0x1f);
    lh->sys_faults = *(int8_t*)(data + 0x20);
    lh->timestamp = tc;
    // Push the new lighthouse data to the callee
    if (tracker->driver->lighthouse_cb)
      tracker->driver->lighthouse_cb(lh);
  } else {
    printf("Lighthouse ID out of range\n");
  }
}

// Swap endianness of 16 bit unsigned integer
static uint16_t swaps(uint16_t val) {
    return    ((val << 8) & 0xff00)
            | ((val >> 8) & 0x00ff);
}

// Swap endianness of 32 bit unsigned integer
static uint32_t swapl(uint32_t val) {
  return      ((val >> 24) & 0x000000ff)
            | ((val << 8)  & 0x00ff0000)
            | ((val >> 8)  & 0x0000ff00)
            | ((val << 24) & 0xff000000);
}

// Process a single bit of the OOTX data
void ootx_feed(tracker_t * tracker,
  uint8_t lh, uint8_t bit, uint32_t tc) {
  // OOTX decoders to gather base station configuration
  if (lh >= MAX_NUM_LIGHTHOUSES)
    return;
  // Get the correct context for this OOTX
  ootx_t *ctx = &tracker->ootx[lh];
  // Always check for preamble and reset if needed
  if (bit) {
    if (ctx->preamble >= PREAMBLE_LENGTH) {
      // printf("Preamble found\n");
      ctx->state = LENGTH;
      ctx->length = 0;
      ctx->pos = 0;
      ctx->syn = 0;
      ctx->preamble = 0;
      return;
    }
    ctx->preamble = 0;
  } else {
    ctx->preamble++;
  }

  // State machine
  switch (ctx->state) {
  case PREAMBLE:
    return;
  case LENGTH:
    if (ctx->syn == 16) {
      ctx->length = swaps(ctx->length);   // LE to BE
      // printf("LEN: %u for LH %u\n", ctx->length, lh);
      ctx->pad = (ctx->length % 2);       // Force even num bytes
      // printf("PAD: %u for LH %u\n", ctx->pad , lh);
      ctx->state = PREAMBLE;
      if (ctx->length + ctx->pad <= MAX_PACKET_LEN) {
        // printf("[PRE] -> [PAY]\n");
        ctx->state = PAYLOAD;
        ctx->syn = ctx->pos = 0;
        memset(ctx->data, 0x0, MAX_PACKET_LEN);
      }
      return;
    }
    ctx->length |= (((uint16_t)bit) << (15 - ctx->syn++));
    return;
  case PAYLOAD:
    // Decrement the pointer every 8 bits to find the byte offset
    if (ctx->syn == 8 || ctx->syn == 16) {
      // Increment the byte offset
      ctx->pos++;
      // If we can't decrement pointer then we have received all the data
      if (ctx->pos == ctx->length + ctx->pad) {
        // printf("[PAY] -> [CRC]\n");
        ctx->state = CHECKSUM;
        ctx->syn = ctx->pos = 0;
        ctx->crc = 0;
        return;
      }
    }
    // The 17th bit is a sync bit, and should be swallowed
    if (ctx->syn == 16) {
      ctx->syn = 0;
      return;
    }
    // Append data to the current byte in the sequence
    ctx->data[ctx->pos] |= (bit << (7 - ctx->syn++ % 8));
    return;
  case CHECKSUM:
    // Decrement the pointer every 8 bits to find the byte offset
    if (ctx->syn == 8 || ctx->syn == 16) {
      ctx->pos++;
      if (ctx->pos == 4) {
        // Calculate the CRC
        uint32_t crc = crc32( 0L, 0 /*Z_NULL*/, 0 );
        crc = crc32(crc, ctx->data, ctx->length);
        // Print some debug info
        // printf("[CRC] -> [PRE]\n");
        // printf("[CRC] RX = %08x\n", swapl(ctx->crc));
        // printf("[CRC] CA = %08x\n", crc);
        if (crc == swapl(ctx->crc))
          decode_packet(tracker, lh, ctx->data, tc);
        // Return to state
        ctx->state = PREAMBLE;
        ctx->pos = ctx->syn = 0;
        ctx->preamble = 0;
        ctx->length = 0;
        return;
      }
    }
    // The 17th bit is a sync bit, and should be swallowed
    if (ctx->syn == 16) {
      ctx->syn = 0;
      return;
    }
    ctx->crc |= (((uint32_t)bit) << (31 - (ctx->pos * 8 + ctx->syn++ % 8)));
    return;
  }
}
