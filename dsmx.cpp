#include "mbed.h"
#include "dsmx.h"
 
#define NUM_CHAN (7)
#define NUM_BYTES_IN_FRAME (2*NUM_CHAN+2)
 
void spektRx_init(spektRx_t* self){
    self -> state = 0;
    self -> valid = false;
    self -> frameNum = 0L;
}
 
bool spektRx_hasValidFrame(spektRx_t* self){
    return self->valid;
}
 
void spektRx_runChar(spektRx_t* self, unsigned char c){
    switch(self->state){
        case 0: /* new frame cycle */
            /* first preamble byte received: now expect 2nd preamble byte */
            self->state = (c == 0x03) ? 1 : 0;
            break;
 
        case 1:
            /* 2nd preamble byte received: Now expect first data byte */             
            self->state = (c == 0x01) ? 2 : 0;
            break;
 
        default:
            /* store received byte */
            self->data[self -> state - 2] = c;
            ++self->state;
 
            if (self->state == NUM_BYTES_IN_FRAME){
                /* one complete frame was received.
                 * - Copy the data
                 * - mask out bits 10..15
                 */
                int ix; 
                for (ix = 0; ix < NUM_CHAN; ++ix){
                    self->out[ix] = ((self->data[2*ix] & 3) << 8) | self->data[2*ix+1];
                }
 
                /* output data is now valid */
                self->valid = true;
                ++self->frameNum;
 
                /* ready for the next frame */
                self->state = 0;
            } /* if frame complete */
    } /* switch state */
}
 
void spektRx_runSerial(spektRx_t* self, Serial* s){
    while(s -> readable())
        spektRx_runChar(self, s -> getc());
}
 
unsigned short* spektRx_getChannelData(spektRx_t* self){
    return (unsigned short*) self -> out;
}
 
float spektRx_getChannelValue(spektRx_t* self, char ixChan){
    return (float)self->out[ixChan] / 512.0f - 1.0f;
}
 
long spektRx_getFrameNum(spektRx_t* self){
    return self->frameNum;
}
 
void spektRx_invalidateFrame(spektRx_t* self){
    self->valid = false;
}
 
