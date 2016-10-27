/* Driver for Spektrum satellite Rx
 * M. Nentwig
 * version 0.1, 25.10.2011
 * Connections:
 * - orange wire to 3.3V regulated out
 * - black wire to GND
 * - grey wire to pin 10 (for example)
 * - Serial serIn(p9, p10);
 * - serIn.baud(115200);
 * - see also http://diydrones.ning.com/profiles/blog/show?id=705844%3ABlogPost%3A64228
 */
#ifndef SPEKTRX_H
#define SPEKTRX_H
 
class SpektrumDSM {
public:
  
};
 
typedef struct _spektRx {
    char state;
    unsigned char data[14];
    unsigned short out[7];
    bool valid;
    long frameNum;
} spektRx_t;
 
/* Initialize the spektRx */
void spektRx_init(spektRx_t* self);
 
/* Feed a single byte of received data into the spektRx */
void spektRx_runChar(spektRx_t* self, unsigned char c);
 
/* alternatively, let it loose on the serial input
 * checks readable -> non-blocking
 */
void spektRx_runSerial(spektRx_t* self, Serial* s);
 
/* Check, whether a frame has been received */
bool spektRx_hasValidFrame(spektRx_t* self);
 
/* If a valid frame is available, retrieve one channel
 * and convert to float (quick-and-dirty option)  */
unsigned short* spektRx_getChannelData(spektRx_t* self);
 
/* Gets pointer to array with all 7 input channels as 
* 10-bit unsigned float (preferred way of reading) */
float spektRx_getChannelValue(spektRx_t* self, char ixChan);
 
/* Get the number of the currently received frame. 
* Starts with 1 */
long spektRx_getFrameNum(spektRx_t* self);
 
/* Mark the current frame as invalid.
* spektRx_haseValidFrame() will return false, until
* the next frame has been received */
void spektRx_invalidateFrame(spektRx_t* self);
#endif
          
