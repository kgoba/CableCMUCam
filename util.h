#include <stdlib.h>
#include <stdint.h>


float clip(float value, float minimum, float maximum);


template<typename T, int size>
class FIFO
{
    T buffer[size];
    size_t head, count;
 
public:
    FIFO() {
      head = count = 0;
    }
    bool put(const T& data) {
      if (count == size) return false;
      size_t tail = head + count;
      if (tail >= size) tail -= size;
      buffer[tail] = data;
      count++;
      return true;
    }
    bool get(T& data) {
      if (count == 0) return false;
      data = buffer[head];
      head++;
      if (head >= size) head -= size;
      count--;
      return true;
    }
    size_t available() {
      return count;
    }
    size_t free() {
      return size - count;
    }
    bool empty() {
      return count == 0;
    }
};



struct PID {
  float kp, td;
  float dt;
  
  float cv;
  float sp;
  float error;
  
  PID(float dt, float kp = 1, float td = 0) {
    this->dt = dt;
    this->kp = kp;
    this->td = td;
    this->sp = 0;
    this->error = 0;
  }
  
  void setKP(float kp) {
    this->kp = kp;
  }
  
  void setTD(float td) {
    this->td = td;
  }
  
  void setSP(float sp) {
    this->sp = sp;
  }
  
  float update(float pv) {
    float newError = pv - sp;
    cv = kp * (newError + (newError - error) * td / dt);
    error = newError;
    return cv;
  }
  
  float getCV() {
    return cv;
  }
  
  float getCV(float min, float max) {
    return clip(cv, min, max);
  }
};


