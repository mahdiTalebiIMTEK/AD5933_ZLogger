#ifndef FIFOE_H
#define FIFOE_H
#define BUFFERSIZE 11600


class FifoE {

    public:
      bool putIn(int value);
      int getOut();
      int count();
    private:
      unsigned int in = 0;
      unsigned int out = 0;
      int fifo[BUFFERSIZE];
};

#endif
