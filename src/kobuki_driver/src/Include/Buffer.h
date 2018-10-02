#define BUFFER_LEN 100000
#define BUFFER_WORD_LEN 1

#include <stdint.h>

class Buffer{

    private:

        uint8_t buf[BUFFER_LEN];
        int tail;
        int head;
        int count;

    public:

        Buffer();
        ~Buffer();

        void push(uint8_t);
        uint8_t pop();

	    void clear();
};