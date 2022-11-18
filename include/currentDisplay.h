#ifndef CURRENTDISPLAY
#define CURRENTDISPLAY 
#include <stdint.h>
class currentDisplay{
    public:
        void savePixel(int x, int y);
        void clearDisplay(void);
        bool isPixel(int x, int y);
        uint8_t blockPattern;
        void removePixel(int x, int y);
};
#endif