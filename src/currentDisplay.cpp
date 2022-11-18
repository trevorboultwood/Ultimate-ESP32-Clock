#include <currentDisplay.h>
#include <stdint.h>
static uint8_t xymatrix[63][63]; 
void currentDisplay::savePixel(int x, int y)
{
    xymatrix[x][y] = 0xff;
    //xymatrix[x] = xymatrix[x] + 1<<y;
}
void currentDisplay::clearDisplay(void)
{
    for(int x = 0; x < 63; x++)
        for(int y = 0; y < 63; y++)
        {
            xymatrix[x][y] = 0x00;
        }
        //xymatrix[x] = 0;   
}

bool currentDisplay::isPixel(int x, int y)
{
    if(xymatrix[x][y] == 0xff)// & 1<<y)
        return true;
    else
        return false;

}
void currentDisplay::removePixel(int x, int y)
{
    xymatrix[x][y] = 0x00;
}