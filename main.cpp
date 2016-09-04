#include "lidar.h"

int main(int argc, char **argv)
{
    lidar parser(2368, 500);

    while (1)
        if (!parser.CanGetRaw())
                parser.process();
}