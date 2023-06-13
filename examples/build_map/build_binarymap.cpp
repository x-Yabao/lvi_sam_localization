#include "Location.h"
#include "parameters.h"

// 把地图以文件夹组织的形式转变为bin文件的格式

int main(int argc, char **argv) {

    readParameters();
    Location location;

    location.loadFolderMap();
    location.saveBinaryMap();

    return 0;
}
