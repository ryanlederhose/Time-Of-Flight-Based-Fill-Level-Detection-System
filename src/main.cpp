#include "Facade/Facade.h"

int main(int argc, char** argv) {

    Facade facade;

    std::thread facadeLoop(&Facade::loop, &facade);

    facadeLoop.join();

    return 0;
}