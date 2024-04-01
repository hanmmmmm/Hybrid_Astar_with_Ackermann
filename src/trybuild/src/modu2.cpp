
#include "modu2.h"


namespace mytest
{
    Modu2::Modu2()
    {
    }

    Modu2::~Modu2()
    {
    }

    void Modu2::modu2Func()
    {
        std::cout << "Hello from Modu2!" << std::endl;

        auto mt = MyTest();
        mt.printHello();
    }
}
