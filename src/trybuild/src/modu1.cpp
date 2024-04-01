
#include "modu1.h"


namespace mytest
{
    Modu1::Modu1()
    {
    }

    Modu1::~Modu1()
    {
    }

    void Modu1::modu1Func()
    {
        std::cout << "Hello from Modu1!" << std::endl;

        auto mt = MyTest();
        mt.printHello();
    }
}   