
#include "mytest.h"
#include "modu1.h"
#include "modu2.h"


int main(int argc, char **argv)
{
    auto a = argc;
    auto b = argv;
    if(a == 0)
    {
        
    }
    if (b == nullptr)
    {
        
    }

    mytest::MyTest mytest;
    mytest.printHello();

    mytest::Modu1 modu1;
    modu1.modu1Func();

    mytest::Modu2 modu2;
    modu2.modu2Func();

    return 0;
}

