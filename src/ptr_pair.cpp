#include "ptr_pair.hpp"
#include <iostream>

ptr_pair::ptr_pair()
{
    start = NULL;
    end = NULL;
};


ptr_pair::ptr_pair(float* s, float* e)
{
    start = s;
    end = e;
};