#ifndef HL_ASSERT_H
#define HL_ASSERT_H


#include <stdio.h>
#include <stdlib.h>

#define HL_ASSERT(expr)                                     \
    do                                                      \
    {                                                       \
        if (!(expr))                                        \
        {                                                   \
            printf("%s:%d: %s\n", __FILE__, __LINE__, #expr); \
        }                                                   \
    } while (0)


#endif