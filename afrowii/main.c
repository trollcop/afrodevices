#include "config.h"
#include "def.h"
#include "sysdep.h"

void setup(void);
void loop(void);

int main(void)
{
    // system dependent hardware init
    hw_init();

    setup();

    for (;;)
        loop();
}
