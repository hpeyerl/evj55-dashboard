// vss_test.cpp - standalone runtime test for vss_reader (DRAFT).
// Build:  g++ -O2 vss_test.cpp vss_reader.cpp -lgpiod -lpthread -o vss_test
// Run:    sudo ./vss_test            (VSS = gpiochip3 line 27)
// Drive VSS_SIG with the bench generator (0-12V square) and watch the printed
// speed track frequency:  mph ~= (f_hz * 3600) / pulses_per_mile.
#include "vss_reader.h"
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

int main(int argc, char **argv)
{
    const char  *chip = (argc > 1) ? argv[1] : "gpiochip3";
    unsigned int line = (argc > 2) ? (unsigned)atoi(argv[2]) : 27;

    vss_cal_t cal; vss_default_cal(&cal);
    if (vss_start(chip, line, &cal) != 0) { fprintf(stderr, "vss_start failed\n"); return 1; }
    printf("vss_test: %s line %u, pulses/mile=%.0f. Ctrl-C to stop.\n",
           chip, line, cal.pulses_per_mile);
    for (;;) {
        printf("  speed = %6.1f mph (%6.1f km/h)   odo = %.4f mi\n",
               vss_speed_mph(), vss_speed_mph() * 1.60934, vss_miles());
        fflush(stdout);
        sleep(1);
    }
    return 0;
}
