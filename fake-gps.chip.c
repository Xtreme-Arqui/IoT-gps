// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Bolu Agbana (OBAA)

// original project https://wokwi.com/projects/333785509332517459

  #include "wokwi-api.h"
  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>    // contains strlen() function

  #define LEN(arr) ((int)(sizeof(arr) / sizeof(arr)[0]))    // macro

  #define SECOND 1000000    // micros

  // A NMEA 0183 sentence can have a maximum of 80 characters plus a
  // carriage return and a line feed

  const char gps_tx_data[][80] = { // GPRMC & GPGGA (Hypothetical Data)
    "$GPGGA,195640.490,4347.173,N,07913.613,W,1,12,1.0,0.0,M,0.0,M,,*77\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195640.490,A,4347.173,N,07913.613,W,011.7,325.4,201223,000.0,W*64\r\n",
    "$GPGGA,195641.490,4347.176,N,07913.615,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195641.490,A,4347.176,N,07913.615,W,011.7,325.4,201223,000.0,W*66\r\n",
    "$GPGGA,195642.490,4347.179,N,07913.617,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195642.490,A,4347.179,N,07913.617,W,011.7,325.4,201223,000.0,W*68\r\n",
    "$GPGGA,195643.490,4347.182,N,07913.619,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195643.490,A,4347.182,N,07913.619,W,011.7,041.9,201223,000.0,W*6F\r\n",
    "$GPGGA,195644.490,4347.185,N,07913.616,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195644.490,A,4347.185,N,07913.616,W,011.7,070.9,201223,000.0,W*62\r\n",
    "$GPGGA,195645.490,4347.186,N,07913.612,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195645.490,A,4347.186,N,07913.612,W,011.7,070.9,201223,000.0,W*64\r\n",
    "$GPGGA,195646.490,4347.188,N,07913.608,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195646.490,A,4347.188,N,07913.608,W,011.7,070.9,201223,000.0,W*62\r\n",
    "$GPGGA,195647.490,4347.189,N,07913.604,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195647.490,A,4347.189,N,07913.604,W,011.7,070.9,201223,000.0,W*6E\r\n",
    "$GPGGA,195648.490,4347.190,N,07913.600,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195648.490,A,4347.190,N,07913.600,W,011.7,070.9,201223,000.0,W*6D\r\n",
    "$GPGGA,195649.490,4347.192,N,07913.596,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195649.490,A,4347.192,N,07913.596,W,011.7,070.9,201223,000.0,W*62\r\n",
    "$GPGGA,195650.490,4347.193,N,07913.592,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195650.490,A,4347.193,N,07913.592,W,011.7,070.9,201223,000.0,W*6F\r\n",
    "$GPGGA,195651.490,4347.195,N,07913.588,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195651.490,A,4347.195,N,07913.588,W,011.7,070.9,201223,000.0,W*63\r\n",
    "$GPGGA,195652.490,4347.196,N,07913.584,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195652.490,A,4347.196,N,07913.584,W,011.7,070.9,201223,000.0,W*6F\r\n",
    "$GPGGA,195653.490,4347.197,N,07913.580,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195653.490,A,4347.197,N,07913.580,W,011.7,070.9,201223,000.0,W*6B\r\n",
    "$GPGGA,195654.490,4347.199,N,07913.576,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195654.490,A,4347.199,N,07913.576,W,011.7,070.9,201223,000.0,W*6B\r\n",
    "$GPGGA,195655.490,4347.200,N,07913.572,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195655.490,A,4347.200,N,07913.572,W,011.7,070.9,201223,000.0,W*6D\r\n",
    "$GPGGA,195656.490,4347.202,N,07913.568,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195656.490,A,4347.202,N,07913.568,W,011.7,070.9,201223,000.0,W*67\r\n",
    "$GPGGA,195657.490,4347.203,N,07913.564,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195657.490,A,4347.203,N,07913.564,W,011.7,015.3,201223,000.0,W*62\r\n",
    "$GPGGA,195658.490,4347.206,N,07913.563,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195658.490,A,4347.206,N,07913.563,W,011.7,320.4,201223,000.0,W*6D\r\n",
    "$GPGGA,195659.490,4347.209,N,07913.565,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195659.490,A,4347.209,N,07913.565,W,011.7,320.4,201223,000.0,W*65\r\n",
    "$GPGGA,195700.490,4347.212,N,07913.567,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195700.490,A,4347.212,N,07913.567,W,011.7,289.6,201223,000.0,W*60\r\n",
    "$GPGGA,195701.490,4347.213,N,07913.571,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195701.490,A,4347.213,N,07913.571,W,011.7,324.5,201223,000.0,W*62\r\n",
    "$GPGGA,195702.490,4347.216,N,07913.573,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195702.490,A,4347.216,N,07913.573,W,011.7,324.5,201223,000.0,W*66\r\n",
    "$GPGGA,195703.490,4347.219,N,07913.575,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195703.490,A,4347.219,N,07913.575,W,011.7,324.5,201223,000.0,W*6E\r\n",
    "$GPGGA,195704.490,4347.222,N,07913.577,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195704.490,A,4347.222,N,07913.577,W,011.7,266.5,201223,000.0,W*64\r\n",
    "$GPGGA,195705.490,4347.221,N,07913.582,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195705.490,A,4347.221,N,07913.582,W,011.7,315.3,201223,000.0,W*6F\r\n",
    "$GPGGA,195706.490,4347.224,N,07913.585,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195706.490,A,4347.224,N,07913.585,W,011.7,315.3,201223,000.0,W*6E\r\n",
    "$GPGGA,195707.490,4347.227,N,07913.587,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195707.490,A,4347.227,N,07913.587,W,011.7,315.3,201223,000.0,W*6E\r\n",
    "$GPGGA,195708.490,4347.229,N,07913.590,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195708.490,A,4347.229,N,07913.590,W,011.7,324.9,201223,000.0,W*61\r\n",
    "$GPGGA,195709.490,4347.232,N,07913.592,W,1,12,1.0,0.0,M,0.0,M,,*77\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195709.490,A,4347.232,N,07913.592,W,011.7,324.9,201223,000.0,W*68\r\n",
    "$GPGGA,195710.490,4347.235,N,07913.594,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195710.490,A,4347.235,N,07913.594,W,011.7,324.9,201223,000.0,W*61\r\n",
    "$GPGGA,195711.490,4347.238,N,07913.596,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195711.490,A,4347.238,N,07913.596,W,011.7,324.9,201223,000.0,W*6F\r\n",
    "$GPGGA,195712.490,4347.241,N,07913.598,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195712.490,A,4347.241,N,07913.598,W,011.7,324.9,201223,000.0,W*6C\r\n",
    "$GPGGA,195713.490,4347.244,N,07913.600,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195713.490,A,4347.244,N,07913.600,W,011.7,324.9,201223,000.0,W*6A\r\n",
    "$GPGGA,195714.490,4347.247,N,07913.602,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195714.490,A,4347.247,N,07913.602,W,011.7,324.9,201223,000.0,W*6C\r\n",
    "$GPGGA,195715.490,4347.250,N,07913.604,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195715.490,A,4347.250,N,07913.604,W,011.7,324.9,201223,000.0,W*6D\r\n",
    "$GPGGA,195716.490,4347.252,N,07913.606,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195716.490,A,4347.252,N,07913.606,W,011.7,324.9,201223,000.0,W*6E\r\n",
    "$GPGGA,195717.490,4347.255,N,07913.608,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195717.490,A,4347.255,N,07913.608,W,011.7,324.9,201223,000.0,W*66\r\n",
    "$GPGGA,195718.490,4347.258,N,07913.610,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195718.490,A,4347.258,N,07913.610,W,011.7,252.0,201223,000.0,W*64\r\n",
    "$GPGGA,195719.490,4347.257,N,07913.614,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195719.490,A,4347.257,N,07913.614,W,011.7,252.0,201223,000.0,W*6E\r\n",
    "$GPGGA,195720.490,4347.256,N,07913.618,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195720.490,A,4347.256,N,07913.618,W,011.7,252.0,201223,000.0,W*69\r\n",
    "$GPGGA,195721.490,4347.254,N,07913.622,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195721.490,A,4347.254,N,07913.622,W,011.7,252.0,201223,000.0,W*63\r\n",
    "$GPGGA,195722.490,4347.253,N,07913.626,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195722.490,A,4347.253,N,07913.626,W,011.7,252.0,201223,000.0,W*63\r\n",
    "$GPGGA,195723.490,4347.252,N,07913.631,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195723.490,A,4347.252,N,07913.631,W,015.6,248.1,201223,000.0,W*6A\r\n",
    "$GPGGA,195724.490,4347.249,N,07913.636,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195724.490,A,4347.249,N,07913.636,W,015.6,248.1,201223,000.0,W*60\r\n",
    "$GPGGA,195725.490,4347.247,N,07913.641,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195725.490,A,4347.247,N,07913.641,W,015.6,248.1,201223,000.0,W*6F\r\n",
    "$GPGGA,195726.490,4347.245,N,07913.646,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195726.490,A,4347.245,N,07913.646,W,015.6,248.1,201223,000.0,W*69\r\n",
    "$GPGGA,195727.490,4347.243,N,07913.651,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195727.490,A,4347.243,N,07913.651,W,015.6,248.1,201223,000.0,W*68\r\n",
    "$GPGGA,195728.490,4347.241,N,07913.657,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195728.490,A,4347.241,N,07913.657,W,015.6,248.1,201223,000.0,W*63\r\n",
    "$GPGGA,195729.490,4347.239,N,07913.662,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195729.490,A,4347.239,N,07913.662,W,015.6,248.1,201223,000.0,W*6B\r\n",
    "$GPGGA,195730.490,4347.237,N,07913.667,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195730.490,A,4347.237,N,07913.667,W,023.3,248.5,201223,000.0,W*6C\r\n",
    "$GPGGA,195731.490,4347.234,N,07913.675,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195731.490,A,4347.234,N,07913.675,W,023.3,248.5,201223,000.0,W*6D\r\n",
    "$GPGGA,195732.490,4347.231,N,07913.683,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195732.490,A,4347.231,N,07913.683,W,023.3,248.5,201223,000.0,W*62\r\n",
    "$GPGGA,195733.490,4347.228,N,07913.691,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195733.490,A,4347.228,N,07913.691,W,023.3,248.5,201223,000.0,W*68\r\n",
    "$GPGGA,195734.490,4347.225,N,07913.699,W,1,12,1.0,0.0,M,0.0,M,,*77\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195734.490,A,4347.225,N,07913.699,W,023.3,248.5,201223,000.0,W*6A\r\n",
    "$GPGGA,195735.490,4347.221,N,07913.707,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195735.490,A,4347.221,N,07913.707,W,023.3,248.5,201223,000.0,W*69\r\n",
    "$GPGGA,195736.490,4347.218,N,07913.714,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195736.490,A,4347.218,N,07913.714,W,023.3,248.5,201223,000.0,W*62\r\n",
    "$GPGGA,195737.490,4347.215,N,07913.722,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195737.490,A,4347.215,N,07913.722,W,023.3,248.5,201223,000.0,W*6B\r\n",
    "$GPGGA,195738.490,4347.212,N,07913.730,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195738.490,A,4347.212,N,07913.730,W,023.3,248.5,201223,000.0,W*60\r\n",
    "$GPGGA,195739.490,4347.209,N,07913.738,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195739.490,A,4347.209,N,07913.738,W,023.3,248.5,201223,000.0,W*63\r\n",
    "$GPGGA,195740.490,4347.206,N,07913.746,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195740.490,A,4347.206,N,07913.746,W,023.3,248.5,201223,000.0,W*6B\r\n",
    "$GPGGA,195741.490,4347.203,N,07913.754,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195741.490,A,4347.203,N,07913.754,W,023.3,248.5,201223,000.0,W*6C\r\n",
    "$GPGGA,195742.490,4347.200,N,07913.762,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195742.490,A,4347.200,N,07913.762,W,023.3,248.5,201223,000.0,W*69\r\n",
    "$GPGGA,195743.490,4347.197,N,07913.770,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195743.490,A,4347.197,N,07913.770,W,023.3,248.5,201223,000.0,W*66\r\n",
    "$GPGGA,195744.490,4347.194,N,07913.777,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195744.490,A,4347.194,N,07913.777,W,023.3,248.5,201223,000.0,W*65\r\n",
    "$GPGGA,195745.490,4347.190,N,07913.785,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195745.490,A,4347.190,N,07913.785,W,023.3,248.5,201223,000.0,W*6D\r\n",
    "$GPGGA,195746.490,4347.187,N,07913.793,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195746.490,A,4347.187,N,07913.793,W,019.4,244.9,201223,000.0,W*61\r\n",
    "$GPGGA,195747.490,4347.184,N,07913.799,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195747.490,A,4347.184,N,07913.799,W,019.4,244.9,201223,000.0,W*69\r\n",
    "$GPGGA,195748.490,4347.181,N,07913.806,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195748.490,A,4347.181,N,07913.806,W,019.4,244.9,201223,000.0,W*6A\r\n",
    "$GPGGA,195749.490,4347.179,N,07913.812,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195749.490,A,4347.179,N,07913.812,W,019.4,244.9,201223,000.0,W*69\r\n",
    "$GPGGA,195750.490,4347.176,N,07913.818,W,1,12,1.0,0.0,M,0.0,M,,*77\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195750.490,A,4347.176,N,07913.818,W,019.4,244.9,201223,000.0,W*64\r\n",
    "$GPGGA,195751.490,4347.173,N,07913.825,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195751.490,A,4347.173,N,07913.825,W,019.4,244.9,201223,000.0,W*6E\r\n",
    "$GPGGA,195752.490,4347.170,N,07913.831,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195752.490,A,4347.170,N,07913.831,W,019.4,244.9,201223,000.0,W*6B\r\n",
    "$GPGGA,195753.490,4347.167,N,07913.837,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195753.490,A,4347.167,N,07913.837,W,015.6,246.1,201223,000.0,W*6E\r\n",
    "$GPGGA,195754.490,4347.165,N,07913.842,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195754.490,A,4347.165,N,07913.842,W,015.6,246.1,201223,000.0,W*69\r\n",
    "$GPGGA,195755.490,4347.162,N,07913.847,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195755.490,A,4347.162,N,07913.847,W,015.6,246.1,201223,000.0,W*6A\r\n",
    "$GPGGA,195756.490,4347.160,N,07913.852,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195756.490,A,4347.160,N,07913.852,W,015.6,246.1,201223,000.0,W*6F\r\n",
    "$GPGGA,195757.490,4347.158,N,07913.858,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195757.490,A,4347.158,N,07913.858,W,015.6,246.1,201223,000.0,W*6F\r\n",
    "$GPGGA,195758.490,4347.156,N,07913.863,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195758.490,A,4347.156,N,07913.863,W,015.6,246.1,201223,000.0,W*66\r\n",
    "$GPGGA,195759.490,4347.153,N,07913.868,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195759.490,A,4347.153,N,07913.868,W,015.6,246.1,201223,000.0,W*69\r\n",
    "$GPGGA,195800.490,4347.151,N,07913.873,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195800.490,A,4347.151,N,07913.873,W,011.7,249.8,201223,000.0,W*61\r\n",
    "$GPGGA,195801.490,4347.150,N,07913.877,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195801.490,A,4347.150,N,07913.877,W,011.7,249.8,201223,000.0,W*65\r\n",
    "$GPGGA,195802.490,4347.148,N,07913.881,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195802.490,A,4347.148,N,07913.881,W,011.7,249.8,201223,000.0,W*66\r\n",
    "$GPGGA,195803.490,4347.147,N,07913.885,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195803.490,A,4347.147,N,07913.885,W,011.7,249.8,201223,000.0,W*6C\r\n",
    "$GPGGA,195804.490,4347.145,N,07913.889,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195804.490,A,4347.145,N,07913.889,W,011.7,219.6,201223,000.0,W*6E\r\n",
    "$GPGGA,195805.490,4347.142,N,07913.891,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195805.490,A,4347.142,N,07913.891,W,011.7,219.6,201223,000.0,W*61\r\n",
    "$GPGGA,195806.490,4347.140,N,07913.893,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195806.490,A,4347.140,N,07913.893,W,011.7,219.6,201223,000.0,W*62\r\n",
    "$GPGGA,195807.490,4347.137,N,07913.896,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195807.490,A,4347.137,N,07913.896,W,011.7,217.6,201223,000.0,W*68\r\n",
    "$GPGGA,195808.490,4347.134,N,07913.898,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195808.490,A,4347.134,N,07913.898,W,011.7,217.6,201223,000.0,W*6A\r\n",
    "$GPGGA,195809.490,4347.131,N,07913.900,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195809.490,A,4347.131,N,07913.900,W,011.7,169.5,201223,000.0,W*67\r\n",
    "$GPGGA,195810.490,4347.128,N,07913.899,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195810.490,A,4347.128,N,07913.899,W,011.7,169.5,201223,000.0,W*66\r\n",
    "$GPGGA,195811.490,4347.125,N,07913.899,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195811.490,A,4347.125,N,07913.899,W,015.6,144.9,201223,000.0,W*6C\r\n",
    "$GPGGA,195812.490,4347.121,N,07913.896,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195812.490,A,4347.121,N,07913.896,W,015.6,144.9,201223,000.0,W*64\r\n",
    "$GPGGA,195813.490,4347.117,N,07913.893,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195813.490,A,4347.117,N,07913.893,W,015.6,144.9,201223,000.0,W*65\r\n",
    "$GPGGA,195814.490,4347.113,N,07913.891,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195814.490,A,4347.113,N,07913.891,W,015.6,144.9,201223,000.0,W*64\r\n",
    "$GPGGA,195815.490,4347.109,N,07913.888,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195815.490,A,4347.109,N,07913.888,W,015.6,144.9,201223,000.0,W*66\r\n",
    "$GPGGA,195816.490,4347.106,N,07913.885,W,1,12,1.0,0.0,M,0.0,M,,*79\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195816.490,A,4347.106,N,07913.885,W,015.6,144.9,201223,000.0,W*67\r\n",
    "$GPGGA,195817.490,4347.102,N,07913.883,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195817.490,A,4347.102,N,07913.883,W,015.6,144.9,201223,000.0,W*64\r\n",
    "$GPGGA,195818.490,4347.098,N,07913.880,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195818.490,A,4347.098,N,07913.880,W,015.6,144.9,201223,000.0,W*6A\r\n",
    "$GPGGA,195819.490,4347.094,N,07913.877,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195819.490,A,4347.094,N,07913.877,W,011.7,151.4,201223,000.0,W*63\r\n",
    "$GPGGA,195820.490,4347.091,N,07913.876,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195820.490,A,4347.091,N,07913.876,W,011.7,151.4,201223,000.0,W*6D\r\n",
    "$GPGGA,195821.490,4347.088,N,07913.874,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195821.490,A,4347.088,N,07913.874,W,011.7,151.4,201223,000.0,W*66\r\n",
    "$GPGGA,195822.490,4347.085,N,07913.872,W,1,12,1.0,0.0,M,0.0,M,,*7C\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195822.490,A,4347.085,N,07913.872,W,011.7,069.5,201223,000.0,W*65\r\n",
    "$GPGGA,195823.490,4347.086,N,07913.868,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195823.490,A,4347.086,N,07913.868,W,011.7,069.5,201223,000.0,W*6C\r\n",
    "$GPGGA,195824.490,4347.088,N,07913.864,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195824.490,A,4347.088,N,07913.864,W,011.7,069.5,201223,000.0,W*69\r\n",
    "$GPGGA,195825.490,4347.089,N,07913.860,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195825.490,A,4347.089,N,07913.860,W,011.7,069.5,201223,000.0,W*6D\r\n",
    "$GPGGA,195826.490,4347.091,N,07913.856,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195826.490,A,4347.091,N,07913.856,W,015.6,068.8,201223,000.0,W*6B\r\n",
    "$GPGGA,195827.490,4347.093,N,07913.851,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195827.490,A,4347.093,N,07913.851,W,015.6,068.8,201223,000.0,W*6F\r\n",
    "$GPGGA,195828.490,4347.095,N,07913.846,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195828.490,A,4347.095,N,07913.846,W,015.6,068.8,201223,000.0,W*60\r\n",
    "$GPGGA,195829.490,4347.097,N,07913.841,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195829.490,A,4347.097,N,07913.841,W,015.6,068.8,201223,000.0,W*64\r\n",
    "$GPGGA,195830.490,4347.099,N,07913.835,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195830.490,A,4347.099,N,07913.835,W,015.6,068.8,201223,000.0,W*61\r\n",
    "$GPGGA,195831.490,4347.101,N,07913.830,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195831.490,A,4347.101,N,07913.830,W,019.4,069.5,201223,000.0,W*67\r\n",
    "$GPGGA,195832.490,4347.104,N,07913.823,W,1,12,1.0,0.0,M,0.0,M,,*71\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195832.490,A,4347.104,N,07913.823,W,019.4,069.5,201223,000.0,W*63\r\n",
    "$GPGGA,195833.490,4347.106,N,07913.817,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195833.490,A,4347.106,N,07913.817,W,019.4,069.5,201223,000.0,W*67\r\n",
    "$GPGGA,195834.490,4347.109,N,07913.810,W,1,12,1.0,0.0,M,0.0,M,,*7A\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195834.490,A,4347.109,N,07913.810,W,019.4,069.5,201223,000.0,W*68\r\n",
    "$GPGGA,195835.490,4347.111,N,07913.803,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195835.490,A,4347.111,N,07913.803,W,019.4,069.5,201223,000.0,W*62\r\n",
    "$GPGGA,195836.490,4347.113,N,07913.797,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195836.490,A,4347.113,N,07913.797,W,019.4,069.5,201223,000.0,W*61\r\n",
    "$GPGGA,195837.490,4347.116,N,07913.790,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195837.490,A,4347.116,N,07913.790,W,019.4,069.5,201223,000.0,W*62\r\n",
    "$GPGGA,195838.490,4347.118,N,07913.783,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195838.490,A,4347.118,N,07913.783,W,019.4,069.5,201223,000.0,W*61\r\n",
    "$GPGGA,195839.490,4347.121,N,07913.777,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195839.490,A,4347.121,N,07913.777,W,019.4,069.5,201223,000.0,W*61\r\n",
    "$GPGGA,195840.490,4347.123,N,07913.770,W,1,12,1.0,0.0,M,0.0,M,,*78\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195840.490,A,4347.123,N,07913.770,W,023.3,069.4,201223,000.0,W*65\r\n",
    "$GPGGA,195841.490,4347.126,N,07913.762,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195841.490,A,4347.126,N,07913.762,W,023.3,069.4,201223,000.0,W*62\r\n",
    "$GPGGA,195842.490,4347.129,N,07913.754,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195842.490,A,4347.129,N,07913.754,W,023.3,069.4,201223,000.0,W*6B\r\n",
    "$GPGGA,195843.490,4347.132,N,07913.746,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195843.490,A,4347.132,N,07913.746,W,023.3,069.4,201223,000.0,W*63\r\n",
    "$GPGGA,195844.490,4347.135,N,07913.738,W,1,12,1.0,0.0,M,0.0,M,,*77\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195844.490,A,4347.135,N,07913.738,W,023.3,069.4,201223,000.0,W*6A\r\n",
    "$GPGGA,195845.490,4347.138,N,07913.730,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195845.490,A,4347.138,N,07913.730,W,023.3,069.4,201223,000.0,W*6E\r\n",
    "$GPGGA,195846.490,4347.141,N,07913.722,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195846.490,A,4347.141,N,07913.722,W,023.3,069.4,201223,000.0,W*60\r\n",
    "$GPGGA,195847.490,4347.144,N,07913.715,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195847.490,A,4347.144,N,07913.715,W,023.3,069.4,201223,000.0,W*60\r\n",
    "$GPGGA,195848.490,4347.147,N,07913.707,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195848.490,A,4347.147,N,07913.707,W,023.3,069.4,201223,000.0,W*6F\r\n",
    "$GPGGA,195849.490,4347.150,N,07913.699,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195849.490,A,4347.150,N,07913.699,W,023.3,069.4,201223,000.0,W*6E\r\n",
    "$GPGGA,195850.490,4347.153,N,07913.691,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195850.490,A,4347.153,N,07913.691,W,019.4,061.3,201223,000.0,W*6C\r\n",
    "$GPGGA,195851.490,4347.157,N,07913.685,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195851.490,A,4347.157,N,07913.685,W,019.4,061.3,201223,000.0,W*6C\r\n",
    "$GPGGA,195852.490,4347.160,N,07913.679,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195852.490,A,4347.160,N,07913.679,W,019.4,061.3,201223,000.0,W*68\r\n",
    "$GPGGA,195853.490,4347.163,N,07913.673,W,1,12,1.0,0.0,M,0.0,M,,*7C\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195853.490,A,4347.163,N,07913.673,W,015.6,056.0,201223,000.0,W*69\r\n",
    "$GPGGA,195854.490,4347.166,N,07913.668,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195854.490,A,4347.166,N,07913.668,W,015.6,056.0,201223,000.0,W*61\r\n",
    "$GPGGA,195855.490,4347.169,N,07913.664,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195855.490,A,4347.169,N,07913.664,W,015.6,056.0,201223,000.0,W*63\r\n",
    "$GPGGA,195856.490,4347.172,N,07913.660,W,1,12,1.0,0.0,M,0.0,M,,*7B\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195856.490,A,4347.172,N,07913.660,W,011.7,071.3,201223,000.0,W*6D\r\n",
    "$GPGGA,195857.490,4347.173,N,07913.656,W,1,12,1.0,0.0,M,0.0,M,,*7E\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195857.490,A,4347.173,N,07913.656,W,011.7,071.3,201223,000.0,W*68\r\n",
    "$GPGGA,195858.490,4347.175,N,07913.652,W,1,12,1.0,0.0,M,0.0,M,,*73\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195858.490,A,4347.175,N,07913.652,W,011.7,071.3,201223,000.0,W*65\r\n",
    "$GPGGA,195859.490,4347.176,N,07913.647,W,1,12,1.0,0.0,M,0.0,M,,*75\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195859.490,A,4347.176,N,07913.647,W,011.7,071.3,201223,000.0,W*63\r\n",
    "$GPGGA,195900.490,4347.177,N,07913.643,W,1,12,1.0,0.0,M,0.0,M,,*7D\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195900.490,A,4347.177,N,07913.643,W,011.7,071.3,201223,000.0,W*6B\r\n",
    "$GPGGA,195901.490,4347.179,N,07913.639,W,1,12,1.0,0.0,M,0.0,M,,*7F\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195901.490,A,4347.179,N,07913.639,W,011.7,071.3,201223,000.0,W*69\r\n",
    "$GPGGA,195902.490,4347.180,N,07913.635,W,1,12,1.0,0.0,M,0.0,M,,*76\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195902.490,A,4347.180,N,07913.635,W,011.7,071.3,201223,000.0,W*60\r\n",
    "$GPGGA,195903.490,4347.181,N,07913.631,W,1,12,1.0,0.0,M,0.0,M,,*72\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195903.490,A,4347.181,N,07913.631,W,009.7,106.3,201223,000.0,W*6C\r\n",
    "$GPGGA,195904.490,4347.180,N,07913.628,W,1,12,1.0,0.0,M,0.0,M,,*7C\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195904.490,A,4347.180,N,07913.628,W,009.7,106.3,201223,000.0,W*62\r\n",
    "$GPGGA,195905.490,4347.179,N,07913.624,W,1,12,1.0,0.0,M,0.0,M,,*77\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195905.490,A,4347.179,N,07913.624,W,009.7,106.3,201223,000.0,W*69\r\n",
    "$GPGGA,195906.490,4347.178,N,07913.621,W,1,12,1.0,0.0,M,0.0,M,,*70\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195906.490,A,4347.178,N,07913.621,W,009.7,119.8,201223,000.0,W*6B\r\n",
    "$GPGGA,195907.490,4347.177,N,07913.618,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195907.490,A,4347.177,N,07913.618,W,009.7,119.8,201223,000.0,W*6F\r\n",
    "$GPGGA,195908.490,4347.175,N,07913.615,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195908.490,A,4347.175,N,07913.615,W,009.7,119.8,201223,000.0,W*6F\r\n",
    "$GPGGA,195909.490,4347.173,N,07913.612,W,1,12,1.0,0.0,M,0.0,M,,*74\r\n",
    "$GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.0,1.0,1.0*30\r\n",
    "$GPRMC,195909.490,A,4347.173,N,07913.612,W,009.7,119.8,201223,000.0,W*6F\r\n",
  };


  typedef struct {
    uart_dev_t uart0;
    uint32_t   gps_tx_index;
  } chip_state_t;


  static void chip_timer_event  (void *user_data);


  void chip_init(void) {

    setvbuf(stdout, NULL, _IOLBF, 1024);                // ???     // Disable buffering for debug logs

    chip_state_t *chip = malloc(sizeof(chip_state_t));

    const uart_config_t uart_config = {
      .tx         = pin_init("TX", INPUT_PULLUP),
      .rx         = pin_init("RX", INPUT),
      .baud_rate  = 9600,
      .user_data  = chip,
    };

    chip->uart0        = uart_init(&uart_config);
  
    chip->gps_tx_index = 0;

    const timer_config_t timer_config = {
      .callback  = chip_timer_event,
      .user_data = chip,
    };

    timer_t timer = timer_init(&timer_config);

    timer_start(timer, SECOND, true);

    printf("GPS Chip initialized!\n");         // prints to web browser console (developer tools)

  }

  void chip_timer_event(void *user_data) {

    chip_state_t *chip = (chip_state_t*) user_data;

    printf("chip_timer_event\n");
//  printf ("LEN(gps_tx_data):  %d\n", LEN(gps_tx_data)   );    // number of messages
//  printf ("message length  :  %d\n", strlen(message)    );    // actual message length
//  printf ("gps_tx_index    :  %u\n", chip->gps_tx_index );    // message index

    const char * message = gps_tx_data[chip->gps_tx_index++];

    uart_write(chip->uart0, (uint8_t *) message, strlen(message));

    if (chip->gps_tx_index >= LEN(gps_tx_data)) {       
      chip->gps_tx_index = 0;
    }
  }

