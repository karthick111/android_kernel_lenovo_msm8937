
/*
 * Copyright (c) 2015, The lenovo mobile Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

struct Board_Info
{
  int  index;
  int hwid_gpio1;   //gpio 116
  int hwid_gpio2;   //gpio 127
  int hwid_adc;    //pmic8937 mpp_02
  char boardname[32];
  char phname[32];
};

enum
{
   HW_ID_START=1,
   S82937AA1=HW_ID_START,
   S82937BA1,
   S82937CA1,
   S82937DA1,
   S82937EA1,
   S82937FA1,
   S82937GA1,
   S82938AA1,
   S82937HA1,
   S82938BA1,
   S82937KA1,
   S82939AA1,
   S82939BA1,
   S82939CA1,
   S82939DA1,
   S82939EA1,
   S82939FA1,
   S82939GA1,
   HW_ID_END
};


static struct Board_Info board_info_arry[]=

{/*--index---hwid_gpio1---hwid_gpio2---hwid_adc---boardname---phname---*/
   {S82937AA1, 0, 1, 200,  "a48_dual_sim",        "S82937AA1"},
   {S82937BA1, 0, 1, 480,  "a48_dual_sim_pro",    "S82937BA1"},
   {S82937CA1, 0, 1, 758,  "a48_signal_sim",      "S82937CA1"},
   {S82937DA1, 0, 1, 1038, "b36_Las_dual_sim",    "S82937DA1"},
   {S82937EA1, 0, 1, 1317, "a37_Las_signal_sim",  "S82937EA1"},
   {S82937FA1, 0, 1, 1591, "c70",                 "S82937FA1"},
   {S82937GA1, 0, 0, 200,  "c70_pro",             "S82937GA1"},
   {S82938AA1, 0, 0, 480,  "karate_power_low",    "S82938AA1"},
   {S82937HA1, 0, 0, 758,  "b36_Brazil_dual_sim", "S82937HA1"},
   {S82938BA1, 0, 0, 1038, "karate_power_high",   "S82938BA1"},
   {S82937KA1, 0, 0, 1317, "reserve",             "S82937KA1"},
   {S82939AA1, 1, 1, 200,  "a40_dual_sim",        "S82939AA1"},
   {S82939BA1, 1, 1, 480,  "a40_dual_sim_pro",    "S82939BA1"},
   {S82939CA1, 1, 1, 758,  "a41_signal_sim",      "S82939CA1"},
   {S82939DA1, 1, 1, 1038, "c78_dual_sim",        "S82939DA1"},
   {S82939EA1, 1, 1, 1317, "c78_dual_sim_pro",    "S82939EA1"},
   {S82939FA1, 1, 1, 1591, "b36_dual_sim",        "S82939FA1"},
   {S82939GA1, 1, 0, 200,  "b37_signal_sim",      "S82939GA1"},
};
