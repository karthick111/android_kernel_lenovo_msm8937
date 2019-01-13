/*
 * TS3A227E Autonous Audio Accessory Detection and Configureation Switch
 *
 * Copyright (C) 2014 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _TS3A227E_H
#define _TS3A227E_H

#define TS3A227E_SWITCH_AUTO          0
#define TS3A227E_SWITCH_3POLE         1
#define TS3A227E_SWITCH_4POLE_STD     2
#define TS3A227E_SWITCH_4POLE_OMTP    3
#define TS3A227E_SWITCH_UKNOWN        4

int ts3a227e_switch(int  mode);
bool ts3a227e_next_switch(void);

int karate_hp_switch_set(bool enable, int user);
bool karate_hp_switch_get(void);

#endif
