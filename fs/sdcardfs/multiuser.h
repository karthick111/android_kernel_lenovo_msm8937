/*
 * fs/sdcardfs/multiuser.h
 *
 * Copyright (c) 2015 Lenovo Co. Ltd
 *   Authors: liaohs , jixj
 *
 * This program has been developed as a stackable file system based on
 * the WrapFS which written by 
 *
 * Copyright (c) 1998-2014 Erez Zadok
 * Copyright (c) 2009	   Shrikar Archak
 * Copyright (c) 2003-2014 Stony Brook University
 * Copyright (c) 2003-2014 The Research Foundation of SUNY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define MULTIUSER_APP_PER_USER_RANGE 100000

typedef uid_t userid_t;
typedef long appid_t;

static inline userid_t multiuser_get_user_id(uid_t uid) {
    return uid / MULTIUSER_APP_PER_USER_RANGE;
}       
        
static inline appid_t multiuser_get_app_id(uid_t uid) {
    return uid % MULTIUSER_APP_PER_USER_RANGE;
}       
    
static inline uid_t multiuser_get_uid(userid_t userId, appid_t appId) {
    return userId * MULTIUSER_APP_PER_USER_RANGE + (appId % MULTIUSER_APP_PER_USER_RANGE);
}

