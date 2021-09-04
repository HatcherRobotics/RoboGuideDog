/*
* rle_file.h
* Definition of RLE file headers and constants
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-5
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "../types.h"

namespace rpos { namespace system { namespace encoding {

#if defined(_WIN32) || defined(__ICCARM__)
#pragma pack(push)
#pragma pack(1)
#endif

    struct RleFileHeader {
        types::_u8 signature[3];
        types::_u8 sentinel1;
        types::_u8 sentinel2;
        types::_u32 decompressedSize;
    } __attribute__((packed));

#if defined(_WIN32) || defined(__ICCARM__)
#pragma pack(pop)
#endif
    
    static const types::_u8 RleSignature[] = { 'R', 'L', 'E' };

} } }
