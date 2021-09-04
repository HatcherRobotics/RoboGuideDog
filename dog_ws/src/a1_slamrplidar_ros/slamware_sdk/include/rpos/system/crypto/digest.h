/*
* digest.h
* Digest algorithms
*
* Created by Tony Huang (tony@slamtec.com) at 2017-3-13
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/core/rpos_core_config.h>
#include <cstdint>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace rpos { namespace system { namespace crypto {

    class RPOS_CORE_API IDigest {
    public:
        virtual ~IDigest() {}
        virtual size_t digestSize() const = 0;
        virtual void reset() = 0;
        virtual void update(const void* buffer, size_t size) = 0;
        virtual void getDigest(std::uint8_t* buffer) = 0;
    };

    enum DigestAlgorithm {
        DigestAlgorithmMD5,
        DigestAlgorithmSHA256
    };

    RPOS_CORE_API boost::shared_ptr<IDigest> createDigest(DigestAlgorithm algorithm);
    RPOS_CORE_API std::vector<std::uint8_t> digest(DigestAlgorithm algorithm, const void* buffer, size_t size);
    RPOS_CORE_API std::string digestToHex(DigestAlgorithm algorithm, const void* buffer, size_t size);

} } }
