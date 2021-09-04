/*
* aes_encryptor.h
* Encrypt stream with AES algorithm
*
* Created by Tony Huang (tony@slamtec.com) at 2018-10-7
* Copyright 2018 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_encoder.h"
#include <openssl/aes.h>
#include <cstdint>
#include <vector>

namespace rpos { namespace system { namespace encoding {

    /**
    * Default AES Encryption Block Size (16 bytes)
    */
    static const size_t kAesEncryptorDefaultChunkSize = 16u;

    /**
    * AES Encryption Mode
    */
    enum AesMode {
        /// AES-ECB Mode, each block is encrypted individually
        AesModeEcb = 1,

        /// AES-CBC Mode, each block is encrypted with a ivec and key, and generate new ivec for next block (default)
        AesModeCbc = 2,

        /// Using encryptor as a encryptor (default)
        AesModeEncrypt = 0x10,

        /// Using encryptor as a decryptor
        AesModeDecrypt = 0x20,

        AesModeDefault = AesModeCbc | AesModeEncrypt
    };
    
    /**
    * AES Encryptor
    */
    class RPOS_CORE_API AesEncryptor : public IEncoder {
    public:
        typedef std::uint8_t byte_t;

    public:
        /**
        * Constructor
        *
        * @param key       The AES encryption key
        * @param nbytes    The size of the key (in bytes)
        * @param mode      The encryptor work mode (default: encrypt + ecb)
        * @param chunkSize Chunk size
        */
        AesEncryptor(const byte_t* key, size_t nbytes, AesMode mode = AesModeDefault, size_t chunkSize = kAesEncryptorDefaultChunkSize);

        /**
        * Constructor
        *
        * @param key     The AES encryption key
        * @param mode    The encryptor work mode (default: encrypt + ecb)
        * @param chunkSize Chunk size
        */
        AesEncryptor(const std::vector<byte_t>& key, AesMode mode = AesModeDefault, size_t chunkSize = kAesEncryptorDefaultChunkSize);

        /**
        * Constructor
        *
        * @param key     The AES encryption key
        * @param mode    The encryptor work mode (default: encrypt + ecb)
        * @param chunkSize Chunk size
        */
        AesEncryptor(const AES_KEY* key, AesMode mode = AesModeDefault, size_t chunkSize = kAesEncryptorDefaultChunkSize);

        /**
        * Destructor
        */
        virtual ~AesEncryptor();

    public:
        /**
        * Set encryption key
        *
        * @param key       The AES encryption key
        * @param nbytes    The size of the key (in bytes)
        */
        void setKey(const byte_t* key, size_t nbytes);

        /**
        * Set encryption key
        *
        * @param key       The AES encryption key
        */
        void setKey(const std::vector<byte_t>& key);

        /**
        * Set encryption key
        *
        * @param key       The AES encryption key
        */
        void setKey(const AES_KEY* key);

        /**
        * Reset encryptor status
        */
        void reset();

    public:
        /**
        * Encode data from source buffer to dest buffer
        *
        * @param srcBuffer        The source buffer
        * @param srcSize          The size of source buffer
        * @param destBuffer       The buffer to store encoded data
        * @param destSize         The size of dest buffer
        * @param dataPosition     Indicate the position of data in the source stream (for encoders like base64, we need this to decide use one or two pads)
        * @param consumedSrcSize  Bytes encoded in the source stream
        * @param consumedDestSize Bytes written to the dest stream
        * @return The encoding is successful or not
        */
        virtual bool encode(
            const void* srcBuffer, size_t srcSize,
            void* destBuffer, size_t destSize,
            StreamPosition dataPosition,
            size_t& comsumedSrcSize, size_t& consumedDestSize
        );

        /**
        * Estimate encoded size
        *
        * @param srcBuffer  The source buffer
        * @param srcSize    The size of source
        *
        * Estimated size of encoded data (this should be equal to or greater than the actual encoded size)
        */
        virtual size_t estimateEncodedSize(const void* srcBuffer, size_t srcSize);

        /**
        * Estimate encoded size
        *
        * @param srcSize    The size of source
        *
        * Estimated size of encoded data (this should be equal to or greater than the actual encoded size)
        */
        virtual size_t estimateEncodedSize(size_t srcSize);

    private:
        AesMode mode_;
        AES_KEY key_;
        size_t chunkSize_;
        byte_t *ivec_;
        byte_t *paddingBuffer_;
    };

} } }
