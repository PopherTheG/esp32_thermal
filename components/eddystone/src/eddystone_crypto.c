/*
 * eddystone_crypto.c
 *
 *  Created on: Apr 23, 2020
 *      Author: spestano@spinginemail.com
 */
#include <assert.h>
#include <string.h>
#include "mbedtls/md.h"
#include "mbedtls/aes.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "eddystone_crypto.h"

// SRC: https://github.com/google/eddystone/blob/bb8738d7ddac0ddd3dfa70e594d011a0475e763d/implementations/mbed/source/EIDFrame.cpp

void eddystone_swap_endian_array(uint8_t* array, uint8_t* big_endian, size_t len)
{
    for (int i = 0; i < len; i++) {
        big_endian[i] = array[len - i - 1];
    }
}
void eddystone_crypto_aes_128_encrypt(uint8_t* key, uint8_t* in, uint8_t* out)
{
    mbedtls_aes_context aes_ctx;
    mbedtls_aes_init(&aes_ctx);
    mbedtls_aes_setkey_enc(&aes_ctx, key, 128);
    mbedtls_aes_crypt_ecb(&aes_ctx, MBEDTLS_AES_ENCRYPT, in, out);
    mbedtls_aes_free(&aes_ctx);
}

void eddystone_crypto_gen_key(uint8_t* pub_key, uint8_t* priv_key)
{
    mbedtls_ecdh_context ecdh_ctx;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_entropy_context entropy;
    mbedtls_ecdh_init( &ecdh_ctx );
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_init(&entropy);

    assert(mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                    NULL, 0) == 0);


    if (mbedtls_ecp_group_load(&ecdh_ctx.grp, MBEDTLS_ECP_DP_CURVE25519) != 0) {
        return ;//EID_GRP_FAIL;
    }
    if (mbedtls_ecdh_gen_public(&ecdh_ctx.grp, &ecdh_ctx.d, &ecdh_ctx.Q, mbedtls_ctr_drbg_random, &ctr_drbg) != 0) {
        return ;//EID_GENKEY_FAIL;
    }

    mbedtls_mpi_write_binary(&ecdh_ctx.d, priv_key, ECDH_KEY_SIZE);
    mbedtls_mpi_write_binary(&ecdh_ctx.Q.X, pub_key, ECDH_KEY_SIZE);

    mbedtls_ecdh_free( &ecdh_ctx );
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);
}

void eddystone_crypto_gen_ecdh_shared_key(uint8_t *pub_key, uint8_t *priv_key,
        uint8_t *server_pub_key, uint8_t *eid_out)
{
    int16_t ret = 0;
    uint8_t tmp[32];

    mbedtls_ecdh_context ecdh_ctx;
    // initialize context
    mbedtls_ecdh_init( &ecdh_ctx );
    mbedtls_ecp_group_load( &ecdh_ctx.grp, MBEDTLS_ECP_DP_CURVE25519 );

    // copy binary beacon private key (previously generated!) into context
    // Note: As the PrivateKey is generated locally, it is Big Endian
    ret = mbedtls_mpi_read_binary( &ecdh_ctx.d, priv_key, ECDH_KEY_SIZE);

    // copy server-public-key (received through GATT characteristic 10) into context
    ret = mbedtls_mpi_lset( &ecdh_ctx.Qp.Z, 1 );
    eddystone_swap_endian_array(server_pub_key, tmp, ECDH_KEY_SIZE); // To make it Big Endian
    ret = mbedtls_mpi_read_binary( &ecdh_ctx.Qp.X, tmp , ECDH_KEY_SIZE );

    // ECDH point multiplication
    size_t olen; // actual size of shared secret
    uint8_t sharedSecret[32]; // shared ECDH secret
    memset(sharedSecret, 0, 32);
    ret = mbedtls_ecdh_calc_secret( &ecdh_ctx, &olen, sharedSecret, sizeof(sharedSecret), NULL, NULL );
    eddystone_swap_endian_array(sharedSecret, tmp, 32);
    memcpy(sharedSecret, tmp, 32);
//    LOG(("Shared secret=")); EddystoneService::logPrintHex(sharedSecret, 32);
    assert(olen == sizeof(sharedSecret));
    assert(ret != MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    // Convert the shared secret to key material using HKDF-SHA256. HKDF is used with
    // the salt set to a concatenation of the resolver's public key and beacon's
    // public key, with a null context.

    // build HKDF key
    unsigned char k[ 64 ];
    eddystone_swap_endian_array(pub_key, tmp, ECDH_KEY_SIZE);
    memcpy(&k[0], server_pub_key, ECDH_KEY_SIZE);
    memcpy(&k[32], tmp, ECDH_KEY_SIZE);

    // compute HKDF: see https://tools.ietf.org/html/rfc5869
    mbedtls_md_context_t md_ctx;
    mbedtls_md_init( &md_ctx );
    mbedtls_md_setup( &md_ctx, mbedtls_md_info_from_type( MBEDTLS_MD_SHA256 ), 1 );
    mbedtls_md_hmac_starts( &md_ctx, k, sizeof( k ) );
    mbedtls_md_hmac_update( &md_ctx, sharedSecret, sizeof(sharedSecret) );
    unsigned char prk[ 32 ];
    mbedtls_md_hmac_finish( &md_ctx, prk );
    mbedtls_md_hmac_starts( &md_ctx, prk, sizeof( prk ) );
    const unsigned char const1[] = { 0x01 };
    mbedtls_md_hmac_update( &md_ctx, const1, sizeof( const1 ) );
    unsigned char t[ 32 ];
    mbedtls_md_hmac_finish( &md_ctx, t );

    //Truncate the key material to 16 bytes (128 bits) to convert it to an AES-128 secret key.
    memcpy( eid_out, t, 16 );
//    LOG(("\r\nEIDIdentityKey=")); EddystoneService::logPrintHex(t, 32); LOG(("\r\n"));

    mbedtls_md_free( &md_ctx );
    mbedtls_ecdh_free( &ecdh_ctx );
}
