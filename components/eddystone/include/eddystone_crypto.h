/*
 * eddystone_crypto.h
 *
 *  Created on: Apr 23, 2020
 *      Author: spestano@spinginemail.com
 */

#ifndef MAIN_EDDYSTONE_CRYPTO_H_
#define MAIN_EDDYSTONE_CRYPTO_H_

#include <stdlib.h>
#include <stdint.h>

#define ECDH_KEY_SIZE 32
void eddystone_swap_endian_array(uint8_t* array, uint8_t* big_endian, size_t len);

void eddystone_crypto_aes_128_encrypt(uint8_t* key, uint8_t* in, uint8_t* out);

void eddystone_crypto_gen_key(uint8_t* pub_key, uint8_t* priv_key);

void eddystone_crypto_gen_ecdh_shared_key(uint8_t *pub_key, uint8_t *priv_key,
        uint8_t *server_pub_key, uint8_t *eid_out);

#endif /* MAIN_EDDYSTONE_CRYPTO_H_ */
