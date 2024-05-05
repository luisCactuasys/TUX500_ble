/*
 * M3Crc32.h
 *
 *  Created on: 29/06/2023
 *      Author: Utilizador
 */



#ifndef _CRC32_H_
#define _CRC32_H_           //!< Evitar dupla inclus�o deste ficheiro "header"


/*
 * *********************** Includes *******************************************
 */
#include <stdint.h>

/*
 * ********************* Defines **********************************************
 */
#define CRC_INITIALVALUE 0xFFFFFFFF
#define POLY 0x4C11DB7

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
/*
 * ********************** Public Function Prototypes **************************
 */

/*!
 * \brief Initializes calculate crc32.
 *
 * \param start Initial data.
 *
 * \return Current result of crc32.
 */
uint32_t M3Set_crc32_init_value (uint32_t start);

/*!
 * \brief Calculates the crc32 of a data buffer.
 *
 * \param sum Previous result..
 * \param p Pointer to the data to calculate..
 * \param len Size of the data to calculate.
 *
 * \return Current result of crc32.
 */
uint32_t M3Fast_crc32(uint32_t sum, uint8_t *p, uint32_t len);

#endif
