/*!
 * @file  PN532_cardEmulation.c
 * @brief PN532 card emulation APIs and their implementations.
 * ----------
 * Adapted code from Seeed Studio PN532 driver for Arduino.
 * You can find the Seeed Studio PN532 driver here: https://github.com/Seeed-Studio/PN532
 * ----------
 * NXP PN532 Data Sheet:  https://www.nxp.com/docs/en/nxp/data-sheets/PN532_C1.pdf
 * NXP PN532 User Manual: https://www.nxp.com/docs/en/user-guide/141520.pdf
 * ----------
 * For future development and updates, please follow this repository: https://github.com/ZeeLivermorium/PN532_TM4C123
 * ----------
 * If you find any bug or problem, please create new issue or a pull request with a fix in the repository.
 * Or you can simply email me about the problem or bug at zeelivermorium@gmail.com
 * Much Appreciated!
 * ----------
 * @author Zee Livermorium
 * @date   Nov 13, 2018
 */

#include <stdint.h>
#include <string.h>
#include "PN532.h"
#include "PN532_cardEmulation.h"

#define MAX_TGREAD

// Command APDU
#define C_APDU_CLA   0
#define C_APDU_INS   1 // instruction
#define C_APDU_P1    2 // parameter 1
#define C_APDU_P2    3 // parameter 2
#define C_APDU_LC    4 // length command
#define C_APDU_DATA  5 // data

#define C_APDU_P1_SELECT_BY_ID   0x00
#define C_APDU_P1_SELECT_BY_NAME 0x04

// Response APDU
#define R_APDU_SW1_COMMAND_COMPLETE 0x90
#define R_APDU_SW2_COMMAND_COMPLETE 0x00

#define R_APDU_SW1_NDEF_TAG_NOT_FOUND 0x6a
#define R_APDU_SW2_NDEF_TAG_NOT_FOUND 0x82

#define R_APDU_SW1_FUNCTION_NOT_SUPPORTED 0x6A
#define R_APDU_SW2_FUNCTION_NOT_SUPPORTED 0x81

#define R_APDU_SW1_MEMORY_FAILURE 0x65
#define R_APDU_SW2_MEMORY_FAILURE 0x81

#define R_APDU_SW1_END_OF_FILE_BEFORE_REACHED_LE_BYTES 0x62
#define R_APDU_SW2_END_OF_FILE_BEFORE_REACHED_LE_BYTES 0x82

// ISO7816-4 commands
#define ISO7816_SELECT_FILE 0xA4
#define ISO7816_READ_BINARY 0xB0
#define ISO7816_UPDATE_BINARY 0xD6
/****************************************************
 *                                                  *
 *                    Properties                    *
 *                                                  *
 ****************************************************/
 
uint8_t* cardUid;
uint8_t  ndef_file[128];
uint8_t  tag_writeable = 1;
uint8_t  tag_writtenByInitiator = 0;

void (*updateNdefCallback)(uint8_t *ndef, uint16_t length);

typedef enum { NONE, CC, NDEF } tag_file;

/**
 * emulation_setNDEFFile
 * ----------
 * @param  data         data to be stored in the tag.
 * @param  data_length  length of data.
 * ----------
 * @brief  set NDEF data for the tag to be emulated.
 */
void emulation_setNDEFFile (uint8_t* data, const int16_t data_length) {
    // check size maybe
    ndef_file[0] = data_length >> 8;
    ndef_file[1] = data_length & 0xFF;
    memcpy (ndef_file + 2, data, data_length);
}

/**
 * emulation_setUid
 * ----------
 * @param  uid  pointer to byte array of length 3 (uid is 4 bytes - first byte is fixed) or zero for uid.
 * ----------
 * @brief  set uid of the tag to be emulated.
 */
void emulation_setUid (uint8_t* uid) {
    cardUid = uid;
}

/**
 * emulate_card
 * ----------
 * @param  wait_time  wait time for a response for emulation.
 * ----------
 * @return .
 * ----------
 * @brief  emulate PN532 as a tag.
 */
uint8_t emulate_card (const uint16_t wait_time) {
    // command array
    uint8_t cmd[] = {
        PN532_COMMAND_TGINITASTARGET,
        0x05,                // MODE: PICC only, Passive only
        
        0x04, 0x00,          // SENS_RES
        0x00, 0x00, 0x00,    // NFCID1, SINGLE
        0x20,                // SEL_RES
        
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,     // FeliCaParams
        0,0,
        
        0,0,0,0,0,0,0,0,0,0, // NFCID3t
        
        0,                   // length of general bytes
        0                    // length of historical bytes
    };
    
    if (cardUid) {
        memcpy (cmd + 4, cardUid, 3);       // copy uid into command array
    }
    
    if (PN532_tgInitAsTarget(cmd, sizeof(cmd), wait_time) != 1) {
        return 0;                           // 0 for error
    }
    
    uint8_t compatibility_container[] = {
        0, 0x0F,
        0x20,
        0, 0x54,
        0, 0xFF,
        0x04,       // T
        0x06,       // L
        0xE1, 0x04, // File identifier
        ((NDEF_MAX_LENGTH & 0xFF00) >> 8), (NDEF_MAX_LENGTH & 0xFF), // maximum NDEF file size
        0x00,       // read access 0x0 = granted
        0x00        // write access 0x0 = granted | 0xFF = deny
    };
    
    if (!tag_writeable){
        compatibility_container[14] = 0xFF;
    }
    
    tag_writtenByInitiator = 0;
    
    uint8_t rw_buffer[128];
    uint8_t data_length;
    tag_file current_file = NONE;
    
    while(1){
        
        if (PN532_tgGetData(rw_buffer, sizeof(rw_buffer)) < 0) {
            PN532_inRelease(0);
            return 1;
        }
        
        uint8_t c_apdu_p1 = rw_buffer[C_APDU_P1];
        uint8_t c_apdu_p2 = rw_buffer[C_APDU_P2];
        uint8_t c_apdu_lc = rw_buffer[C_APDU_LC];
        uint16_t c_apdu_param = ((uint16_t) c_apdu_p1 << 8) + c_apdu_p2;
        uint8_t ndef_tag_application_name_v2[] = {0, 0x7, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
        switch (rw_buffer[C_APDU_INS]) {
                
            case ISO7816_SELECT_FILE:
                switch (c_apdu_p1) {
                    case C_APDU_P1_SELECT_BY_ID:
                        if (c_apdu_p2 != 0x0c) {
                            set_response(COMMAND_COMPLETE, rw_buffer, &data_length, 0);
                        } else if(c_apdu_lc == 2 && rw_buffer[C_APDU_DATA] == 0xE1 && (rw_buffer[C_APDU_DATA+1] == 0x03 || rw_buffer[C_APDU_DATA+1] == 0x04)){
                            set_response(COMMAND_COMPLETE, rw_buffer, &data_length, 0);
                            if(rw_buffer[C_APDU_DATA+1] == 0x03){
                                current_file = CC;
                            } else if(rw_buffer[C_APDU_DATA+1] == 0x04){
                                current_file = NDEF;
                            }
                        } else {
                            set_response(TAG_NOT_FOUND, rw_buffer, &data_length, 0);
                        }
                        break;
                    case C_APDU_P1_SELECT_BY_NAME:
                        if(0 == memcmp(ndef_tag_application_name_v2, rw_buffer + C_APDU_P2, sizeof(ndef_tag_application_name_v2))){
                            set_response(COMMAND_COMPLETE, rw_buffer, &data_length, 0);
                        } else{
                            set_response(FUNCTION_NOT_SUPPORTED, rw_buffer, &data_length, 0);
                        }
                        break;
                }
                break;
                
            case ISO7816_READ_BINARY:
                switch(current_file){
                    case NONE:
                        set_response(TAG_NOT_FOUND, rw_buffer, &data_length, 0);
                        break;
                    case CC:
                        if( c_apdu_param > NDEF_MAX_LENGTH){
                            set_response(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rw_buffer, &data_length, 0);
                        }else {
                            memcpy(rw_buffer,compatibility_container + c_apdu_param, c_apdu_lc);
                            set_response(COMMAND_COMPLETE, rw_buffer + c_apdu_lc, &data_length, c_apdu_lc);
                        }
                        break;
                    case NDEF:
                        if( c_apdu_param > NDEF_MAX_LENGTH){
                            set_response(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rw_buffer, &data_length, 0);
                        }else {
                            memcpy(rw_buffer, ndef_file + c_apdu_param, c_apdu_lc);
                            set_response(COMMAND_COMPLETE, rw_buffer + c_apdu_lc, &data_length, c_apdu_lc);
                        }
                        break;
                }
                break;
                
            case ISO7816_UPDATE_BINARY:
                if(!tag_writeable){
                    set_response(FUNCTION_NOT_SUPPORTED, rw_buffer, &data_length, 0);
                } else{
                    if( c_apdu_param > NDEF_MAX_LENGTH){
                        set_response(MEMORY_FAILURE, rw_buffer, &data_length, 0);
                    }
                    else{
                        memcpy(ndef_file + c_apdu_param, rw_buffer + C_APDU_DATA, c_apdu_lc);
                        set_response(COMMAND_COMPLETE, rw_buffer, &data_length, 0);
                        tag_writtenByInitiator = 1;
                        
                        uint16_t ndef_length = (ndef_file[0] << 8) + ndef_file[1];
                        if ((ndef_length > 0) && (updateNdefCallback != 0)) {
                            updateNdefCallback(ndef_file + 2, ndef_length);
                        }
                    }
                }
                break;
                
            default:
                set_response(FUNCTION_NOT_SUPPORTED, rw_buffer, &data_length, 0);
        }
        
        if (PN532_tgSetData(rw_buffer, data_length) == 0) {
            PN532_inRelease(0);
            return 1;
        }
    }

}


/*
 *
 */
void set_response (responseCommand cmd, uint8_t* buffer, uint8_t* length, uint8_t offset){
    switch (cmd) {
        case COMMAND_COMPLETE:
            buffer[0] = R_APDU_SW1_COMMAND_COMPLETE;
            buffer[1] = R_APDU_SW2_COMMAND_COMPLETE;
            *length = 2 + offset;
            break;
        case TAG_NOT_FOUND:
            buffer[0] = R_APDU_SW1_NDEF_TAG_NOT_FOUND;
            buffer[1] = R_APDU_SW2_NDEF_TAG_NOT_FOUND;
            *length = 2;
            break;
        case FUNCTION_NOT_SUPPORTED:
            buffer[0] = R_APDU_SW1_FUNCTION_NOT_SUPPORTED;
            buffer[1] = R_APDU_SW2_FUNCTION_NOT_SUPPORTED;
            *length = 2;
            break;
        case MEMORY_FAILURE:
            buffer[0] = R_APDU_SW1_MEMORY_FAILURE;
            buffer[1] = R_APDU_SW2_MEMORY_FAILURE;
            *length = 2;
            break;
        case END_OF_FILE_BEFORE_REACHED_LE_BYTES:
            buffer[0] = R_APDU_SW1_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
            buffer[1] = R_APDU_SW2_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
            *length = 2;
            break;
    }
}
