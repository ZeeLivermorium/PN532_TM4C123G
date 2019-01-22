/*!
 * @file NDEF_Record.c
 * @brief
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
 * @date Nov 13, 2018
 */

#include <stdint.h>
#include <string.h>
#include "NDEF_Record.h"

//NdefRecord(const NdefRecord& rhs) {
//    //Serial.println("NdefRecord Constructor 2 (copy)");
//
//    _tnf = rhs._tnf;
//    _typeLength = rhs._typeLength;
//    _payloadLength = rhs._payloadLength;
//    _idLength = rhs._idLength;
//    _type = (byte *)NULL;
//    _payload = (byte *)NULL;
//    _id = (byte *)NULL;
//
//    if (_typeLength)
//    {
//        _type = (byte*)malloc(_typeLength);
//        memcpy(_type, rhs._type, _typeLength);
//    }
//
//    if (_payloadLength)
//    {
//        _payload = (byte*)malloc(_payloadLength);
//        memcpy(_payload, rhs._payload, _payloadLength);
//    }
//
//    if (_idLength)
//    {
//        _id = (byte*)malloc(_idLength);
//        memcpy(_id, rhs._id, _idLength);
//    }
//
//}
//
//
//NdefRecord& operator=(const NdefRecord& rhs)
//{
//    //Serial.println("NdefRecord ASSIGN");
//
//    if (this != &rhs)
//    {
//        // free existing
//        if (_typeLength)
//        {
//            free(_type);
//        }
//
//        if (_payloadLength)
//        {
//            free(_payload);
//        }
//
//        if (_idLength)
//        {
//            free(_id);
//        }
//
//        _tnf = rhs._tnf;
//        _typeLength = rhs._typeLength;
//        _payloadLength = rhs._payloadLength;
//        _idLength = rhs._idLength;
//
//        if (_typeLength)
//        {
//            _type = (byte*)malloc(_typeLength);
//            memcpy(_type, rhs._type, _typeLength);
//        }
//
//        if (_payloadLength)
//        {
//            _payload = (byte*)malloc(_payloadLength);
//            memcpy(_payload, rhs._payload, _payloadLength);
//        }
//
//        if (_idLength)
//        {
//            _id = (byte*)malloc(_idLength);
//            memcpy(_id, rhs._id, _idLength);
//        }
//    }
//    return *this;
//}

/*
 *  Bit 7     6       5       4       3       2       1       0
 * ------  ------  ------  ------  ------  ------  ------  ------
 * [ MB ]  [ ME ]  [ CF ]  [ SR ]  [ IL ]  [        TNF         ]
 *
 * [                        TYPE LENGTH                         ]
 *
 * [                       PAYLOAD LENGTH                       ]
 *
 * [                         ID LENGTH                          ]
 *
 * [                        RECORD TYPE                         ]
 *
 * [                            ID                              ]
 *
 * [                          PAYLOAD                           ]
 */

/**
 *
 */
void NDEF_Record_encode (NDEF_Record ndef_record, uint8_t* data_buffer, uint8_t first_record, uint8_t last_record) {
    // assert data > getEncodedSize()
    data_buffer[0] = encodeHeader(ndef_record, first_record, last_record);
    data_buffer[1] = ndef_record.type_length;
    
    data_buffer += 2;
    
    if (ndef_record.payload_length <= 0xFF) {  // short record
        *data_buffer = ndef_record.payload_length;
        data_buffer++;
    } else { // long format
        // 4 bytes but we store length as an int
        data_buffer[0] = 0x0; // (_payloadLength >> 24) & 0xFF;
        data_buffer[1] = 0x0; // (_payloadLength >> 16) & 0xFF;
        data_buffer[2] = (ndef_record.payload_length >> 8) & 0xFF;
        data_buffer[3] = ndef_record.payload_length & 0xFF;
        data_buffer += 4;
    }
    
    if (ndef_record.id_length) {
        *data_buffer = ndef_record.id_length;
        data_buffer++;
    }
    
    memcpy(data_buffer, ndef_record.type, ndef_record.type_length);
    data_buffer += ndef_record.type_length;
    
    memcpy(data_buffer, ndef_record.payload, ndef_record.payload_length);
    data_buffer += ndef_record.payload_length;
    
    if (ndef_record.id_length) {
        memcpy(data_buffer, ndef_record.id, ndef_record.id_length);
        data_buffer += ndef_record.id_length;
    }
}


/**
 *
 */
uint8_t encodeHeader (NDEF_Record ndef_record, uint8_t first_record, uint8_t last_record) {
    
    uint8_t byte = ndef_record.tnf;
    // MB bit
    if (first_record) {
        byte = byte | 0x80;
    }
    // ME bit
    if (last_record) {
        byte = byte | 0x40;
    }
    // CF(chunked flag) bit
    // if (cf) {
    //     byte = byte | 0x20;
    // }
    // SR(short record) bit
    if (ndef_record.payload_length <= 0xFF) {
        byte = byte | 0x10;
    }
    // IL(id length) bit
    if (ndef_record.id_length) {
        byte = byte | 0x8;
    }
    
    return byte;
}

/**
 *
 */
uint32_t NDEF_Record_getEncodedSize (NDEF_Record ndef_record) {
    // header + typeLength
    int size = 2;
    // if not short record, payload length would be 32 bit
    if (ndef_record.payload_length > 0xFF) {
        size += 4;
    } else {
        // if short record, payload length would be 8 bit
        size += 1;
    }
    // add ID
    if (ndef_record.id_length) {
        size += 1;
    }
    // add the lengths together
    size += ndef_record.type_length + 
		        ndef_record.payload_length + 
		        ndef_record.id_length;
    
    return size;
}


//void print()
//{
//    Serial.println(F("  NDEF Record"));
//    Serial.print(F("    TNF 0x"));Serial.print(_tnf, HEX);Serial.print(" ");
//    switch (_tnf) {
//        case TNF_EMPTY:
//            Serial.println(F("Empty"));
//            break;
//        case TNF_WELL_KNOWN:
//            Serial.println(F("Well Known"));
//            break;
//        case TNF_MIME_MEDIA:
//            Serial.println(F("Mime Media"));
//            break;
//        case TNF_ABSOLUTE_URI:
//            Serial.println(F("Absolute URI"));
//            break;
//        case TNF_EXTERNAL_TYPE:
//            Serial.println(F("External"));
//            break;
//        case TNF_UNKNOWN:
//            Serial.println(F("Unknown"));
//            break;
//        case TNF_UNCHANGED:
//            Serial.println(F("Unchanged"));
//            break;
//        case TNF_RESERVED:
//            Serial.println(F("Reserved"));
//            break;
//        default:
//            Serial.println();
//    }
//    Serial.print(F("    Type Length 0x"));Serial.print(_typeLength, HEX);Serial.print(" ");Serial.println(_typeLength);
//    Serial.print(F("    Payload Length 0x"));Serial.print(_payloadLength, HEX);;Serial.print(" ");Serial.println(_payloadLength);
//    if (_idLength)
//    {
//        Serial.print(F("    Id Length 0x"));Serial.println(_idLength, HEX);
//    }
//    Serial.print(F("    Type "));PrintHexChar(_type, _typeLength);
//    // TODO chunk large payloads so this is readable
//    Serial.print(F("    Payload "));PrintHexChar(_payload, _payloadLength);
//    if (_idLength)
//    {
//        Serial.print(F("    Id "));PrintHexChar(_id, _idLength);
//    }
//    Serial.print(F("    Record is "));Serial.print(getEncodedSize());Serial.println(" bytes");
//
//}
//
