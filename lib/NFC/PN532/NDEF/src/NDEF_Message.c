/*!
 * @file  NDEF_Message.c
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

#include <string.h>
#include "NDEF_Message.h"
#include "NDEF_Record.h"

/**
 *
 */
uint32_t NDEF_Message_getEncodedSize (NDEF_Message ndef_message) {
    
    uint32_t size = 0;
    // accumulate size
    for (int i = 0; i < ndef_message.record_count; i++)
        size += NDEF_Record_getEncodedSize(ndef_message.records[i]);
    
    return size;
}

/**
 *
 */
void NDEF_Message_encode(NDEF_Message ndef_message, uint8_t* data_buffer) {
    
    for (int i = 0; i < ndef_message.record_count; i++) {
        NDEF_Record_encode(ndef_message.records[i], data_buffer, i == 0, (i + 1) == ndef_message.record_count);
        data_buffer += NDEF_Record_getEncodedSize(ndef_message.records[i]);
    }
    
}





//void addMimeMediaRecord(String mimeType, String payload)
//{
//
//    byte payloadBytes[payload.length() + 1];
//    payload.getBytes(payloadBytes, sizeof(payloadBytes));
//
//    addMimeMediaRecord(mimeType, payloadBytes, payload.length());
//}
//
//void addMimeMediaRecord(String mimeType, uint8_t* payload, int payloadLength)
//{
//    NdefRecord r = NdefRecord();
//    r.setTnf(TNF_MIME_MEDIA);
//
//    byte type[mimeType.length() + 1];
//    mimeType.getBytes(type, sizeof(type));
//    r.setType(type, mimeType.length());
//
//    r.setPayload(payload, payloadLength);
//
//    addRecord(r);
//}

//void addTextRecord(String text)
//{
//    addTextRecord(text, "en");
//}

///**
// * add_text_record
// * ----------
// * @param  ndef_message    the ndef message which the new record is appended to.
// * @param  prefix          prefix for uri.
// * @param  uri             actual uri payload.
// * @param  payload_buffer  payload buffer point to store the result payload.
// * ----------
// * @brief add an uri record to the given ndef message.
// */
//void add_text_record (NDEF_Message* ndef_message, String text, String encoding) {

//    NDEF_Record* ndef_record = &ndef_message->records[ndef_message->record_count];
//    // tnf byte as well known
//    ndef_record->tnf = TNF_WELL_KNOWN;
//    // U for Uri type
//    ndef_record->type = "T";



//    // X is a placeholder for encoding length
//    // TODO is it more efficient to build w/o string concatenation?
//    String payloadString = "X" + encoding + text;

//    byte payload[payloadString.length() + 1];
//    payloadString.getBytes(payload, sizeof(payload));

//    // replace X with the real encoding length
//    payload[0] = encoding.length();

//    r.setPayload(payload, payloadString.length());


//}


/**
 * add_uri_record
 * ----------
 * @param  ndef_message    the ndef message which the new record is appended to.
 * @param  prefix          prefix for uri.
 * @param  uri             actual uri payload.
 * @param  payload_buffer  payload buffer point to store the result payload.
 * ----------
 * @brief add an uri record to the given ndef message.
 */
void add_uri_record (NDEF_Message* ndef_message, uint8_t prefix, char* uri, char* payload_buffer) {
    
    NDEF_Record* ndef_record = &ndef_message->records[ndef_message->record_count];
    // tnf byte as well known
    ndef_record->tnf = TNF_WELL_KNOWN;
    // U for Uri type
    ndef_record->type = "U";
    // wrtie type length
    ndef_record->type_length = strlen(ndef_record->type);
    // prefix sub
    payload_buffer[0] = prefix;
    // copy uri content into temp string
    strcpy (payload_buffer + 1, uri);
//    // safe practice, add string termination byte
//    temp_str[1 + strlen(uri)] = '\0';
    // copy temp string into payload
    ndef_record->payload = payload_buffer;
    // write payload length
    ndef_record->payload_length = strlen(ndef_record->payload);
    // increment record count
    ndef_message->record_count++;
    
}

//void addEmptyRecord()
//{
//    NdefRecord* r = new NdefRecord();
//    r->setTnf(TNF_EMPTY);
//    addRecord(*r);
//    delete(r);
//}





//void print()
//{
//    Serial.print(F("\nNDEF Message "));Serial.print(recordCount);Serial.print(F(" record"));
//    recordCount == 1 ? Serial.print(", ") : Serial.print("s, ");
//    Serial.print(getEncodedSize());Serial.println(F(" bytes"));
//
//    int i;
//    for (i = 0; i < recordCount; i++)
//    {
//        _records[i].print();
//    }
//}

