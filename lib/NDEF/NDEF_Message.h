

#ifndef __NDEF_MESSAGE_H__
#define __NDEF_MESSAGE_H__

#include <Ndef.h>
#include <NDEF_Record.h>

#define MAX_NDEF_RECORDS 4

void NdefMessage(void);
void NdefMessage(const byte *data, const int numBytes);
void NdefMessage(const NdefMessage& rhs);
void NdefMessage& operator=(const NdefMessage& rhs);
int getEncodedSize(); // need so we can pass array to encode
void encode(byte *data);
int addRecord(NdefRecord& record);
void addMimeMediaRecord(String mimeType, String payload);
void addMimeMediaRecord(String mimeType, byte *payload, int payloadLength);
void addTextRecord(String text);
void addTextRecord(String text, String encoding);
void addUriRecord(String uri);
void addEmptyRecord();
uint32_t getRecordCount();
NdefRecord getRecord(int index);
NdefRecord operator[](int index);
void print();
NdefRecord _records[MAX_NDEF_RECORDS];

#endif
