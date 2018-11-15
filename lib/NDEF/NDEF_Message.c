

#include "NDEF_Message.h"

uint8_t recordCount;

NDEF_Message(void) {
    recordCount = 0;
}

NDEF_Message(const byte * data, const int numBytes) {
    recordCount = 0;
    
    int index = 0;
    
    while (index <= numBytes)
    {
        
        // decode tnf - first byte is tnf with bit flags
        // see the NFDEF spec for more info
        byte tnf_byte = data[index];
        bool mb = (tnf_byte & 0x80) != 0;
        bool me = (tnf_byte & 0x40) != 0;
        bool cf = (tnf_byte & 0x20) != 0;
        bool sr = (tnf_byte & 0x10) != 0;
        bool il = (tnf_byte & 0x8) != 0;
        byte tnf = (tnf_byte & 0x7);
        
        NdefRecord record = NdefRecord();
        record.setTnf(tnf);
        
        index++;
        int typeLength = data[index];
        
        int payloadLength = 0;
        if (sr)
        {
            index++;
            payloadLength = data[index];
        }
        else
        {
            payloadLength =
            ((0xFF & data[++index]) << 24)
            | ((0xFF & data[++index]) << 16)
            | ((0xFF & data[++index]) << 8)
            | (0xFF & data[++index]);
        }
        
        int idLength = 0;
        if (il)
        {
            index++;
            idLength = data[index];
        }
        
        index++;
        record.setType(&data[index], typeLength);
        index += typeLength;
        
        if (il)
        {
            record.setId(&data[index], idLength);
            index += idLength;
        }
        
        record.setPayload(&data[index], payloadLength);
        index += payloadLength;
        
        addRecord(record);
        
        if (me) break; // last message
    }
    
}

NDEF_Message(const NDEF_Message& rhs) {
    
    recordCount = rhs.recordCount;
    for (int i = 0; i < recordCount; i++) {
        _records[i] = rhs._records[i];
    }
    
}

NDEF_Message& operator=(const NDEF_Message& rhs)
{
    
    if (this != &rhs)
    {
        
        // delete existing records
        for (int i = 0; i < recordCount; i++)
        {
            // TODO Dave: is this the right way to delete existing records?
            _records[i] = NdefRecord();
        }
        
        recordCount = rhs.recordCount;
        for (int i = 0; i < recordCount; i++)
        {
            _records[i] = rhs._records[i];
        }
    }
    return *this;
}

unsigned int getRecordCount()
{
    return recordCount;
}

int getEncodedSize()
{
    int size = 0;
    for (int i = 0; i < recordCount; i++)
    {
        size += _records[i].getEncodedSize();
    }
    return size;
}

// TODO change this to return uint8_t*
void encode(uint8_t* data)
{
    // assert sizeof(data) >= getEncodedSize()
    uint8_t* data_ptr = &data[0];
    
    for (int i = 0; i < recordCount; i++)
    {
        _records[i].encode(data_ptr, i == 0, (i + 1) == recordCount);
        // TODO can NdefRecord.encode return the record size?
        data_ptr += _records[i].getEncodedSize();
    }
    
}

boolean addRecord(NdefRecord& record)
{
    
    if (recordCount < MAX_NDEF_RECORDS)
    {
        _records[recordCount] = record;
        recordCount++;
        return true;
    }
    else
    {
        Serial.println(F("WARNING: Too many records. Increase MAX_NDEF_RECORDS."));
        return false;
    }
}

void addMimeMediaRecord(String mimeType, String payload)
{
    
    byte payloadBytes[payload.length() + 1];
    payload.getBytes(payloadBytes, sizeof(payloadBytes));
    
    addMimeMediaRecord(mimeType, payloadBytes, payload.length());
}

void addMimeMediaRecord(String mimeType, uint8_t* payload, int payloadLength)
{
    NdefRecord r = NdefRecord();
    r.setTnf(TNF_MIME_MEDIA);
    
    byte type[mimeType.length() + 1];
    mimeType.getBytes(type, sizeof(type));
    r.setType(type, mimeType.length());
    
    r.setPayload(payload, payloadLength);
    
    addRecord(r);
}

void addTextRecord(String text)
{
    addTextRecord(text, "en");
}

void addTextRecord(String text, String encoding)
{
    NdefRecord r = NdefRecord();
    r.setTnf(TNF_WELL_KNOWN);
    
    uint8_t RTD_TEXT[1] = { 0x54 }; // TODO this should be a constant or preprocessor
    r.setType(RTD_TEXT, sizeof(RTD_TEXT));
    
    // X is a placeholder for encoding length
    // TODO is it more efficient to build w/o string concatenation?
    String payloadString = "X" + encoding + text;
    
    byte payload[payloadString.length() + 1];
    payloadString.getBytes(payload, sizeof(payload));
    
    // replace X with the real encoding length
    payload[0] = encoding.length();
    
    r.setPayload(payload, payloadString.length());
    
    addRecord(r);
}

void addUriRecord(String uri)
{
    NdefRecord* r = new NdefRecord();
    r->setTnf(TNF_WELL_KNOWN);
    
    uint8_t RTD_URI[1] = { 0x55 }; // TODO this should be a constant or preprocessor
    r->setType(RTD_URI, sizeof(RTD_URI));
    
    // X is a placeholder for identifier code
    String payloadString = "X" + uri;
    
    byte payload[payloadString.length() + 1];
    payloadString.getBytes(payload, sizeof(payload));
    
    // add identifier code 0x0, meaning no prefix substitution
    payload[0] = 0x0;
    
    r->setPayload(payload, payloadString.length());
    
    addRecord(*r);
    delete(r);
}

void addEmptyRecord()
{
    NdefRecord* r = new NdefRecord();
    r->setTnf(TNF_EMPTY);
    addRecord(*r);
    delete(r);
}

NdefRecord getRecord(int index)
{
    if (index > -1 && index < recordCount)
    {
        return _records[index];
    }
    else
    {
        return NdefRecord(); // would rather return NULL
    }
}

NdefRecord operator[](int index)
{
    return getRecord(index);
}

void print()
{
    Serial.print(F("\nNDEF Message "));Serial.print(recordCount);Serial.print(F(" record"));
    recordCount == 1 ? Serial.print(", ") : Serial.print("s, ");
    Serial.print(getEncodedSize());Serial.println(F(" bytes"));
    
    int i;
    for (i = 0; i < recordCount; i++)
    {
        _records[i].print();
    }
}

