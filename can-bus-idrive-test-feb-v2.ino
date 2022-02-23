// 
// BMW IDrive controller - cira E60 age.
// Uses MCP2515 as a CAN controller and TJA1054 for the driver IC
// Low speed CAN2.0A -> 100kb/s.
//
// Code tested on Teensy 3.2. 
//

#include "jb_mcp_can.h"
#include <SPI.h>

// send
unsigned long prevTX = 0;
unsigned long prevTX1 = 0;
unsigned long prevTX2 = 0;
unsigned long prevTX3 = 0;

long unsigned int myTxID = 0x4A8;
byte myTxData[8] = {0x68, 0x01, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

byte data_0C0[] = {0xF0,0xFF};                                            // Alive Central Gateway
int AliveGatewayByte = 0xF0;

byte data_480[] = {0x00, 0x01, 0xFF, 0x21, 0x00, 0x02, 0x02, 0x04};       // CanOpen - "master init"?
byte data_1AA[] = {0x00, 0x00, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0};            // Effect Ergo Commander [10]

long unsigned int highestNodeID = 0x0;

// recieve
long unsigned int rxID;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
bool mcpconfigok = false;
bool Send1AAMsg = false;
byte sndStat;

/*   
  0x4E7... status from this controller?
  0x1B8... data from the idrive (Bedienung ErgoCommander [6])   
*/

#define CAN0_INT 9                          // Set INT to pin 
MCP_CAN CAN0(10);                           // Set CS to pin 10

void setup()
{
    Serial.begin(115200); 
    delay(2000);
  
    // Initialize MCP2515 and the masks and filters disabled.
    while(mcpconfigok==false)
    {
        if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK)
        {
            Serial.println("MCP2515 Initialized Successfully!");
            mcpconfigok = true;
        } else {
            Serial.println("Error Initializing MCP2515...");
            delay(2000);            
        }
    }

    CAN0.setMode(MCP_NORMAL);                             // Change to normal mode to allow messages to be transmitted    
    pinMode(CAN0_INT, INPUT);                             // Configuring pin for /INT input       
    prevTX = millis();                                    //
    delay(200);
    prevTX1 = millis();
    delay(200);
    prevTX2 = millis();
    delay(200);
    prevTX3 = millis();    
}

void loop()
{
    if(Send1AAMsg == false)
    {
        if(millis() - prevTX >= 1500)
        {
            // initial sync message?        
            prevTX = millis();
            Serial.println("");
            sprintf(msgString, "TX ID: 0x%01X            Data:", 0x480);
            Serial.print(msgString);
            debugDataArray(data_480, 8);
            sndStat = CAN0.sendMsgBuf(0x480, 0, 8, data_480);
            SendStatResult(sndStat);
    
            highestNodeID = 0;
        }

        if(millis() - prevTX2 >= 200)
        {
            // alive central gateway - 200ms
            prevTX2 = millis();
            //sprintf(msgString, "TX ID: 0x%.3lX", 0x0C0);
            //Serial.print(msgString);                     
            sndStat = CAN0.sendMsgBuf(0x0C0, 0, 2, data_0C0);
            //debugDataArray(data_0C0, 2);
            data_0C0[0]++;
            if(data_0C0[0] == 0x00)  {
                // counter wraps > 0xFF, so reset.
                data_0C0[0] = 0xF0;
            }        
            //SendStatResult(sndStat);
        }
        
    }

    if(Send1AAMsg == true)
    {
      if(millis() - prevTX3 >= 500)
      {
          // poll controller          
          prevTX3 = millis();

          sprintf(msgString, "TX ID: 0x%01X            Data:", 0x1AA);
          Serial.print(msgString);
          debugDataArray(data_1AA, 8);
          sndStat = CAN0.sendMsgBuf(0x1AA, 0, 8, data_1AA);
          SendStatResult(sndStat);          
      }      
    }
 
    // recieve
    if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxID, &len, rxBuf);           // Read data: len = data length, buf = data byte(s)

        if (rxID == 0x1B8)
        {
            // iDrive controller messages.
            // Bedienung ErgoCommander [6]

            sprintf(msgString, "RX ID: 0x%01X   Len: %1d   Data:", rxID, len);
            Serial.print(msgString);
            for(byte i = 0; i<len; i++)
            {
                sprintf(msgString, " 0x%.2X", rxBuf[i]);
                Serial.print(msgString);
            }

            Serial.print("             (Knob:"); 
            switch(rxBuf[0])
            {
                case 0x00:
                    Serial.print("up,"); 
                break;              
                case 0x02:
                    Serial.print("right,"); 
                break;                 
                case 0x04:
                    Serial.print("down,"); 
                break;              
                case 0x06:
                    Serial.print("left,"); 
                break;           
            }
            if(rxBuf[1] == 0xC0)
            {
                Serial.print("no push,");
            }
            else
            {
                Serial.print("push   ,");
            }
            Serial.print("rotate:");
            if(rxBuf[2] >= 0x1 and rxBuf[2] < 0x7F)
            {
                // CW
                Serial.print("CW ");
                Serial.print(rxBuf[2]);
            }
            else if (rxBuf[2] > 0x7F and rxBuf[2] <= 0xFF)
            {
                // CCW
                Serial.print("CCW ");
                Serial.print(0xFF-rxBuf[2]); 
            }
            else
            {
                Serial.print("none");
            }
            Serial.println(")");
        }
        else        
        {
            /*
            if(rxID != 0x4E7)
            {
                sprintf(msgString, "RX ID: 0x%01X***Len: %1d   Data:", rxID, len);
                Serial.print(msgString);
                for(byte i = 0; i<len; i++)
                {
                    sprintf(msgString, " 0x%.2X", rxBuf[i]);
                    Serial.print(msgString);
                }
                Serial.println("");
            }
            else
            {
                sprintf(msgString, "RX ID: 0x%01X   Len: %1d   Data:", rxID, len);
                Serial.print(msgString);
                for(byte i = 0; i<len; i++)
                {
                    sprintf(msgString, " 0x%.2X", rxBuf[i]);
                    Serial.print(msgString);
                }
                Serial.println("");
            } */
            
        }        

        // negotition / keep alive?
        if (rxID >= 0x480 and rxID < 0x500)
        {
            if(rxBuf[0] > highestNodeID)
            {
                highestNodeID = rxBuf[0];
            }
        }

        switch(rxID)
        {
            case 0x4E7:     // our iDrive Controller

                if(Send1AAMsg == false)
                {
                    if (rxBuf[1] == 0x1)
                    {
                        sprintf(msgString, "TX ID: 0x%01X            Data:", myTxID);
                        Serial.print(msgString);                  
                        myTxData[0] = myTxID;
                        myTxData[1] = 0x1;               
                        debugDataArray(myTxData, 8);
                        sndStat = CAN0.sendMsgBuf(myTxID, 0, 8, myTxData);
                        SendStatResult(sndStat);                               
                    }
                    else if (rxBuf[1] == 0x12)
                    {
                        sprintf(msgString, "TX ID: 0x%01X            Data:", myTxID);
                        Serial.print(msgString);
                        myTxData[0] = highestNodeID;
                        myTxData[1] = 0x12;
                        debugDataArray(myTxData, 8);
                        sndStat = CAN0.sendMsgBuf(myTxID, 0, 8, myTxData);
                        SendStatResult(sndStat);
                    }                                 
                }
                break;

            case 0x5E7:
                    // tells you to send polls from 1AA (dash) for position data.
                    Send1AAMsg = true;
                break;
               
            default:
            break;
        }      
        
    }
}


void debugDataArray(byte dataArray[], int _len)
{
    for(byte i = 0; i< _len; i++)
    {
        sprintf(msgString, " 0x%.2X", dataArray[i]);
        Serial.print(msgString);
    }
    Serial.print(" ");
}

void SendStatResult(int _stat)
{
    if(_stat == CAN_OK)
    {
        Serial.println(" (SendOK!)");
    }
    else if (_stat == CAN_SENDMSGTIMEOUT)
    {
        Serial.println("  (Send Msg Timeout 7)");
    }
    else if (_stat == CAN_GETTXBFTIMEOUT)
    {
        Serial.println("  (TX Buffer timeout 6)");       
    } 
    else 
    {        
        Serial.print("  (Error-");
        Serial.print(sndStat);
        Serial.println(")");
    }      
}
