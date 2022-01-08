#include <windows.h>
#include <string>
#include <array>
#include <stdio.h>

#define ARDUINO_WAIT_TIME 2000
using namespace std;

string portNumber;
HANDLE hSerial;
bool connected;
COMSTAT status;

bool connectPrinter(char *portName)
{

    // Connects to the port.
    hSerial = CreateFileA(portName,
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          NULL,
                          OPEN_EXISTING,
                          FILE_ATTRIBUTE_NORMAL,
                          NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);
            return false;
        }
        else
        {
            // If port is in use by another program or did not closed previous process.
            printf("ERROR!!! \n");
            CloseHandle(hSerial);
            return false;
        }
    }
    else
    {

        DCB dcbSerialParams = {0};

        if (!GetCommState(hSerial, &dcbSerialParams))
        {
            printf("failed to get current serial parameters!");
            return false;
        }
        else
        {
            // Set Serial Port specifications.clear
            dcbSerialParams.BaudRate = 1000000 ;
            dcbSerialParams.ByteSize = 8;
            dcbSerialParams.StopBits = ONESTOPBIT;
            dcbSerialParams.Parity = NOPARITY;

            if (!SetCommState(hSerial, &dcbSerialParams))
            {
                printf("ALERT: Could not set Serial Port parameters");
                return false;
            }
            else
            {
                connected = true;
                printf("Connection Successful for :%s !!! \n", portName);
                // Wait 2s as the arduino board will be reseting
                return true;
                Sleep(ARDUINO_WAIT_TIME);
            }
        }
    }
}
byte x_sign = 1;
byte y_sign = 1;
const uint8_t compress_var[] {5,7,9,13,21,37,69,133,255};

int readData(char *buffer, unsigned int nbChar)
{

    DWORD bytesRead;
    DWORD dwCommModemStatus;

    // Set the type of data to be caught.(RXCHAR -> Data available on RX pin.)
    SetCommMask(hSerial, EV_RXCHAR);

    // printf("HELLO");
    bool pressed = 0;
    while (hSerial != INVALID_HANDLE_VALUE)
    {
        // Wait for an event to occur for the port.
        WaitCommEvent(hSerial, &dwCommModemStatus, 0);

        if (dwCommModemStatus & EV_RXCHAR)
        {
            int n = 0;
           while (n)
            {
                byte start_message;
                ReadFile(hSerial, &start_message, sizeof(start_message), &bytesRead, 0);
                if (start_message == 85)
                {
                    start_message = 0;
                    n--;
                    printf("n= %i\n",n);
                }else{
                    n = 4;
                }
                printf("start comment waiting\n");
            }

            do
            {
                // short int message[4];
                uint8_t message;
                // container for coming message from arduino
                // 3 messages include x y z velocity vector.

                ReadFile(hSerial, &message, sizeof(message), &bytesRead, 0);
                // while(message[0] != 1 && message[0] != 2 && message[0] != 4 && message[0] != 8 &&message[0] != 16 );
                // the size of (3*short int) is read, then written on message array
                // change the bytesRead according to the resting data.
                uint8_t dx = ((message << 4) & 0b11110000)>>4;
                uint8_t dy = message >> 4;
                printf("%i\t%i\n", x_sign ? compress_var[dx]:-compress_var[dx], y_sign ? compress_var[dy]: - compress_var[dy]);
                // printf("%i\n",message[3]);
                //if(message[0] == MOUSE_MOVED){
                if(dx == 14){
                    x_sign = ! x_sign;
                    dx = 0;
                }
                if (dy == 14){
                    y_sign = ! y_sign;
                    dy = 0;
                }
                mouse_event(MOUSE_MOVED,x_sign ? compress_var[dx]:-compress_var[dx], y_sign ? compress_var[dy]: - compress_var[dy], 0, 0);
                //}else if(message[0] == MOUSEEVENTF_LEFTDOWN || message[0] == MOUSEEVENTF_LEFTUP){
                   // mouse_event(message[0] ,0 ,0 , 0, 0);
                //}

            } while (bytesRead > 0);
        }
    }
    // If nothing has been read, or that an error was detected return -1
    return -1;
}

int main()
{

    char *buffer = new char[1];
    int nbChar = 1;
    string str = "COM9"; // Use your port number to connect
    // This COM7 might be changed. At the end it must be automated for arduino

    char *writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';

    connectPrinter(writable);
    delete[] writable;
    readData(buffer, nbChar);
    return 0;
}