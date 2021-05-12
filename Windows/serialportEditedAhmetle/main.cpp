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

void moveMouse(int movex, int movey)
{
    POINT p;
    GetCursorPos(&p);
    SetCursorPos(movex+p.x, movey+p.y);
    //Sleep(1); //çalışmazsa bir dene
}


bool connectPrinter(char *portName)
{

    //Connects to the port.
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
            //If port is in use by another program or did not closed previous process.
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
            // Set Serial Port specifications.
            dcbSerialParams.BaudRate = CBR_115200;
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
                //Wait 2s as the arduino board will be reseting
                return true;
                Sleep(ARDUINO_WAIT_TIME);
            }
        }
    }
}

int readData(char *buffer, unsigned int nbChar)
{

    DWORD bytesRead;
    DWORD dwCommModemStatus;

    //Set the type of data to be caught.(RXCHAR -> Data available on RX pin.)
    SetCommMask(hSerial, EV_RXCHAR);

    //printf("HELLO");
    while (hSerial != INVALID_HANDLE_VALUE)
    {
        // Wait for an event to occur for the port.
        WaitCommEvent(hSerial, &dwCommModemStatus, 0);

        if (dwCommModemStatus & EV_RXCHAR)
        {
           // a b c 9999
            do
            {
                short int message[3];

                ReadFile(hSerial, &message, sizeof(message), &bytesRead, 0);
                //printf("%i %i %i\n", message[0], message[1], message[2]);
                moveMouse(message[0],-message[1]);
                

            } while (bytesRead > 0);
            
        }
    }
    //}
    //If nothing has been read, or that an error was detected return -1
    return -1;
}

int main()
{

    char *buffer = new char[1];
    int nbChar = 1;
    string str = "COM7"; // Use your port number to connect

    char *writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';

    connectPrinter(writable);
    delete[] writable;
    readData(buffer, nbChar);
    return 0;
}

/*

void gotoXY(int x, int y) 
{ 
CursorPosition.X = x; 
CursorPosition.Y = y; 
SetConsoleCursorPosition(console,CursorPosition); 
}

*/