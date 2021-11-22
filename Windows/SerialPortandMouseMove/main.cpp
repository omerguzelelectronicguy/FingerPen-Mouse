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
            dcbSerialParams.BaudRate = 250000;
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
            while (1)
            {
                byte start_message;
                ReadFile(hSerial, &start_message, sizeof(start_message), &bytesRead, 0);
                if (start_message == 85)
                {
                    printf("started");
                    break;
                }
                printf("start comment waiting\n");
            }

            do
            {
                // short int message[4];
                short int message[4];
                // container for coming message from arduino
                // 3 messages include x y z velocity vector.

                ReadFile(hSerial, &message, sizeof(message), &bytesRead, 0);

                // the size of (3*short int) is read, then written on message array
                // change the bytesRead according to the resting data.
                // printf("%i\t%i\t%i\t%i\n", message[0],message[1],message[2],message[3]);
                // printf("%i\n",message[3]);
                // moveMouse(float(-message[2]*100),float(-message[0]*100));
                mouse_event(message[0], message[1], message[2], 0, 0);

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
    string str = "COM7"; // Use your port number to connect
    // This COM7 might be changed. At the end it must be automated for arduino

    char *writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';

    connectPrinter(writable);
    delete[] writable;
    readData(buffer, nbChar);
    return 0;
}
