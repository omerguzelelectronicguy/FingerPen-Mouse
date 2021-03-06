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

void moveMouse(int movex, int movey) // Created function to move cursor.
{
    POINT p;
    GetCursorPos(&p);
    // it is to determine the cursor position
    SetCursorPos(movex + p.x, movey + p.y);
    // the coming values from arduino added to the current position values.
    // Sleep(1); //çalışmazsa bir dene
}

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
            float prex = 0, prey = 0;
            do
            {
                // short int message[4];
                float message[4];
                // container for coming message from arduino
                // 3 messages include x y z velocity vector.

                ReadFile(hSerial, &message, sizeof(message), &bytesRead, 0);
                // the size of (3*short int) is read, then written on message array
                // change the bytesRead according to the resting data.
                // printf("%f\t%f\t%f\t%f\n", message[0],message[1],message[2],message[3]);
                // printf("%i\n",message[3]);
                // moveMouse(float(-message[2]*100),float(-message[0]*100));
                // call the function to move cursor.
                // This function will move cursor directly to stated position.
                // SetCursorPos(float(1300-message[2]*2000),float(450-message[0]*1500));
                int dx = int(2000 * (prex - message[2]));
                int dy = int(1500 * (prey - message[0]));
                mouse_event(1, dx, dy, 0, 0);
                // printf("%i\t%i\t", dx,dy);
                prex = message[2];
                prey = message[0];

                if (int(message[3]) == 1)
                {
                    if (!pressed)
                    {
                        mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
                         pressed = 1;
                    }
                }
                else
                {
                    if (pressed)
                    {
                        pressed = 0;
                        mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                    }
                }
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
