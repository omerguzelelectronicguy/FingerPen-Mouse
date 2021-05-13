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
    bool pressed=0;
    while (hSerial != INVALID_HANDLE_VALUE)
    {
        // Wait for an event to occur for the port.
        WaitCommEvent(hSerial, &dwCommModemStatus, 0);

        if (dwCommModemStatus & EV_RXCHAR)
        {
            do
            {
                short int message[4];
<<<<<<< Updated upstream
=======
                // container for coming message from arduino
                // 3 messages include x y z velocity vector.
>>>>>>> Stashed changes

                ReadFile(hSerial, &message, sizeof(message), &bytesRead, 0);
                // the size of (3*short int) is read, then written on message array
                // change the bytesRead according to the resting data.
                //printf("%i %i %i\n", message[0], message[1], message[2]);
<<<<<<< Updated upstream
                moveMouse(message[0],-message[1]);
                if(message[3] == 1){
                    mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
                }
                mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                

=======
                //printf("%i\n",message[3]);
                moveMouse(message[0]/10,-message[1]/10);
                // call the function to move cursor.
                if(message[3] == 1){
                    mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
                    pressed=1;
                }else{
                    if (pressed){
                        pressed=0;
                        mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                    }
                }
>>>>>>> Stashed changes
            } while (bytesRead > 0);
            
        }
    }
    //If nothing has been read, or that an error was detected return -1
    return -1;
}

int main()
{

    char *buffer = new char[1];
    int nbChar = 1;
<<<<<<< Updated upstream
    string str = "COM8"; // Use your port number to connect
=======
    string str = "COM7"; // Use your port number to connect
    // This COM7 might be changed. At the end it must be automated for arduino
>>>>>>> Stashed changes

    char *writable = new char[str.size() + 1];
    std::copy(str.begin(), str.end(), writable);
    writable[str.size()] = '\0';

    connectPrinter(writable);
    delete[] writable;
    readData(buffer, nbChar);
    return 0;
<<<<<<< Updated upstream
}

/*

void gotoXY(int x, int y) 
{ 
CursorPosition.X = x; 
CursorPosition.Y = y; 
SetConsoleCursorPosition(console,CursorPosition); 
}

*/
=======
}
>>>>>>> Stashed changes
