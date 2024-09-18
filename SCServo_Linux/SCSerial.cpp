/*
 * SCSerial.cpp
 * Hardware interface layer program for serial servos
 * Date: 2022.3.29
 * Author:
 */

#include "SCSerial.h"

SCSerial::SCSerial()
{
    IOTimeOut = 100;
    fd = -1;
    txBufLen = 0;
}

SCSerial::SCSerial(u8 End) : SCS(End)
{
    IOTimeOut = 100;
    fd = -1;
    txBufLen = 0;
}

SCSerial::SCSerial(u8 End, u8 Level) : SCS(End, Level)
{
    IOTimeOut = 100;
    fd = -1;
    txBufLen = 0;
}

bool SCSerial::begin(int baudRate, const char *serialPort)
{
    if (fd != -1)
    {
        close(fd);
        fd = -1;
    }

    if (serialPort == NULL)
        return false;

    fd = open(serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        perror("open:");
        return false;
    }

    fcntl(fd, F_SETFL, FNDELAY);
    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);

    speed_t CR_BAUDRATE;
    switch (baudRate)
    {
    case 9600:
        CR_BAUDRATE = B9600;
        break;
    case 19200:
        CR_BAUDRATE = B19200;
        break;
    case 38400:
        CR_BAUDRATE = B38400;
        break;
    case 57600:
        CR_BAUDRATE = B57600;
        break;
    case 115200:
        CR_BAUDRATE = B115200;
        break;
    case 500000:
        CR_BAUDRATE = B500000;
        break;
    case 1000000:
        CR_BAUDRATE = B1000000;
        break;
    default:
        CR_BAUDRATE = B115200;
        break;
    }

    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);

    // Set serial port settings: 8N1 (8 data bits, no parity, 1 stop bit)
    curopt.c_cflag &= ~PARENB;
    curopt.c_cflag &= ~CSTOPB;
    curopt.c_cflag &= ~CSIZE;
    curopt.c_cflag |= CS8;
    curopt.c_cflag |= CREAD;
    curopt.c_cflag |= CLOCAL; // Disable modem status check
    cfmakeraw(&curopt);       // Set raw mode
    curopt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    if (tcsetattr(fd, TCSANOW, &curopt) == 0)
    {
        return true;
    }
    else
    {
        perror("tcsetattr:");
        return false;
    }
}

int SCSerial::setBaudRate(int baudRate)
{
    if (fd == -1)
    {
        return -1;
    }

    tcgetattr(fd, &orgopt);
    tcgetattr(fd, &curopt);

    speed_t CR_BAUDRATE;
    switch (baudRate)
    {
    case 9600:
        CR_BAUDRATE = B9600;
        break;
    case 19200:
        CR_BAUDRATE = B19200;
        break;
    case 38400:
        CR_BAUDRATE = B38400;
        break;
    case 57600:
        CR_BAUDRATE = B57600;
        break;
    case 115200:
        CR_BAUDRATE = B115200;
        break;
    case 230400:
        CR_BAUDRATE = B230400;
        break;
    case 500000:
        CR_BAUDRATE = B500000;
        break;
    default:
        break;
    }

    cfsetispeed(&curopt, CR_BAUDRATE);
    cfsetospeed(&curopt, CR_BAUDRATE);

    return 1;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
    int fs_sel;
    fd_set fs_read;
    int rvLen = 0;
    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    time.tv_sec = 0;
    time.tv_usec = IOTimeOut * 1000;

    // Use select to implement multiplexed communication
    while (1)
    {
        fs_sel = select(fd + 1, &fs_read, NULL, NULL, &time);
        if (fs_sel)
        {
            rvLen += read(fd, nDat + rvLen, nLen - rvLen);
            if (rvLen < nLen)
            {
                continue;
            }
            else
            {
                return rvLen;
            }
        }
        else
        {
            return rvLen;
        }
    }
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
    while (nLen--)
    {
        txBuf[txBufLen++] = *nDat++;
    }
    return txBufLen;
}

int SCSerial::writeSCS(unsigned char bDat)
{
    txBuf[txBufLen++] = bDat;
    return txBufLen;
}

void SCSerial::rFlushSCS()
{
    tcflush(fd, TCIFLUSH);
}

void SCSerial::wFlushSCS()
{
    if (txBufLen)
    {
        txBufLen = write(fd, txBuf, txBufLen);
        txBufLen = 0;
    }
}

void SCSerial::end()
{
    fd = -1;
    close(fd);
}
