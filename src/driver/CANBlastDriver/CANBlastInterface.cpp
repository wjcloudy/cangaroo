/*

  Copyright (c) 2022 Ethan Zonca

  This file is part of cangaroo.

  cangaroo is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  cangaroo is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with cangaroo.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "CANBlastInterface.h"

#include <core/Backend.h>
#include <core/MeasurementInterface.h>
#include <core/CanMessage.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <QString>
#include <QStringList>
#include <QProcess>
#include <QThread>
#include <QTimer>
#include <QNetworkDatagram>
#include <linux/can.h>


CANBlastInterface::CANBlastInterface(CANBlastDriver *driver, int index, QString name, bool fd_support)
  : CanInterface((CanDriver *)driver),
	_idx(index),
    _isOpen(false),
    _name(name),
    _ts_mode(ts_mode_SIOCSHWTSTAMP),
    _socket(NULL)
{
    // Set defaults
    _settings.setBitrate(500000);
    _settings.setSamplePoint(875);

    _config.supports_canfd = fd_support;

    _heartbeat_timer = new QTimer(this);
    connect(_heartbeat_timer, &QTimer::timeout, this, &CANBlastInterface::udpHeartbeat);

}

CANBlastInterface::~CANBlastInterface() {
}

QString CANBlastInterface::getDetailsStr() const {
    if(_config.supports_canfd)
    {
        return "CANBlast with CANFD support";
    }
    else
    {
        return "CANBlast with standard CAN support";
    }
}

QString CANBlastInterface::getName() const {
	return _name;
}

void CANBlastInterface::setName(QString name) {
    _name = name;
}

QList<CanTiming> CANBlastInterface::getAvailableBitrates()
{
    QList<CanTiming> retval;
    QList<unsigned> bitrates({10000, 20000, 50000, 83333, 100000, 125000, 250000, 500000, 800000, 1000000});
    QList<unsigned> bitrates_fd({0, 2000000, 5000000});

    QList<unsigned> samplePoints({875});

    unsigned i=0;
    foreach (unsigned br, bitrates) {
        foreach(unsigned br_fd, bitrates_fd) {
            foreach (unsigned sp, samplePoints) {
                retval << CanTiming(i++, br, br_fd, sp);
            }
        }
    }

    return retval;
}


void CANBlastInterface::applyConfig(const MeasurementInterface &mi)
{
    // Save settings for port configuration
    _settings = mi;
}

bool CANBlastInterface::updateStatus()
{

}

bool CANBlastInterface::readConfig()
{

}

bool CANBlastInterface::readConfigFromLink(rtnl_link *link)
{

}

bool CANBlastInterface::supportsTimingConfiguration()
{
    return _config.supports_timing;
}

bool CANBlastInterface::supportsCanFD()
{
    return _config.supports_canfd;
}

bool CANBlastInterface::supportsTripleSampling()
{
    return false;
}

unsigned CANBlastInterface::getBitrate() {

}

uint32_t CANBlastInterface::getCapabilities()
{
    uint32_t retval =
        CanInterface::capability_config_os |
        CanInterface::capability_listen_only |
        CanInterface::capability_auto_restart;

    if (supportsCanFD()) {
        retval |= CanInterface::capability_canfd;
    }

    if (supportsTripleSampling()) {
        retval |= CanInterface::capability_triple_sampling;
    }

    return retval;
}

bool CANBlastInterface::updateStatistics()
{
    return updateStatus();
}

uint32_t CANBlastInterface::getState()
{
    /*
    switch (_status.can_state) {
        case CAN_STATE_ERROR_ACTIVE: return state_ok;
        case CAN_STATE_ERROR_WARNING: return state_warning;
        case CAN_STATE_ERROR_PASSIVE: return state_passive;
        case CAN_STATE_BUS_OFF: return state_bus_off;
        case CAN_STATE_STOPPED: return state_stopped;
        default: return state_unknown;
    }*/
}

int CANBlastInterface::getNumRxFrames()
{
    return _status.rx_count;
}

int CANBlastInterface::getNumRxErrors()
{
    return _status.rx_errors;
}

int CANBlastInterface::getNumTxFrames()
{
    return _status.tx_count;
}

int CANBlastInterface::getNumTxErrors()
{
    return _status.tx_errors;
}

int CANBlastInterface::getNumRxOverruns()
{
    return _status.rx_overruns;
}

int CANBlastInterface::getNumTxDropped()
{
    return _status.tx_dropped;
}

int CANBlastInterface::getIfIndex() {
    return _idx;
}

const char *CANBlastInterface::cname()
{
    return _name.toStdString().c_str();
}

void CANBlastInterface::open()
{
    _socket = new QUdpSocket(this);
    if(_socket->bind(QHostAddress::LocalHost, 20001))
    {
        _heartbeat_timer->start(500);
        _isOpen = true;
    }
    else
    {
        perror("CANBlaster Bind Failed!");
    }
    //connect(_socket, SIGNAL(readyRead()), this, SLOT(udpRead()));



}

void CANBlastInterface::close()
{
    _heartbeat_timer->stop();
    _isOpen = false;
}

bool CANBlastInterface::isOpen()
{
    return _isOpen;
}

void CANBlastInterface::sendMessage(const CanMessage &msg) {


}

void CANBlastInterface::udpRead()
{
    // when data comes in
    QByteArray buffer;
    buffer.resize(_socket->pendingDatagramSize());

    QHostAddress sender;
    quint16 senderPort;

    // qint64 QUdpSocket::readDatagram(char * data, qint64 maxSize,
    //                 QHostAddress * address = 0, quint16 * port = 0)
    // Receives a datagram no larger than maxSize bytes and stores it in data.
    // The sender's host address and port is stored in *address and *port
    // (unless the pointers are 0).

    _socket->readDatagram(buffer.data(), buffer.size(),
                   &sender, &senderPort);

    qDebug() << "Message from: " << sender.toString();
    qDebug() << "Message port: " << senderPort;
    qDebug() << "Message: " << buffer;
    fprintf(stderr, "Got some data\r\n");

}

void CANBlastInterface::udpHeartbeat()
{
    if(_isOpen)
    {
        QByteArray Data;
        Data.append("Heartbeat");
        _socket->writeDatagram(Data, QHostAddress::LocalHost, 20002);
    }
}

bool CANBlastInterface::readMessage(CanMessage &msg, unsigned int timeout_ms)
{

    // NOTE: This only works with standard CAN frames right now!

    // Don't saturate the thread. Read the buffer every 1ms.
    QThread().msleep(1);

    // Process all pending datagrams
    while (_socket->hasPendingDatagrams())
    {
        can_frame frame;
        QHostAddress address;
        quint16 port;
        int res = _socket->readDatagram((char*)&frame, sizeof(can_frame), &address, &port);

        // TODO: Read all bytes... to CANFD_MTU max
//        if (nbytes == CANFD_MTU) {
//                printf("got CAN FD frame with length %d\n", cfd.len);
//                /* cfd.flags contains valid data */
//        } else if (nbytes == CAN_MTU) {
//                printf("got Classical CAN frame with length %d\n", cfd.len);
//                /* cfd.flags is undefined */
//        } else {
//                fprintf(stderr, "read: invalid CAN(FD) frame\n");
//                return 1;
//        }

        if(res > 0)
        {
            // Set timestamp to current time
            struct timeval tv;
            gettimeofday(&tv,NULL);
            msg.setTimestamp(tv);

            msg.setInterfaceId(getId());
            msg.setId(frame.can_id & CAN_ERR_MASK);
            msg.setBRS(false);
            msg.setErrorFrame(frame.can_id & CAN_ERR_FLAG);
            msg.setExtended(frame.can_id & CAN_EFF_FLAG);
            msg.setRTR(frame.can_id & CAN_RTR_FLAG);
            msg.setLength(frame.len);


            for(int i=0; i<frame.len && i<CAN_MAX_DLEN; i++)
            {
                msg.setDataAt(i, frame.data[i]);
            }

            return true;
        }
    }
    return false;

}

