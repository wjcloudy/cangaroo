/*

  Copyright (c) 2015, 2016 Hubert Denkmair

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

#include "SLCANInterface.h"

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
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/netlink.h>
#include <linux/sockios.h>
#include <termios.h>
#include <netlink/version.h>
#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

SLCANInterface::SLCANInterface(SLCANDriver *driver, int index, QString name)
  : CanInterface((CanDriver *)driver),
	_idx(index),
    _fd(0),
    _name(name),
    _ts_mode(ts_mode_SIOCSHWTSTAMP),
    _rxbuf_pos(0)
{
}

SLCANInterface::~SLCANInterface() {
}

QString SLCANInterface::getName() const {
	return _name;
}

void SLCANInterface::setName(QString name) {
    _name = name;
}

QList<CanTiming> SLCANInterface::getAvailableBitrates()
{
    QList<CanTiming> retval;
    QList<unsigned> bitrates({10000, 20000, 50000, 83333, 100000, 125000, 250000, 500000, 800000, 1000000});
    QList<unsigned> samplePoints({500, 625, 750, 875});

    unsigned i=0;
    foreach (unsigned br, bitrates) {
        foreach (unsigned sp, samplePoints) {
            retval << CanTiming(i++, br, 0, sp);
        }
    }

    return retval;
}

QString SLCANInterface::buildIpRouteCmd(const MeasurementInterface &mi)
{
    QStringList cmd;
    cmd.append("ip");
    cmd.append("link");
    cmd.append("set");
    cmd.append(getName());
    cmd.append("up");
    cmd.append("type");
    cmd.append("can");

    cmd.append("bitrate");
    cmd.append(QString().number(mi.bitrate()));
    cmd.append("sample-point");
    cmd.append(QString().number((float)mi.samplePoint()/1000.0, 'f', 3));

    if (mi.isCanFD()) {
        cmd.append("dbitrate");
        cmd.append(QString().number(mi.fdBitrate()));
        cmd.append("dsample-point");
        cmd.append(QString().number((float)mi.fdSamplePoint()/1000.0, 'f', 3));
        cmd.append("fd");
        cmd.append("on");
    }

    cmd.append("restart-ms");
    if (mi.doAutoRestart()) {
        cmd.append(QString().number(mi.autoRestartMs()));
    } else {
        cmd.append("0");
    }

    return cmd.join(' ');
}

QStringList SLCANInterface::buildCanIfConfigArgs(const MeasurementInterface &mi)
{
    QStringList args;
    args << "-d";
    args << "-i" << getName();
    args << "-b" << QString::number(mi.bitrate());
    args << "-p" << QString::number(mi.samplePoint());
    args << "-u";
    return args;
}


void SLCANInterface::applyConfig(const MeasurementInterface &mi)
{
    /*
    if (!mi.doConfigure()) {
        log_info(QString("interface %1 not managed by cangaroo, not touching configuration").arg(getName()));
        return;
    }

    log_info(QString("calling canifconfig to reconfigure interface %1").arg(getName()));
    QStringList sl = buildCanIfConfigArgs(mi);
    sl.prepend("canifconfig");
    log_info(sl.join(" "));

    QProcess canIfConfig;
    canIfConfig.start("canifconfig", buildCanIfConfigArgs(mi));
    if (!canIfConfig.waitForFinished()) {
        log_error(QString("timeout waiting for canifconfig"));
        return;
    }

    if (canIfConfig.exitStatus()!=QProcess::NormalExit) {
        log_error(QString("canifconfig crashed"));
        return;
    }

    if (canIfConfig.exitCode() != 0) {
        log_error(QString("canifconfig failed: ") + QString(canIfConfig.readAllStandardError()).trimmed());
        return;
    } */

}

#if (LIBNL_CURRENT<=216)
#warning we need at least libnl3 version 3.2.22 to be able to get link status via netlink
int rtnl_link_can_state(struct rtnl_link *link, uint32_t *state) {
    (void) link;
    (void) state;
    return -1;
}
#endif

bool SLCANInterface::updateStatus()
{
    /*
    bool retval = false;

    struct nl_sock *sock = nl_socket_alloc();
    struct nl_cache *cache;
    struct rtnl_link *link;
    uint32_t state;

    _status.can_state = state_unknown;

    nl_connect(sock, NETLINK_ROUTE);
    if (rtnl_link_alloc_cache(sock, AF_UNSPEC, &cache) >= 0) {
        if (rtnl_link_get_kernel(sock, _idx, 0, &link) == 0) {

            _status.rx_count = rtnl_link_get_stat(link, RTNL_LINK_RX_PACKETS);
            _status.rx_overruns = rtnl_link_get_stat(link, RTNL_LINK_RX_OVER_ERR);
            _status.tx_count = rtnl_link_get_stat(link, RTNL_LINK_TX_PACKETS);
            _status.tx_dropped = rtnl_link_get_stat(link, RTNL_LINK_TX_DROPPED);

            if (rtnl_link_is_can(link)) {
                if (rtnl_link_can_state(link, &state)==0) {
                    _status.can_state = state;
                }
                _status.rx_errors = rtnl_link_can_berr_rx(link);
                _status.tx_errors = rtnl_link_can_berr_tx(link);
            } else {
                _status.rx_errors = 0;
                _status.tx_errors = 0;
            }
            retval = true;
        }
    }

    nl_cache_free(cache);
    nl_close(sock);
    nl_socket_free(sock);

    return retval;
    */
}

bool SLCANInterface::readConfig()
{
    /*
    bool retval = false;

    struct nl_sock *sock = nl_socket_alloc();
    struct nl_cache *cache;
    struct rtnl_link *link;

    nl_connect(sock, NETLINK_ROUTE);
    int result = rtnl_link_alloc_cache(sock, AF_UNSPEC, &cache);

    if (result>=0) {
        if (rtnl_link_get_kernel(sock, _idx, 0, &link) == 0) {
            retval = readConfigFromLink(link);
        }
    }

    nl_cache_free(cache);
    nl_close(sock);
    nl_socket_free(sock);

    return retval;
    */
}

bool SLCANInterface::readConfigFromLink(rtnl_link *link)
{
    /*
    _config.state = state_unknown;
    _config.supports_canfd = (rtnl_link_get_mtu(link)==72);
    _config.supports_timing = rtnl_link_is_can(link);
    if (_config.supports_timing) {
        rtnl_link_can_freq(link, &_config.base_freq);
        rtnl_link_can_get_ctrlmode(link, &_config.ctrl_mode);
        rtnl_link_can_get_bittiming(link, &_config.bit_timing);
        rtnl_link_can_get_sample_point(link, &_config.sample_point);
        rtnl_link_can_get_restart_ms(link, &_config.restart_ms);
    } else {
        // maybe a vcan interface?
    }
    return true; */
}

bool SLCANInterface::supportsTimingConfiguration()
{
    return _config.supports_timing;
}

bool SLCANInterface::supportsCanFD()
{
    return _config.supports_canfd;
}

bool SLCANInterface::supportsTripleSampling()
{
    return false;
}

unsigned SLCANInterface::getBitrate() {
    if (readConfig()) {
        return _config.bit_timing.bitrate;
    } else {
        return 0;
    }
}

uint32_t SLCANInterface::getCapabilities()
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

bool SLCANInterface::updateStatistics()
{
    return updateStatus();
}

uint32_t SLCANInterface::getState()
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

int SLCANInterface::getNumRxFrames()
{
    return _status.rx_count;
}

int SLCANInterface::getNumRxErrors()
{
    return _status.rx_errors;
}

int SLCANInterface::getNumTxFrames()
{
    return _status.tx_count;
}

int SLCANInterface::getNumTxErrors()
{
    return _status.tx_errors;
}

int SLCANInterface::getNumRxOverruns()
{
    return _status.rx_overruns;
}

int SLCANInterface::getNumTxDropped()
{
    return _status.tx_dropped;
}

int SLCANInterface::getIfIndex() {
    return _idx;
}

const char *SLCANInterface::cname()
{
    return _name.toStdString().c_str();
}

void SLCANInterface::open()
{
    _fd = ::open(_name.toStdString().c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if(_fd < 0) {
        perror("Error while opening serial port");
	}


    termios termio;
    tcflag_t baud_specifier;

    //reset device state...
    memset (&termio, 0, sizeof (termios));

    //configure device state...
    // TODO add baud rate spec
    termio.c_cflag = CS8 | CLOCAL | CREAD;

    termio.c_iflag = IGNPAR;
    termio.c_oflag = 0;
    termio.c_lflag = 0;

    //com port timing, no wait between characters and read unblocks as soon as there is a character
    termio.c_cc[VTIME]    = 0;
    termio.c_cc[VMIN]     = 0;


    if (tcflush (_fd, TCIOFLUSH) == -1) {
        perror("Error flushing serial port");
    }

    //apply our device config...
    if (tcsetattr (_fd, TCSANOW, &termio) == -1)
    {
        perror("Error setting serial config");
    }


}

void SLCANInterface::close() {
    ::close(_fd);
}

void SLCANInterface::sendMessage(const CanMessage &msg) {


    // SLCAN_MTU
    char buf[SLCAN_MTU] = {0};

    uint8_t msg_idx = 0;

    // Add character for frame type
    if (msg.isRTR()) {
        buf[msg_idx] = 'r';
    }
    else
    {
        buf[msg_idx] = 't';
    }



    // Assume standard identifier
    uint8_t id_len = SLCAN_STD_ID_LEN;
    uint32_t tmp = msg.getId();

    // Check if extended
    if (msg.isExtended())
    {
        // Convert first char to upper case for extended frame
        buf[msg_idx] -= 32;
        id_len = SLCAN_EXT_ID_LEN;
    }
    msg_idx++;

    // Add identifier to buffer
    for(uint8_t j = id_len; j > 0; j--)
    {
        // Add nibble to buffer
        buf[j] = (tmp & 0xF);
        tmp = tmp >> 4;
        msg_idx++;
    }

    // Sanity check length
    int8_t bytes = msg.getLength();

    // TODO: CANFD
    if(bytes > 8)
        bytes = 8;
    // Check bytes value
//    if(bytes < 0)
//        return -1;
//    if(bytes > 64)
//        return -1;


    // Add DLC to buffer
    buf[msg_idx++] = bytes;

    // Add data bytes
    for (uint8_t j = 0; j < bytes; j++)
    {
        buf[msg_idx++] = (msg.getByte(j) >> 4);
        buf[msg_idx++] = (msg.getByte(j) & 0x0F);
    }

    // Convert to ASCII (2nd character to end)
    for (uint8_t j = 1; j < msg_idx; j++)
    {
        if (buf[j] < 0xA) {
            buf[j] += 0x30;
        } else {
            buf[j] += 0x37;
        }
    }

    // Add CR for slcan EOL
    buf[msg_idx++] = '\r';

    // Write string to serial device
    write (_fd, buf, msg_idx);

}

bool SLCANInterface::readMessage(CanMessage &msg, unsigned int timeout_ms) {

    struct timespec ts_rcv;
    struct timeval tv_rcv;
    struct timeval timeout;
    //struct ifreq hwtstamp;
    fd_set fdset;

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = 1000 * (timeout_ms % 1000);

    FD_ZERO(&fdset);
    FD_SET(_fd, &fdset);


    int rv = select(_fd+1, &fdset, NULL, NULL, &timeout);

    if (rv>0) {

        // Attempt read
        char inbyte[1] = {0};
        size_t bytes_read = read(_fd, inbyte, 1);
        if (bytes_read < 0) {
            return false;
        }

        // If end of message, we should have a full string ready to parse in _rxbuf
        if(inbyte[0] == '\r')
        {
            // Defaults
            msg.setErrorFrame(0);
            msg.setInterfaceId(getId());
            msg.setId(0);

            // Convert from ASCII (2nd character to end)
            for (int i = 1; i < _rxbuf_pos; i++)
            {
                // Lowercase letters
                if(_rxbuf[i] >= 'a')
                    _rxbuf[i] = _rxbuf[i] - 'a' + 10;
                // Uppercase letters
                else if(_rxbuf[i] >= 'A')
                    _rxbuf[i] = _rxbuf[i] - 'A' + 10;
                // Numbers
                else
                    _rxbuf[i] = _rxbuf[i] - '0';
            }


            // Handle each incoming command
            switch(_rxbuf[0])
            {

                // Transmit data frame command
                case 'T':
                    msg.setExtended(1);
                    break;
                case 't':
                    msg.setExtended(0);
                    break;

                // Transmit remote frame command
                case 'r':
                    msg.setExtended(0);
                    msg.setRTR(1);
                    break;
                case 'R':
                    msg.setExtended(1);
                    msg.setRTR(1);
                    break;


                // Invalid command
                default:
                    return false;
            }

            // Start parsing at second byte (skip command byte)
            uint8_t parse_loc = 1;

            // Default to standard id len
            uint8_t id_len = SLCAN_STD_ID_LEN;

            // Update length if message is extended ID
            if(msg.isExtended())
                id_len = SLCAN_EXT_ID_LEN;

            uint32_t id_tmp = 0;

            // Iterate through ID bytes
            while(parse_loc <= id_len)
            {
                id_tmp *= 16;
                id_tmp += _rxbuf[parse_loc++];
            }


            msg.setId(id_tmp);

            // Attempt to parse DLC and check sanity
            uint8_t dlc_code_raw = _rxbuf[parse_loc++];

           /* If dlc is too long for an FD frame
            if(frame_header.FDFormat == FDCAN_FD_CAN && dlc_code_raw > 0xF)
            {
                return -1;
            }
            if(frame_header.FDFormat == FDCAN_CLASSIC_CAN && dlc_code_raw > 0x8)
            {
                return -1;
            }*/

            msg.setLength(dlc_code_raw);

            // Calculate number of bytes we expect in the message
            int8_t bytes_in_msg = dlc_code_raw;

            if(bytes_in_msg < 0)
                return false;
            if(bytes_in_msg > 64)
                return false;

            // Parse data
            // TODO: Guard against walking off the end of the string!
            for (uint8_t i = 0; i < bytes_in_msg; i++)
            {
                msg.setByte(i,  (_rxbuf[parse_loc] << 4) + _rxbuf[parse_loc+1]);
                parse_loc += 2;
            }

            // Reset buffer
            _rxbuf_pos = 0;
            _rxbuf[0] = '\0';
            return true;
        }
        // Got  a char but no newline, save for later
        else if(_rxbuf_pos < SLCAN_MTU*2-1) // Sloppy
        {
            _rxbuf[_rxbuf_pos++] = inbyte[0];
            return false;
        }
        else
        {
            perror("Buffer overflow on slcan rx");
            _rxbuf_pos = 0;
            _rxbuf[0] = '\0';
            return false;
        }



        // FIXME
        if (_ts_mode == ts_mode_SIOCSHWTSTAMP) {
            // TODO implement me
            _ts_mode = ts_mode_SIOCGSTAMPNS;
        }

        if (_ts_mode==ts_mode_SIOCGSTAMPNS) {
            if (ioctl(_fd, SIOCGSTAMPNS, &ts_rcv) == 0) {
                msg.setTimestamp(ts_rcv.tv_sec, ts_rcv.tv_nsec/1000);
            } else {
                _ts_mode = ts_mode_SIOCGSTAMP;
            }
        }

        if (_ts_mode==ts_mode_SIOCGSTAMP) {
            ioctl(_fd, SIOCGSTAMP, &tv_rcv);
            msg.setTimestamp(tv_rcv.tv_sec, tv_rcv.tv_usec);
        }
        return true;
    } else {
        return false;
    }
}
