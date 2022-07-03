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


#include "SLCANDriver.h"
#include "SLCANInterface.h"
#include <core/Backend.h>
#include <driver/GenericCanSetupPage.h>

#include <errno.h>
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

//
#include <QCoreApplication>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

SLCANDriver::SLCANDriver(Backend &backend)
  : CanDriver(backend),
    setupPage(new GenericCanSetupPage())
{
    QObject::connect(&backend, SIGNAL(onSetupDialogCreated(SetupDialog&)), setupPage, SLOT(onSetupDialogCreated(SetupDialog&)));
}

SLCANDriver::~SLCANDriver() {
}

bool SLCANDriver::update() {

    /*
    struct nl_sock *sock = nl_socket_alloc();
    struct nl_cache *cache;

    nl_connect(sock, NETLINK_ROUTE);
    int result = rtnl_link_alloc_cache(sock, AF_UNSPEC, &cache);

    if (result < 0) {
        log_error(QString("Could not access netlink device list: %1").arg(result));
    } else {

        for (struct nl_object *obj = nl_cache_get_first(cache); obj!=0; obj=nl_cache_get_next(obj)) {
            struct rtnl_link *link = (struct rtnl_link *)obj;

            if (rtnl_link_get_arptype(link)==ARPHRD_CAN) {
                SocketCanInterface *intf = createOrUpdateInterface(rtnl_link_get_ifindex(link), QString(rtnl_link_get_name(link)));
                intf->readConfigFromLink(link);
            }
        }
    }

    nl_cache_free(cache);
    nl_close(sock);
    nl_socket_free(sock);
    */

    // TODO: Cross platform enumerate all serial ports available, add interfaces for each

    //intf->readConfigFromLink(link);

    int interface_cnt = 0;

    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        fprintf(stderr, "Name : %s \r\n",  info.portName().toStdString().c_str());
        fprintf(stderr, "Description : %s \r\n", info.description().toStdString().c_str());
        fprintf(stderr, "Manufacturer: %s \r\n", info.manufacturer().toStdString().c_str());

        if(info.description().contains("CANable"))
        {

            perror("This is a CANable device!");
            SLCANInterface *intf = createOrUpdateInterface(interface_cnt, info.portName());
            interface_cnt++;
        }
        else
        {
            perror("This is not a CANable device!");
        }
    }

    return true;
}

QString SLCANDriver::getName() {
    return "SLCAN";
}



SLCANInterface *SLCANDriver::createOrUpdateInterface(int index, QString name) {

    foreach (CanInterface *intf, getInterfaces()) {
        SLCANInterface *scif = dynamic_cast<SLCANInterface*>(intf);
		if (scif->getIfIndex() == index) {
			scif->setName(name);
            return scif;
		}
	}


    //SLCANInterface *scif = new SLCANInterface(this, index, name);
    SLCANInterface *scif = new SLCANInterface(this, index, "/dev/pts/3");
    addInterface(scif);
    return scif;
}
