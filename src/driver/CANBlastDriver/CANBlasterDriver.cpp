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


#include "CANBlasterDriver.h"
#include "CANBlasterInterface.h"
#include <core/Backend.h>
#include <driver/GenericCanSetupPage.h>

#include <errno.h>
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <QCoreApplication>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>


CANBlasterDriver::CANBlasterDriver(Backend &backend)
  : CanDriver(backend),
    setupPage(new GenericCanSetupPage())
{
    QObject::connect(&backend, SIGNAL(onSetupDialogCreated(SetupDialog&)), setupPage, SLOT(onSetupDialogCreated(SetupDialog&)));
}

CANBlasterDriver::~CANBlasterDriver() {
}

bool CANBlasterDriver::update() {

    // TODO: Listen for multicast packets for discovery of canblaster servers

    // Testing
    int interface_cnt = 0;
    CANBlasterInterface *intf = createOrUpdateInterface(interface_cnt++, "Test CANblaster", false);

    return true;
}

QString CANBlasterDriver::getName() {
    return "CANblaster";
}



CANBlasterInterface *CANBlasterDriver::createOrUpdateInterface(int index, QString name, bool fd_support) {

    foreach (CanInterface *intf, getInterfaces()) {
        CANBlasterInterface *scif = dynamic_cast<CANBlasterInterface*>(intf);
		if (scif->getIfIndex() == index) {
			scif->setName(name);
            return scif;
		}
	}


    CANBlasterInterface *scif = new CANBlasterInterface(this, index, name, fd_support);
    addInterface(scif);
    return scif;
}
