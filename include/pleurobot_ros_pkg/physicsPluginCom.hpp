// ========================================================================================================================================================================================

// File:          physicsPluginCom.hpp
// Date:          07-01-2014
// Description:   functions to communicate between controller and physics plugin
// Author:        Robin Thandiackal
// Modifications: -

// ========================================================================================================================================================================================

#ifndef PHYSICSPLUGINCOM_HPP
#define PHYSICSPLUGINCOM_HPP

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <iostream>

#include "physicsPluginComTypDef.h"

// CLASS DEFINITION ---------------------------------------------------

class connection
{
	public:
	webots::Emitter *emitter;
	webots::Receiver *receiver;
	connection();
	bool communicate2physics(webots::Emitter *, webots::Receiver *, c2p_packet, p2c_packet &);
	~connection();

	private:
	bool send(webots::Emitter *, c2p_packet);
	bool receive(webots::Receiver *, p2c_packet &);

};

#endif
