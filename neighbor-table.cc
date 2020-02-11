/*
 * neighbor-table.cc
 *
 *  Created on: Jul 11, 2017
 *      Author: Saurabh Kumar
 */
#include "neighbor-table.h"
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/pointer.h>
#include <ns3/node.h>
#include <ns3/object-base.h>
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Neighbortable");
NS_OBJECT_ENSURE_REGISTERED(Neighbortable);

TypeId Neighbortable::GetTypeId(){
	static TypeId tid =
			TypeId("NS3::Neighbortable").SetParent<ObjectBase>().SetGroupName("Wave").AddConstructor<
					Neighbortable>();

	return tid;
}
Neighbortable::Neighbortable():m_rssi(0), m_next(NULL){

}




}  // namespace ns3



