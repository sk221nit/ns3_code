/*
 * neighbor-table.h
 *
 *  Created on: Jul 11, 2017
 *      Author: Saurabh Kumar
 */

#ifndef NEIGHBOR_TABLE_H_
#define NEIGHBOR_TABLE_H_

#include "ns3/object-base.h"
#include "ns3/core-module.h"

namespace ns3 {

class Neighbortable : public Object {
public:
	static TypeId GetTypeId();
	virtual TypeId GetInstanceTypeId() const{return GetTypeId();};
	Neighbortable();
	Ptr<Neighbortable> GetNext(){return m_next; };
	void SetNext(Ptr<Neighbortable> next){m_next = next; };
	uint32_t GetNeighborNode(){return m_neighbor_node; };
	void SetNeighborNode(uint32_t neighbor){m_neighbor_node = neighbor; };
	void SetTime(Time time){m_update_time = time; } ;
	Time GetTime(){ return m_update_time; };

private:
	uint32_t m_neighbor_node;
	Time m_update_time;
	uint8_t m_rssi;
	Ptr<Neighbortable> m_next;
	Ptr<Neighbortable> m_prv;
};

}  // namespace ns3

#endif /* NEIGHBOR_TABLE_H_ */
