/*
 * neighbor-table-header.cc
 *
 *  Created on: Jul 12, 2017
 *      Author: moonstar
 */

#include "neighbor-table-header.h"
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/pointer.h>
#include <ns3/node.h>
#include <ns3/object-base.h>

namespace ns3 {
NS_LOG_COMPONENT_DEFINE("NeighbortableHeader");

NS_OBJECT_ENSURE_REGISTERED(NeighbortableHeader);

NeighbortableHeader::NeighbortableHeader(): m_neighbor_size(0), m_neighbors() {

}
TypeId NeighbortableHeader::GetTypeId() {
	static TypeId tid =
			TypeId("NS3::NeighbortableHeader").SetParent<Header>().SetGroupName(
					"Wave").AddConstructor<NeighbortableHeader>();

	return tid;
}
TypeId NeighbortableHeader::GetInstanceTypeId() const {
	return GetTypeId();
}
void NeighbortableHeader::Print(std::ostream& os) const {
	std::cout << "NeighbortableHeader[ src : " << (int) m_src_id << " Time : "
			<< (int) m_time << " Size: " << (int) m_neighbor_size << " (";
	for (std::vector<uint32_t>::size_type it = 0;
			it != m_neighbors.size(); it++) {
		std::cout << m_neighbors[it] << " ";
	}
	std::cout << ") ]" << std::endl;
}
uint32_t NeighbortableHeader::GetSerializedSize() const {
	uint8_t size = (4 + 4 + 2 + 4 * m_neighbor_size);
	return size;
}
void NeighbortableHeader::Serialize(Buffer::Iterator start) const {
	start.WriteU32(m_src_id);
	start.WriteU32(m_time);
	start.WriteU16(m_neighbor_size);
	for (std::vector<uint32_t>::size_type it = 0;
			it < m_neighbors.size(); it++) {
		start.WriteU32(m_neighbors[it]);
	}

}
uint32_t NeighbortableHeader::Deserialize(Buffer::Iterator start) {
	m_src_id = start.ReadU32();
	m_time = start.ReadU32();
	m_neighbor_size = start.ReadU16();
	for (int i = 0; i < m_neighbor_size; i++) {
		m_neighbors.push_back(start.ReadU32());
	}
	return GetSerializedSize();
}
void NeighbortableHeader::AddNeighbour(uint32_t nodeId) {
	m_neighbors.push_back(nodeId);
}
void NeighbortableHeader::SetSrc(uint32_t src) {
	m_src_id = src;
}
void NeighbortableHeader::SetTime(uint32_t time) {
	m_time = time;
}
void NeighbortableHeader::SetNeighborTableSize(uint16_t size) {
	m_neighbor_size = size;
}
bool NeighbortableHeader::isAckPresent(uint32_t nodeId){
	for(std::vector<uint32_t>::size_type i = 0; i < m_neighbors.size(); i++){
		if(m_neighbors[i] == nodeId){
			return true;
		}
	}
	return false;
}
}  // namespace ns3

