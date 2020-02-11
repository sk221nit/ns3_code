/*
 * neighbor-table-header.h
 *
 *  Created on: Jul 12, 2017
 *      Author: moonstar
 */

#ifndef NEIGHBOR_TABLE_HEADER_H_
#define NEIGHBOR_TABLE_HEADER_H_

#include <ns3/header.h>

namespace ns3 {

class NeighbortableHeader: public Header {

public:
	NeighbortableHeader();
	static TypeId GetTypeId();
	virtual TypeId GetInstanceTypeId() const;
	virtual void Print(std::ostream& os) const;
	virtual uint32_t GetSerializedSize() const;
	virtual void Serialize(Buffer::Iterator start) const;
	virtual uint32_t Deserialize(Buffer::Iterator start);
	void AddNeighbour(uint32_t nodeId);
	void SetSrc(uint32_t src);
	void SetTime(uint32_t time);
	void SetNeighborTableSize(uint16_t size);
	bool isAckPresent(uint32_t nodeId);

private:
	uint32_t m_src_id;
	uint32_t m_time;
	uint16_t m_neighbor_size;
	std::vector<uint32_t> m_neighbors;
};

}  // namespace ns3

#endif /* NEIGHBOR_TABLE_HEADER_H_ */
