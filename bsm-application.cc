/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 North Carolina State University
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Scott E. Carpenter <scarpen@ncsu.edu>
 *
 */

#include "ns3/bsm-application.h"
#include "ns3/log.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/constant-acceleration-mobility-model.h"
#include "ns3/highway-mobility-model.h"
#include "ns3/bsm-header.h"
#include "ns3/neighbor-table.h"
#include "ns3/neighbor-table-header.h"

NS_LOG_COMPONENT_DEFINE("BsmApplication");

namespace ns3 {

// (Arbitrary) port for establishing socket to transmit WAVE BSMs
int BsmApplication::wavePort = 9080;

//Time in ms after which old nodes in the neighbor table will be removed
int BsmApplication::pruneTime = 150;

NS_OBJECT_ENSURE_REGISTERED(BsmApplication);

TypeId BsmApplication::GetTypeId(void) {
	static TypeId tid =
			TypeId("ns3::BsmApplication").SetParent<Application>().SetGroupName(
					"Wave").AddConstructor<BsmApplication>();
	return tid;
}

BsmApplication::BsmApplication() :
		m_waveBsmStats(0), m_txSafetyRangesSq(), m_TotalSimTime(Seconds(10)), m_wavePacketSize(
				200), m_numWavePackets(1), m_waveInterval(MilliSeconds(100)), m_gpsAccuracyNs(
				10000), m_adhocTxInterfaces(0), m_nodesMoving(0), m_unirv(0), m_nodeId(
				0), m_chAccessMode(0), m_txMaxDelay(MilliSeconds(10)), m_prevTxDelay(
				MilliSeconds(0)), m_neighbor_table_header(NULL), m_neighbor_table_trailer(
		NULL) {
	NS_LOG_FUNCTION(this);
}

BsmApplication::~BsmApplication() {
	NS_LOG_FUNCTION(this);
}

void BsmApplication::DoDispose(void) {
	NS_LOG_FUNCTION(this);

	// chain up
	Application::DoDispose();
}

// Application Methods
void BsmApplication::StartApplication() // Called at time specified by Start
{
	NS_LOG_FUNCTION(this);
	std::cout << "BSMApplication start at node " << m_nodeId << std::endl;

	// setup generation of WAVE BSM messages
	Time waveInterPacketInterval = m_waveInterval;

	// BSMs are not transmitted for the first second
	Time startTime = Seconds(1.0);
	// total length of time transmitting WAVE packets
	Time totalTxTime = m_TotalSimTime - startTime;
	// total WAVE packets needing to be sent
	m_numWavePackets = (uint32_t) (totalTxTime.GetDouble()
			/ m_waveInterval.GetDouble());

	TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

	// every node broadcasts WAVE BSM to potentially all other nodes
	Ptr<Socket> recvSink = Socket::CreateSocket(GetNode(m_nodeId), tid);
	recvSink->SetRecvCallback(
			MakeCallback(&BsmApplication::ReceiveWavePacket, this));
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(),
			wavePort);
	recvSink->Bind(local);
	recvSink->BindToNetDevice(GetNetDevice(m_nodeId));
	recvSink->SetAllowBroadcast(true);

	// dest is broadcast address
	InetSocketAddress remote = InetSocketAddress(Ipv4Address("255.255.255.255"),
			wavePort);
	recvSink->Connect(remote);

	// Transmission start time for each BSM:
	// We assume that the start transmission time
	// for the first packet will be on a ns-3 time
	// "Second" boundary - e.g., 1.0 s.
	// However, the actual transmit time must reflect
	// additional effects of 1) clock drift and
	// 2) transmit delay requirements.
	// 1) Clock drift - clocks are not perfectly
	// synchronized across all nodes.  In a VANET
	// we assume all nodes sync to GPS time, which
	// itself is assumed  accurate to, say, 40-100 ns.
	// Thus, the start transmission time must be adjusted
	// by some value, t_drift.
	// 2) Transmit delay requirements - The US
	// minimum performance requirements for V2V
	// BSM transmission expect a random delay of
	// +/- 5 ms, to avoid simultanous transmissions
	// by all vehicles congesting the channel.  Thus,
	// we need to adjust the start trasmission time by
	// some value, t_tx_delay.
	// Therefore, the actual transmit time should be:
	// t_start = t_time + t_drift + t_tx_delay
	// t_drift is always added to t_time.
	// t_tx_delay is supposed to be +/- 5ms, but if we
	// allow negative numbers the time could drift to a value
	// BEFORE the interval start time (i.e., at 100 ms
	// boundaries, we do not want to drift into the
	// previous interval, such as at 95 ms.  Instead,
	// we always want to be at the 100 ms interval boundary,
	// plus [0..10] ms tx delay.
	// Thus, the average t_tx_delay will be
	// within the desired range of [0..10] ms of
	// (t_time + t_drift)

	// WAVE devices sync to GPS time
	// and all devices would like to begin broadcasting
	// their safety messages immediately at the start of
	// the CCH interval.  However, if all do so, then
	// significant collisions occur.  Thus, we assume there
	// is some GPS sync accuracy on GPS devices,
	// typically 40-100 ns.
	// Get a uniformly random number for GPS sync accuracy, in ns.
	Time tDrift = NanoSeconds(m_unirv->GetInteger(0, m_gpsAccuracyNs));

	// When transmitting at a default rate of 10 Hz,
	// the subsystem shall transmit every 100 ms +/-
	// a random value between 0 and 5 ms. [MPR-BSMTX-TXTIM-002]
	// Source: CAMP Vehicle Safety Communications 4 Consortium
	// On-board Minimum Performance Requirements
	// for V2V Safety Systems Version 1.0, December 17, 2014
	// max transmit delay (default 10ms)
	// get value for transmit delay, as number of ns
	uint32_t d_ns = static_cast<uint32_t>(m_txMaxDelay.GetInteger());
	// convert random tx delay to ns-3 time
	// see note above regarding centering tx delay
	// offset by 5ms + a random value.
	Time txDelay = NanoSeconds(m_unirv->GetInteger(0, d_ns));
	m_prevTxDelay = txDelay;

	Time txTime = startTime + tDrift + txDelay;
	// schedule transmission of first packet
	Simulator::ScheduleWithContext(recvSink->GetNode()->GetId(), txTime,
			&BsmApplication::GenerateWaveTraffic, this, recvSink,
			m_wavePacketSize, m_numWavePackets, waveInterPacketInterval,
			m_nodeId);
}

void BsmApplication::StopApplication() // Called at time specified by Stop
{
	NS_LOG_FUNCTION(this);
}

void BsmApplication::Setup(Ipv4InterfaceContainer & i, int nodeId,
		Time totalTime,
		uint32_t wavePacketSize, // bytes
		Time waveInterval, double gpsAccuracyNs,
		std::vector<double> rangesSq,           // m ^2
		Ptr<WaveBsmStats> waveBsmStats, std::vector<int> * nodesMoving,
		int chAccessMode, Time txMaxDelay) {
	NS_LOG_FUNCTION(this);

	m_unirv = CreateObject<UniformRandomVariable>();

	m_TotalSimTime = totalTime;
	m_wavePacketSize = wavePacketSize;
	m_waveInterval = waveInterval;
	m_gpsAccuracyNs = gpsAccuracyNs;
	int size = rangesSq.size();
	m_waveBsmStats = waveBsmStats;
	m_nodesMoving = nodesMoving;
	m_chAccessMode = chAccessMode;
	m_txSafetyRangesSq.clear();
	m_txSafetyRangesSq.resize(size, 0);

	for (int index = 0; index < size; index++) {
		// stored as square of value, for optimization
		m_txSafetyRangesSq[index] = rangesSq[index];
	}

	m_adhocTxInterfaces = &i;
	m_nodeId = nodeId;
	m_txMaxDelay = txMaxDelay;
}

void BsmApplication::GenerateWaveTraffic(Ptr<Socket> socket, uint32_t pktSize,
		uint32_t pktCount, Time pktInterval, uint32_t sendingNodeId) {
	NS_LOG_FUNCTION(this);
	//std::cout << Simulator::Now().GetMilliSeconds()
	//		<< " Generate BSM Traffic at " << m_nodeId << std::endl;
	// more packets to send?
	if (pktCount > 0) {
		// for now, we cannot tell if each node has
		// started mobility.  so, as an optimization
		// only send if  this node is moving
		// if not, then skip
		int txNodeId = sendingNodeId;
		Ptr<Node> txNode = GetNode(txNodeId);
		Ptr<HighwayMobilityModel> txPosition = txNode->GetObject<
				HighwayMobilityModel>();
		NS_ASSERT(txPosition != 0);

		//std::cout<<"m_nodesMoving "<<m_nodesMoving->size()<<std::endl;
		int senderMoving = m_nodesMoving->at(txNodeId);
		if (senderMoving != 0) {
			// send it!
			//BSM packet is sent to RV from HV but this packet contains nothing
			Ptr<Packet> packet = Create<Packet>(pktSize);
			//socket->Send(Create<Packet>(pktSize));

			/*
			 * Blob part
			 * Message count ~ 1 Byte
			 * Temporary Id ~ 4 Bytes
			 * DSecond(SecMark) ~ 2 Bytes
			 * Latitude ~ 4 Bytes
			 * Longitude ~ 4 Bytes
			 * elevation ~ 2 Bytes
			 * Positional Accuracy(GPS) ~ 4 Bytes
			 * Transmission Speed ~ 2 Bytes
			 * Heading ~ 2 Bytes
			 * Steering wheel angle ~ 1 Byte
			 * Longitudinal Acceleration ~ 2 Bytes
			 * Latteral Acceleration ~ 2 Bytes
			 * Vertical Acceleration ~ 1 Byte
			 * Acceleration Yaw Rate ~ 2 Bytes
			 * Brake System Status(Brake) ~ 2 Bytes
			 * Width ~ 3 Bytes
			 * Length ~ 3 Bytes
			 *
			 * Additional 1 Byte Lane information
			 *
			 */
			//std::cout<<"TX Node "<<m_nodeId<<std::endl;
			BSMHeader bsmheader;
			bsmheader.SetSequenceNumber(pktCount);
			bsmheader.SetNodeId(txNodeId);
			bsmheader.SetTime(Simulator::Now().GetMilliSeconds());
			Vector position = txPosition->GetPosition();
			bsmheader.SetLattitude(position.x);
			bsmheader.SetLongitude(position.y);
			bsmheader.SetElevation(position.z);
			bsmheader.SetGPSAccuracy(m_gpsAccuracyNs);
			bsmheader.SetSpeed(txPosition->GetVelocity().x);
			bsmheader.SetHeading(txPosition->GetHeading());
			bsmheader.setSteeringAngle(txPosition->GetHeading());
			Vector acceleration = txPosition->GetAcceleration();
			bsmheader.SetLongitudeAcceleration(acceleration.x);
			bsmheader.SetLattitudeAcceleration(acceleration.y);
			bsmheader.SetVerticleAcceleration(acceleration.z);
			bsmheader.SetYawRate(2);
			bsmheader.SetBrake(txPosition->GetBrake());
			bsmheader.SetWidth(txPosition->GetWidth());
			bsmheader.Setlength(txPosition->GetLength());
			bsmheader.SetLane(txPosition->GetLane());

			//bsmheader.Print(std::cout);
			packet->AddHeader(bsmheader);

			//Add Neighbor table in the BSM packet
			NeighbortableHeader neighborheader;
			neighborheader.SetSrc(m_nodeId);
			neighborheader.SetTime(Simulator::Now().GetMilliSeconds());
			PruneNeighborTable();
			Ptr<Neighbortable> tmp = m_neighbor_table_header;
			int size = 0;
			while (tmp != NULL) {
				if ((Simulator::Now().GetMilliSeconds()
						- tmp->GetTime().GetMilliSeconds()) < 100) {
					neighborheader.AddNeighbour(tmp->GetNeighborNode());
					size++;
				}
				tmp = tmp->GetNext();
			}
			neighborheader.SetNeighborTableSize(size);
			//neighborheader.Print(std::cout);
			packet->AddHeader(neighborheader);

			socket->Send(packet);
			//PrintNeighborTable();
			if (txPosition->GetPosition().x > 2500
					|| txPosition->GetPosition().x < 0)
				std::cout << "wrong position " << Simulator::Now() << " "
						<< m_nodeId << std::endl;

			// count it
			// it maintain one single instance of wavebsmstats hence this variable gives cumulative value
			m_waveBsmStats->IncTxPktCount();//Number of bsm packet transmitted
			m_waveBsmStats->IncTxByteCount(pktSize); // Number of bytes data transmitted
			int wavePktsSent = m_waveBsmStats->GetTxPktCount();
			if ((m_waveBsmStats->GetLogging() != 0)
					&& ((wavePktsSent % 1000) == 0)) {
				NS_LOG_UNCOND("Sending WAVE pkt # " << wavePktsSent);
			}

			// find other nodes within range that would be
			// expected to receive this broadbast
			int nRxNodes = m_adhocTxInterfaces->GetN();
			for (int i = 0; i < nRxNodes; i++) {
				Ptr<Node> rxNode = GetNode(i);
				int rxNodeId = rxNode->GetId();

				if (rxNodeId != txNodeId) {
					Ptr<MobilityModel> rxPosition = rxNode->GetObject<
							MobilityModel>();
					NS_ASSERT(rxPosition != 0);
					// confirm that the receiving node
					// has also started moving in the scenario
					// if it has not started moving, then
					// it is not a candidate to receive a packet
					int receiverMoving = m_nodesMoving->at(rxNodeId);
					if (receiverMoving == 1) {
						double distSq =
								MobilityHelper::GetDistanceSquaredBetween(
										txNode, rxNode);
						if (distSq > 0.0) {

							// dest node within range?
							//check and maintain if the packet is reached in distances mentioned in different ranges
							int rangeCount = m_txSafetyRangesSq.size();
							for (int index = 1; index <= rangeCount; index++) {
								if (distSq <= m_txSafetyRangesSq[index - 1]) {
									// we should expect dest node to receive broadcast pkt
									m_waveBsmStats->IncExpectedRxPktCount(
											index);
								}
							}
						}
					}
				}
			}
		}

		// every BSM must be scheduled with a tx time delay
		// of +/- (5) ms.  See comments in StartApplication().
		// we handle this as a tx delay of [0..10] ms
		// from the start of the pktInterval boundary
		uint32_t d_ns = static_cast<uint32_t>(m_txMaxDelay.GetInteger());
		Time txDelay = NanoSeconds(m_unirv->GetInteger(0, d_ns));

		// do not want the tx delay to be cumulative, so
		// deduct the previous delay value.  thus we adjust
		// to schedule the next event at the next pktInterval,
		// plus some new [0..10] ms tx delay
		Time txTime = pktInterval - m_prevTxDelay + txDelay;
		m_prevTxDelay = txDelay;

		Simulator::ScheduleWithContext(socket->GetNode()->GetId(), txTime,
				&BsmApplication::GenerateWaveTraffic, this, socket, pktSize,
				pktCount - 1, pktInterval, socket->GetNode()->GetId());
	} else {
		socket->Close();
	}
}

void BsmApplication::ReceiveWavePacket(Ptr<Socket> socket) {
	NS_LOG_FUNCTION(this);

	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		Ptr<Node> rxNode = socket->GetNode();

		SocketAddressTag tag;
		bool found;
		found = packet->PeekPacketTag(tag);

		if (found) {
			InetSocketAddress addr = InetSocketAddress::ConvertFrom(
					tag.GetAddress());
			int nodes = m_adhocTxInterfaces->GetN();
			for (int i = 0; i < nodes; i++) {
				if (addr.GetIpv4() == m_adhocTxInterfaces->GetAddress(i)) {
					Ptr<Node> txNode = GetNode(i);
					HandleReceivedBsmPacket(packet, txNode, rxNode);
				}
			}
		}
	}
}
double max = 0.0;
void BsmApplication::HandleReceivedBsmPacket(Ptr<Packet> packet,
		Ptr<Node> txNode, Ptr<Node> rxNode) {
	NS_LOG_FUNCTION(this);
//std::cout << m_nodeId << " received BSM packet from " << txNode->GetId()
//		<< std::endl;
	m_waveBsmStats->IncRxPktCount();

//Ptr<MobilityModel> rxPosition = rxNode->GetObject<MobilityModel>();
	Ptr<HighwayMobilityModel> rxPosition = rxNode->GetObject<
			HighwayMobilityModel>();
	NS_ASSERT(rxPosition != 0);

	NeighbortableHeader neighborheader;
	packet->RemoveHeader(neighborheader);
//neighborheader.Print(std::cout);
	std::cout << " Link Asymmetry " << rxNode->GetId() << "->"
			<< txNode->GetId() << " " << neighborheader.isAckPresent(m_nodeId)
			<< std::endl;

	BSMHeader bsmheader;
	packet->RemoveHeader(bsmheader);

	AddInNeighborTable(txNode->GetId());
//std::cout<<"RX Node "<<m_nodeId<<std::endl;
//bsmheader.Print(std::cout);
// confirm that the receiving node
// has also started moving in the scenario
// if it has not started moving, then
// it is not a candidate to receive a packet
	int rxNodeId = rxNode->GetId();
	int receiverMoving = m_nodesMoving->at(rxNodeId);
	if (receiverMoving == 1) {
		double rxDistSq = MobilityHelper::GetDistanceSquaredBetween(rxNode,
				txNode);
		if (rxDistSq > 0.0) {
			// Max distance with default Tx power is 159.0
			/*if (sqrt(rxDistSq) > max) {
			 max = sqrt(rxDistSq);
			 std::cout << "dist : " << max << std::endl;
			 }*/
			int rangeCount = m_txSafetyRangesSq.size();
			for (int index = 1; index <= rangeCount; index++) {
				if (rxDistSq <= m_txSafetyRangesSq[index - 1]) {
					m_waveBsmStats->IncRxPktInRangeCount(index);
				}
			}
		}
	}
}

int64_t BsmApplication::AssignStreams(int64_t streamIndex) {
	NS_LOG_FUNCTION(this);

	NS_ASSERT(m_unirv);  // should be set by Setup() prevoiusly
	m_unirv->SetStream(streamIndex);

	return 1;
}

Ptr<Node> BsmApplication::GetNode(int id) {
	NS_LOG_FUNCTION(this);

	std::pair<Ptr<Ipv4>, uint32_t> interface = m_adhocTxInterfaces->Get(id);
	Ptr<Ipv4> pp = interface.first;
	Ptr<Node> node = pp->GetObject<Node>();

	return node;
}

Ptr<NetDevice> BsmApplication::GetNetDevice(int id) {
	NS_LOG_FUNCTION(this);

	std::pair<Ptr<Ipv4>, uint32_t> interface = m_adhocTxInterfaces->Get(id);
	Ptr<Ipv4> pp = interface.first;
	Ptr<NetDevice> device = pp->GetObject<NetDevice>();

	return device;
}
void BsmApplication::PrintNeighborTable() {
	Ptr<Neighbortable> tmp = m_neighbor_table_header;
	if (tmp == NULL) {
		std::cout << Simulator::Now().GetMilliSeconds()
				<< " Empty Neighbor table At " << m_nodeId << std::endl;
	} else {
		std::cout << Simulator::Now().GetMilliSeconds() << " At " << m_nodeId
				<< " Neighbors are ";
		while (tmp != NULL) {
			std::cout << tmp->GetNeighborNode() << " update Time "
					<< tmp->GetTime().GetMilliSeconds() << ", ";
			tmp = tmp->GetNext();
		}
		std::cout << std::endl;
	}
}
void BsmApplication::AddInNeighborTable(uint16_t neighbor) {

	Ptr<Neighbortable> tmp = m_neighbor_table_header;
	while (tmp != NULL) {
		if (tmp->GetNeighborNode() == neighbor) {
			tmp->SetTime(Simulator::Now());
			return;
		}
		tmp = tmp->GetNext();
	}

//if not present then add the new node in front
	Ptr<Neighbortable> neighbor_node = CreateObject<Neighbortable>();
	neighbor_node->SetNeighborNode(neighbor);
	neighbor_node->SetTime(Simulator::Now());

	neighbor_node->SetNext(m_neighbor_table_header);
	m_neighbor_table_header = neighbor_node;

//std::cout<<"Neighbour added "<<m_neighbor_table_header->GetNeighborNode()<<" At "<<m_nodeId<<std::endl;

}

void BsmApplication::PruneNeighborTable() {
	Ptr<Neighbortable> tmp = m_neighbor_table_header;
	Ptr<Neighbortable> prv = NULL;

	while (tmp != NULL) {
		if ((Simulator::Now().GetMilliSeconds()
				- tmp->GetTime().GetMilliSeconds())
				>= BsmApplication::pruneTime) {
			//std::cout<<"remove neighbor "<<tmp->GetNeighborNode()<<std::endl;
			if (prv == NULL) {
				tmp = tmp->GetNext();
				m_neighbor_table_header = tmp;
			} else {
				prv->SetNext(tmp->GetNext());
				tmp = tmp->GetNext();
			}
		} else {
			prv = tmp;
			tmp = tmp->GetNext();
		}
	}
}

} // namespace ns3
