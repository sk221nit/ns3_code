/*
 * bsm-header.h
 *
 *  Created on: May 22, 2017
 *      Author: Saurabh Kumar
 *      Email: saurabh.cbnu.ac.kr
 */

#ifndef BSM_HEADER_H_
#define BSM_HEADER_H_

#include <ns3/header.h>

namespace ns3 {

/* BSM Packet contents
 * Message sequence number to future use of packet filtering and
 */
class BSMHeader: public Header {
public:
	BSMHeader();
	static TypeId GetTypeId();
	virtual TypeId GetInstanceTypeId() const;
	virtual void Print(std::ostream& os) const;
	virtual uint32_t GetSerializedSize() const;
	virtual void Serialize(Buffer::Iterator start) const;
	virtual uint32_t Deserialize(Buffer::Iterator start);


	void SetSequenceNumber(uint8_t seq);
	uint8_t GetSequenceNumber(void) const;
	void SetNodeId(uint32_t id);
	uint32_t GetNodeId(void) const;
	void SetTime(uint16_t millisec);
	uint16_t GetTime(void) const;
	void SetLattitude(double x);
	double GetLattitude(void) const;
	void SetLongitude(double y);
	double GetLongitude(void) const;
	void SetElevation(uint16_t elev);
	uint16_t GetElevation(void) const;
	void SetGPSAccuracy(double accuracy);
	double GetGPSAccuracy(void) const;
	void SetSpeed(uint16_t speed);
	uint16_t GetSpeed(void) const;

	void SetHeading(uint16_t heading);
	uint16_t GetHeading(void) const;

	void setSteeringAngle(uint8_t angle);
	uint8_t GetSteeringAngle(void) const;

	void SetLongitudeAcceleration(uint16_t acc_x);
	uint16_t GetLongitudeAcceleration(void) const;

	void SetLattitudeAcceleration(uint16_t acc_y);
	uint16_t GetLattitudeAcceleration(void) const;

	void SetVerticleAcceleration(uint8_t acc_z);
	uint8_t GetVerticleAcceleration(void) const;

	void SetYawRate(uint16_t yaw);
	uint16_t GetYawRate(void) const;

	void SetBrake(uint16_t brake);
	uint16_t GetBrake(void) const;

	void SetWidth(uint16_t width);
	uint16_t GetWidth(void) const;
	void Setlength(uint16_t length);
	uint16_t GetLength(void) const;

	void SetLane(uint8_t lane);
	uint8_t GetLane(void) const;



private:
	/*
	 * SAE J2735 USDOT Basic safety message details
	 * Contents
	 *
	 * DSRC Message Id ~ 1 Byte (Type of Message BSM or WSA etc)
	 *
	 * Blob part
	 *                                               SAE J2735 BSM Packet structure
	 *
	 *              0           1           2           3           4           5           6           7           8
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |    Type   |    Seq    |               Temp ID                         |     Time              |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |                     Latitude                  |                Longitude                      |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |       Elevation       |            GPS Accuracy                       |      Speed            |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |       Heading         |   Angle   |    Long Acc.          |    Lat. Acc.          | Ver. Acc. |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |       Yaw Rate        |         Brake         |           Width                   |  Empty    |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |              Length               |  lane     |                Empty                          |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *              |                                                                                               |
	 *              |                               Optional Extra Data                                             |
	 *              |                       ( Safety Extension & Vehicle Status )                                   |
	 *              |                                                                                               |
	 *              +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	 *
	 *
	 *
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
	 * Extra data
	 * Safety Extension ~ optional
	 * Vehicle Status ~ optional
	 * 		Vehicle height ~ 1 Byte
	 * 		Bumper Height
	 * 			front ~ 1 Byte
	 * 			Rear ~ 1 Byte
	 * 		Vehicle Weight(mass) ~ 1 Byte
	 * 		trailer weight ~ 1 Byte
	 * 		Vehicle Type ~ 1 Byte
	 *
	 *
	 * Addition Lane information 1 Byte
	 */
	// Message count 0~255
	uint8_t m_sequence_number; //sequence number of BSM packet generated by the node, associated with individual node. This will help in message dissemination.
	// Temporary Id( NS3 Does not use Security hence Temp id is same as node ID
	uint32_t m_nodeId;//Node Id of the sender, since this is a application and receiver does not provide the nodeId of Transmitter directly.
	// DSecond ( Time of packet generation )
	uint16_t m_time;
	// 4 Byte x value
	double m_lattitude;	//x
	// 4 Byte y value
	double m_longitude;	//y
	// 2 Bytes Z information
	uint16_t m_elev;	//z
	// 4 Byte GPS Accuracy information
	double m_accuracy;	//Accuracy of the gps location tracking
	// 2 Byte speed (direction is contained in steering angle)
	uint16_t m_speed;	//speed of the vehicle
	// Heading 2 byte information
	uint16_t m_heading;	//direction of the car. we assume x, y co-ordinate system and corresponding angle for heading.(0~359)
	// 1 Byte Steering wheel angle
	uint8_t m_steering_angle;

	uint16_t m_acceleration_long;
	uint16_t m_acceleration_lati;
	uint8_t m_acceleration_verti;
	uint16_t m_yaw_rate;

	uint16_t m_brake;//current brake level//it will define the stopping distance. This will be helpful in forward error collision warning system

	uint16_t m_width;	//width of the vehicle
	uint8_t m_width_dummy;// To make total width 3 Byte
	uint16_t m_length;	//length of the vehicle
	uint8_t m_length_dummy; // To Make the total length 3 Byte


	//double m_acceleration;	//Acceleration of the vehicle

	uint8_t m_lane;

	uint8_t m_vehicle_height;

	uint8_t m_bumper_height_front;
	uint8_t m_bumper_height_rear;

	//mass
	uint8_t m_vehicle_weight;

	uint8_t m_trailer_weight;

	uint8_t m_vehicle_type;

};

}  // namespace ns3

#endif /* BSM_HEADER_H_ */
